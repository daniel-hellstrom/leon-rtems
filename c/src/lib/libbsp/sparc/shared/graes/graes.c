/* GRAES AES dma driver 
 *
 * --------------------------------------------------------------------------
 *  --  This file is a part of GAISLER RESEARCH source code.
 *  --  Copyright (C) 2009, Aeroflex Gaisler AB - all rights reserved.
 *  --
 *  -- ANY USE OR REDISTRIBUTION IN PART OR IN WHOLE MUST BE HANDLED IN
 *  -- ACCORDANCE WITH THE GAISLER LICENSE AGREEMENT AND MUST BE APPROVED
 *  -- IN ADVANCE IN WRITING.
 *  --
 *  -- BY DEFAULT, DISTRIBUTION OR DISCLOSURE IS NOT PERMITTED.
 *  -------------------------------------------------------------------------- 
 *
 *  2010-06-29, Konrad Eisele <konrad@gaisler.com>
 *    GRAES DMA driver based on GRTM driver 
 *
 *  2008-12-11, Daniel Hellstrom <daniel@gaisler.com>
 *    Converted to support driver manager
 *
 *  2007-04-17, Daniel Hellstrom <daniel@gaisler.com>
 *    New driver in sparc shared directory. 
 *
 */


#include <bsp.h>
#include <rtems/libio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <rtems/bspIo.h>

#include <drvmgr/drvmgr.h>
#include <ambapp.h>
#include <drvmgr/ambapp_bus.h>
#include <graes.h>

#define REMOTE_DESCRIPTORS

#ifndef IRQ_GLOBAL_PREPARE
 #define IRQ_GLOBAL_PREPARE(level) rtems_interrupt_level level
#endif

#ifndef IRQ_GLOBAL_DISABLE
 #define IRQ_GLOBAL_DISABLE(level) rtems_interrupt_disable(level)
#endif

#ifndef IRQ_GLOBAL_ENABLE
 #define IRQ_GLOBAL_ENABLE(level) rtems_interrupt_enable(level)
#endif

/*
#define DEBUG
#define DEBUGFUNCS
*/

#include <debug_defs.h>

/* GRAES register map */
struct graes_regs {
	volatile unsigned int	dma_ctrl;	/* DMA Control Register (0x00) */
	volatile unsigned int	dma_status;	/* DMA Status Register (0x04) */
	volatile unsigned int	dma_bd;		/* DMA Descriptor Pointer Register (0x08) */
	volatile unsigned int	d0;	        /*  */	

	volatile unsigned int	d1;	/*  */
	volatile unsigned int	d2;	/*  */

	int unused0[(0x80-0x18)/4];

};

#define GRAES_BDAR_ENTRIES 32
#define GRAES_BDAR_SIZE    (GRAES_BDAR_ENTRIES * sizeof(struct graes_bd))

/* DMA Control Register (0x00) */
#define GRAES_DMA_CTRL_EN_BIT	0
#define GRAES_DMA_CTRL_IE_BIT	1

#define GRAES_DMA_CTRL_EN	(1<<GRAES_DMA_CTRL_EN_BIT)
#define GRAES_DMA_CTRL_IE	(1<<GRAES_DMA_CTRL_IE_BIT)

/* GRAES transmit descriptor (GRAES_BDAR_SIZE Alignment need) */
struct graes_bd {
	volatile unsigned int	ctrl;
	union {
		unsigned int		d[5];
		struct {
			unsigned int		in;
			unsigned int		out;
			unsigned int		iv;
			unsigned int		key;
			unsigned int		next;
		};
	} u;
};

#define GRAES_BD_EN_BIT		0
#define GRAES_BD_IE_BIT		1
#define GRAES_BD_WR_BIT		2
#define GRAES_BD_KEY_BIT        7
#define GRAES_BD_IV_BIT		6
#define GRAES_BD_OUT_BIT        5

#define GRAES_BD_EN		(1<<GRAES_BD_EN_BIT)
#define GRAES_BD_WR		(1<<GRAES_BD_WR_BIT)
#define GRAES_BD_IE		(1<<GRAES_BD_IE_BIT)
#define GRAES_BD_KEY		(1<<GRAES_BD_KEY_BIT)
#define GRAES_BD_IV		(1<<GRAES_BD_IV_BIT)
#define GRAES_BD_OUT		(1<<GRAES_BD_OUT_BIT)

/* Load register */

#define READ_REG(address)	(*(volatile unsigned int *)address)

/* Driver functions */
static rtems_device_driver graes_initialize(rtems_device_major_number  major, rtems_device_minor_number  minor, void *arg);
static rtems_device_driver graes_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver graes_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver graes_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver graes_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver graes_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);

#define GRAES_DRIVER_TABLE_ENTRY { graes_initialize, graes_open, graes_close, graes_read, graes_write, graes_ioctl }

static rtems_driver_address_table graes_driver = GRAES_DRIVER_TABLE_ENTRY;

/* Structure that connects BD with SoftWare Frame */
struct graes_ring {
	struct graes_ring	*next;
	struct graes_bd		*bd;
	struct graes_block	*frm;
};

struct graes_priv {
	struct rtems_drvmgr_dev_info	*dev;		/* Driver manager device */
	char			devName[32];	/* Device Name */
	struct graes_regs	*regs;
	int			irq;
	int			minor;

	int			open;
	int			running;

	struct graes_bd		*bds;
	void			*_bds;

	/* Interrupt generation */
	int			enable_cnt_curr;/* Down counter, when 0 the interrupt bit is set for next descriptor */
	volatile int		handling_transmission;	/* Tells ISR if user are active changing descriptors/queues */

	struct graes_ring 	*_ring;		/* Root of ring */
	struct graes_ring 	*ring;		/* Next ring to use for new frames to be transmitted */
	struct graes_ring 	*ring_end;	/* Oldest activated ring used */

	/* Collections of frames Ready to sent/ Scheduled for transmission/Sent 
	 * frames waiting for the user to reclaim 
	 */
	struct graes_list	ready;		/* Frames Waiting for free BDs */
	struct graes_list	scheduled;	/* Frames in BDs beeing transmitted */
	struct graes_list	sent;		/* Sent Frames waiting for user to reclaim and reuse */

	/* Number of frames in the lists */
	int			ready_cnt;	/* Number of ready frames */
	int			scheduled_cnt;	/* Number of scheduled frames */
	int			sent_cnt;	/* Number of sent frames */

	struct graes_ioc_hw	hw_avail;	/* Hardware support available */
	struct graes_ioc_config	config;
	struct graes_ioc_stats	stats;

	rtems_id		sem_rx;
};

/* Prototypes */
static void *graes_memalign(unsigned int boundary, unsigned int length, void *realbuf);
static void graes_hw_reset(struct graes_priv *pDev);
static void graes_interrupt(int irq, void *arg);

/* Common Global Variables */
static rtems_id graes_dev_sem;
static int graes_driver_io_registered = 0;
static rtems_device_major_number graes_driver_io_major = 0;

/******************* Driver manager interface ***********************/

/* Driver prototypes */
static int graes_register_io(rtems_device_major_number *m);
static int graes_device_init(struct graes_priv *pDev);

static int graes_init2(struct rtems_drvmgr_dev_info *dev);
static int graes_init3(struct rtems_drvmgr_dev_info *dev);

static struct rtems_drvmgr_drv_ops graes_ops = 
{
	{NULL, graes_init2, graes_init3, NULL},
	NULL,
	NULL,
};

static struct amba_dev_id graes_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_GRAESDMA},
	{0, 0}		/* Mark end of table */
};

static struct amba_drv_info graes_drv_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_GRAES_ID,	/* Driver ID */
		"GRAES_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&graes_ops,
		0,				/* No devices yet */
	},
	&graes_ids[0]
};

void graes_register_drv (void)
{
	DBG("Registering GRAES driver\n");
	rtems_drvmgr_drv_register(&graes_drv_info.general);
}

static int graes_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct graes_priv *priv;

	DBG("GRAES[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);
	priv = dev->priv = malloc(sizeof(struct graes_priv));
	if ( !priv )
		return DRVMGR_NOMEM;
	memset(priv, 0, sizeof(*priv));
	priv->dev = dev;

	/* This core will not find other cores, so we wait for init2() */

	return DRVMGR_OK;
}

static int graes_init3(struct rtems_drvmgr_dev_info *dev)
{
	struct graes_priv *priv;
	char prefix[32];
	rtems_status_code status;

	priv = dev->priv;

	/* Do initialization */

	if ( graes_driver_io_registered == 0) {
		/* Register the I/O driver only once for all cores */
		if ( graes_register_io(&graes_driver_io_major) ) {
			/* Failed to register I/O driver */
			DBG("GRAES[%d] Failed to register I/O driver\n", dev->minor_drv);
			dev->priv = NULL;
			return DRVMGR_FAIL;
		}

		graes_driver_io_registered = 1;
	}

	/* I/O system registered and initialized 
	 * Now we take care of device initialization.
	 */
	if ( graes_device_init(priv) ) {
		DBG("GRAES[%d] Failed to call graes_device_init\n", dev->minor_drv);
		return DRVMGR_FAIL;
	}

	/* Get Filesystem name prefix */
	prefix[0] = '\0';
	if ( rtems_drvmgr_get_dev_prefix(dev, prefix) ) {
		/* Failed to get prefix, make sure of a unique FS name
		 * by using the driver minor.
		 */
		sprintf(priv->devName, "/dev/graes%d", dev->minor_drv);
	} else {
		/* Got special prefix, this means we have a bus prefix
		 * And we should use our "bus minor"
		 */
		sprintf(priv->devName, "/dev/%sgraes%d", prefix, dev->minor_bus);
	}

	DBG("GRAES: add dev %s\n",priv->devName);
	
	/* Register Device */
	status = rtems_io_register_name(priv->devName, graes_driver_io_major, dev->minor_drv);
	if (status != RTEMS_SUCCESSFUL) {
		return status;
	}

	return DRVMGR_OK;
}

/******************* Driver Implementation ***********************/

static int graes_register_io(rtems_device_major_number *m)
{
	rtems_status_code r;

	if ((r = rtems_io_register_driver(0, &graes_driver, m)) == RTEMS_SUCCESSFUL) {
		DBG("GRAES driver successfully registered, major: %d\n", *m);
	} else {
		switch(r) {
		case RTEMS_TOO_MANY:
			printk("GRAES rtems_io_register_driver failed: RTEMS_TOO_MANY\n");
			return -1;
		case RTEMS_INVALID_NUMBER:  
			printk("GRAES rtems_io_register_driver failed: RTEMS_INVALID_NUMBER\n");
			return -1;
		case RTEMS_RESOURCE_IN_USE:
			printk("GRAES rtems_io_register_driver failed: RTEMS_RESOURCE_IN_USE\n");
			return -1;
		default:
			printk("GRAES rtems_io_register_driver failed\n");
			return -1;
		}
	}
	return 0;
}

static int graes_device_init(struct graes_priv *pDev)
{
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)pDev->dev->businfo;
	if ( ambadev == NULL ) {
		return -1;
	}
	pnpinfo = &ambadev->info;
	pDev->irq = pnpinfo->irq;
	pDev->regs = (struct graes_regs *)pnpinfo->apb_slv->start;
	pDev->minor = pDev->dev->minor_drv;
	pDev->open = 0;
	pDev->running = 0;

	/* Create Binary RX Semaphore with count = 0 */
	if ( rtems_semaphore_create(rtems_build_name('G', 'P', 'R', '0' + pDev->minor),
		0,
		RTEMS_FIFO|RTEMS_SIMPLE_BINARY_SEMAPHORE|RTEMS_NO_INHERIT_PRIORITY|\
		RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING, 
		0,
		&pDev->sem_rx) != RTEMS_SUCCESSFUL ) {
		return -1;
	}

	/* Allocate Memory for Descriptors */
#ifdef REMOTE_DESCRIPTORS
	pDev->bds = 0xc0800000;
	pDev->_bds = 0xc0800000;
#else
	pDev->bds = (struct graes_bd *)graes_memalign(GRAES_BDAR_SIZE, GRAES_BDAR_SIZE, &pDev->_bds);
#endif
	if ( !pDev->bds ) {
		DBG("GRAES: Failed to allocate descriptor table\n");
		return -1;
	}
	memset(pDev->bds, 0, GRAES_BDAR_SIZE);

	pDev->_ring = malloc(sizeof(struct graes_ring) * GRAES_BDAR_ENTRIES);
	if ( !pDev->_ring ) {
		DBG("GRAES: Failed to allocate ring\n");
		return -1;
	}

	/* Reset Hardware before attaching IRQ handler */
	graes_hw_reset(pDev);

	/* Register interrupt handler */
	if ( rtems_drvmgr_interrupt_register(pDev->dev, 0, graes_interrupt, pDev) ) {
		DBG("GRAES: Failed to allocate irq %d@0x%08x %d:%d apb:0x%x\n", pnpinfo->irq, pDev->regs,pnpinfo->vendor,pnpinfo->device, pnpinfo->apb_slv);
		return -1;
	}

	return 0;
}


static inline void graes_list_clr(struct graes_list *list)
{
	list->head = NULL;
	list->tail = NULL;
}

static void graes_hw_reset(struct graes_priv *pDev)
{
	/* Reset Core */
}

static void graes_hw_get_implementation(struct graes_priv *pDev, struct graes_ioc_hw *hwcfg)
{
	hwcfg->key_size= 256;
}

#warning Extra: Implement proper default calculation from hardware configuration
static void graes_hw_get_default_modes(struct graes_ioc_config *cfg, struct graes_ioc_hw *hwcfg)
{
	cfg->key_size = 256;
	
	/* Interrupt options */
	cfg->blocking = 0;	/* non-blocking mode is default */
	cfg->enable_cnt = 1;	/* generate interrupt every 16 descriptor */
	cfg->isr_desc_proc = 1;	/* Let interrupt handler do descriptor processing */
	cfg->timeout = RTEMS_NO_TIMEOUT;
	
}

static void *graes_memalign(unsigned int boundary, unsigned int length, void *realbuf)
{
	*(int *)realbuf = (int)malloc(length+boundary);
	DBG("GRAES: Alloced %d (0x%x) bytes, requested: %d\n",length+boundary,length+boundary,length);
	return (void *)(((*(unsigned int *)realbuf)+boundary) & ~(boundary-1));
}

static int graes_hw_set_config(struct graes_priv *pDev, struct graes_ioc_config *cfg, struct graes_ioc_hw *hwcfg)
{
	struct graes_regs *regs = pDev->regs;
	unsigned int tmp;
	
	return 0;
}

static int graes_start(struct graes_priv *pDev)
{
	struct graes_regs *regs = pDev->regs;
	int i;
	struct graes_ioc_config *cfg = &pDev->config;
	volatile unsigned int *txrdy_reg;
	unsigned int txrdy_mask, transaddr;

	/* Clear Descriptors */
	memset(pDev->bds,0,GRAES_BDAR_SIZE);
	
	/* Clear stats */
	memset(&pDev->stats,0,sizeof(struct graes_ioc_stats));
	
	/* Init Descriptor Ring */
	memset(pDev->_ring,0,sizeof(struct graes_ring)*GRAES_BDAR_ENTRIES);
	for(i=0;i<(GRAES_BDAR_ENTRIES-1);i++){
		pDev->_ring[i].next = &pDev->_ring[i+1];
		pDev->_ring[i].bd = &pDev->bds[i];
		pDev->_ring[i].frm = NULL;
	}
	pDev->_ring[(GRAES_BDAR_ENTRIES-1)].next = &pDev->_ring[0];
	pDev->_ring[(GRAES_BDAR_ENTRIES-1)].bd = &pDev->bds[(GRAES_BDAR_ENTRIES-1)];
	pDev->_ring[(GRAES_BDAR_ENTRIES-1)].frm = NULL;

	pDev->ring = &pDev->_ring[0];
	pDev->ring_end = &pDev->_ring[0];

	/* Clear Scheduled, Ready and Sent list */
	graes_list_clr(&pDev->ready);
	graes_list_clr(&pDev->scheduled);
	graes_list_clr(&pDev->sent);

	/* Software init */
	pDev->handling_transmission = 0;
	
	/* Reset the transmitter */
	regs->dma_ctrl = 0;	/* Leave Reset */


	/* Set Descriptor Pointer Base register to point to first descriptor */
	rtems_drvmgr_mmap_translate(pDev->dev, 0, (void *)pDev->bds, (void **)&transaddr);
	regs->dma_bd = transaddr;
	
	DBG("GRAES: set bd to 0x%08x\n",transaddr);
		
	/*regs->dma_bd = (unsigned int)pDev->bds;*/

	/* Set hardware options as defined by config */
	if ( graes_hw_set_config(pDev, cfg, &pDev->hw_avail) ) {
		return RTEMS_IO_ERROR;
	}
	

	DBG("GRAES: reset time %d\n",i);


	/* Mark running before enabling the DMA transmitter */
	pDev->running = 1;

	/* Enable interrupts (Error and DMA TX) */
	regs->dma_ctrl = GRAES_DMA_CTRL_IE;

	DBG("GRAES: STARTED\n");

	return RTEMS_SUCCESSFUL;
}

static void graes_stop(struct graes_priv *pDev)
{
	struct graes_regs *regs = pDev->regs;

	/* Disable the transmitter & Interrupts */
	regs->dma_ctrl = 0;
	

	DBG("GRAES: STOPPED\n");

	/* Flush semaphore in case a thread is stuck waiting for TX Interrupts */
	rtems_semaphore_flush(pDev->sem_rx);
}

static rtems_device_driver graes_open(
	rtems_device_major_number major, 
	rtems_device_minor_number minor, 
	void *arg)
{
	struct graes_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&graes_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct graes_priv *)dev->priv;
	
	/* Wait until we get semaphore */
	if ( rtems_semaphore_obtain(graes_dev_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT) != RTEMS_SUCCESSFUL ){
		return RTEMS_INTERNAL_ERROR;
	}

	/* Is device in use? */
	if ( pDev->open ){
		rtems_semaphore_release(graes_dev_sem);
		return RTEMS_RESOURCE_IN_USE;
	}
	
	/* Mark device taken */
	pDev->open = 1;
	
	rtems_semaphore_release(graes_dev_sem);
	
	DBG("graes_open: OPENED minor %d (pDev: 0x%x)\n",pDev->minor,(unsigned int)pDev);
	/* Set defaults */
	pDev->config.timeout = RTEMS_NO_TIMEOUT;	/* no timeout (wait forever) */
	pDev->config.blocking = 0;			/* polling mode */
	
	pDev->running = 0;				/* not in running mode yet */

	memset(&pDev->config,0,sizeof(pDev->config));
	
	/* The core has been reset when we execute here, so it is possible
	 * to read out what HW is implemented from core.
	 */
	graes_hw_get_implementation(pDev, &pDev->hw_avail);

	/* Get default modes */
	graes_hw_get_default_modes(&pDev->config,&pDev->hw_avail);
	
	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver graes_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	struct graes_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&graes_drv_info.general, minor, &dev) ) {
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct graes_priv *)dev->priv;

	if ( pDev->running ){
		graes_stop(pDev);
		pDev->running = 0;
	}
	
	/* Reset core */
	graes_hw_reset(pDev);

	/* Clear descriptor area just for sure */
	memset(pDev->bds, 0, GRAES_BDAR_SIZE);
	
	/* Mark not open */
	pDev->open = 0;

	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver graes_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	FUNCDBG();
	return RTEMS_NOT_IMPLEMENTED;
}

static rtems_device_driver graes_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	FUNCDBG();
	return RTEMS_NOT_IMPLEMENTED;
}

/* Scans the desciptor table for scheduled frames that has been sent, 
 * and moves these frames from the head of the scheduled queue to the
 * tail of the sent queue.
 *
 * Also, for all frames the status is updated.
 *
 * Return Value
 * Number of frames freed.
 */
static int graes_free_encrypted(struct graes_priv *pDev)
{
	struct graes_ring *curr;
	struct graes_block *last_frm, *first_frm;
	int freed_frame_cnt=0;
	unsigned int ctrl;

	curr = pDev->ring_end;

	/* Step into TX ring to find sent frames */
	if ( !curr->frm ){
		/* No scheduled frames, abort */
		return 0;
	}

	/* There has been messages scheduled ==> scheduled messages may have been
	 * transmitted and needs to be collected.
	 */

	first_frm = curr->frm;

	/* Loop until first enabled unsent frame is found. 
	 * A unused descriptor is indicated by an unassigned frm field
	 */
	while ( curr->frm && !((ctrl=READ_REG(&curr->bd->ctrl)) & GRAES_BD_EN) ){
		/* Handle one sent Frame */
#ifdef DEBUG
		printk(" fini bd: 0x%08x @ 0x%08x\n",ctrl,curr->bd);
#endif
		
		/* Remember last handled frame so that insertion/removal from
		 * frames lists go fast.
		 */
		last_frm = curr->frm;
		
		/* 1. Set flags to indicate error(s) and other information */
		last_frm->flags |= GRAES_FLAGS_PROCESSED; /* Mark sent */

		/* Update Stats */
		pDev->stats.blocks_processed++;

		curr->frm = NULL; /* Mark unused */

		/* Increment */
		curr = curr->next;
		freed_frame_cnt++;
	}

	/* 1. Remove all handled frames from scheduled queue
	 * 2. Put all handled frames into sent queue
	 */
	if ( freed_frame_cnt > 0 ){

		/* Save TX ring posistion */
		pDev->ring_end = curr;

		/* Remove all sent frames from scheduled list */
		if ( pDev->scheduled.tail == last_frm ){
			/* All scheduled frames sent... */
			pDev->scheduled.head = NULL;
			pDev->scheduled.tail = NULL;
		}else{
			pDev->scheduled.head = last_frm->next;
		}
		last_frm->next = NULL;

		/* Put all sent frames into "Sent queue" for user to
		 * collect, later on.
		 */
		if ( !pDev->sent.head ){
			/* Sent queue empty */
			pDev->sent.head = first_frm;
			pDev->sent.tail = last_frm;
		}else{
			pDev->sent.tail->next = first_frm;
			pDev->sent.tail = last_frm;
		}
	}
	return freed_frame_cnt;
}

static unsigned int graes_trans(struct graes_priv *pDev, struct graes_block *curr_frm, unsigned int addr_in ) {
	unsigned int addr = addr_in;
	/* Prepare descriptor address. Three cases:
	 *  - GRAES core on same bus as CPU ==> no translation (Address used by CPU = address used by GRAES)
	 *  - GRAES core on remote bus, and payload address given as used by CPU ==> Translation needed
	 *  - GRAES core on remote bus, and payload address given as used by GRAES ==> no translation  [ USER does custom translation]
	 */
	if ( curr_frm->flags & (GRAES_FLAGS_TRANSLATE|GRAES_FLAGS_TRANSLATE_AND_REMEMBER) ) {
		/* Do translation */
		rtems_drvmgr_mmap_translate(pDev->dev, 0, (void *)addr_in, (void **)&addr);
		if ( curr_frm->flags & GRAES_FLAGS_TRANSLATE_AND_REMEMBER ) {
			if ( addr_in != addr ) {
				/* Translation needed */
				curr_frm->flags &= ~GRAES_FLAGS_TRANSLATE_AND_REMEMBER;
				curr_frm->flags |= GRAES_FLAGS_TRANSLATE;
			} else {
				/* No Trnaslation needed */
				curr_frm->flags &= ~(GRAES_FLAGS_TRANSLATE|GRAES_FLAGS_TRANSLATE_AND_REMEMBER);
			}
		}
	} else {
		/* Custom translation or no translation needed */
		addr = (unsigned int)addr_in;
	}
	return addr;
}

/* Moves as many frames in the ready queue (as there are free descriptors for)
 * to the scheduled queue. The free descriptors are then assigned one frame
 * each and enabled for transmission.
 * 
 * Return Value
 * Returns number of frames moved from ready to scheduled queue 
 */
static int graes_schedule_ready(struct graes_priv *pDev, int ints_off)
{
	int cnt;
	unsigned int ctrl, dmactrl, oldLevel, addr;
	struct graes_ring *curr_bd;
	struct graes_block *curr_frm, *last_frm;

	if ( !pDev->ready.head ){
		return 0;
	}

	cnt=0;
	curr_frm = pDev->ready.head;
	curr_bd = pDev->ring;
	while( !curr_bd->frm ){
		int i = 0, j; unsigned int kaddr = -1, iaddr = -1, oaddr = -1, daddr, naddr;
		/* Assign frame to descriptor */
		curr_bd->frm = curr_frm;

		/* Prepare descriptor address. Three cases:
		 *  - GRAES core on same bus as CPU ==> no translation (Address used by CPU = address used by GRAES)
		 *  - GRAES core on remote bus, and payload address given as used by CPU ==> Translation needed
		 *  - GRAES core on remote bus, and payload address given as used by GRAES ==> no translation  [ USER does custom translation]
		 */
		
		ctrl = GRAES_BD_EN;
		
		daddr = addr = graes_trans(pDev, curr_frm, curr_frm->payload);
		curr_bd->bd->u.d[i++] = addr;
			
		if (curr_frm->out) {
			ctrl |= GRAES_BD_OUT;
			oaddr = addr = graes_trans(pDev, curr_frm, curr_frm->out);
			curr_bd->bd->u.d[i++] = addr;
		}
		if (curr_frm->iv) {
			ctrl |= GRAES_BD_IV;
			iaddr = addr = graes_trans(pDev, curr_frm, curr_frm->iv);
			curr_bd->bd->u.d[i++] = addr;
		}
		if (curr_frm->key) {
			ctrl |= GRAES_BD_KEY;
			kaddr = addr = graes_trans(pDev, curr_frm, curr_frm->key);
			curr_bd->bd->u.d[i++] = addr;
		}

		rtems_drvmgr_mmap_translate(pDev->dev, 0, (void *)curr_bd->next->bd, (void **)&naddr);
		curr_bd->bd->u.d[i++] = naddr;
		
		if ( curr_bd->next == pDev->_ring ){
			
			/* Wrap around */
		}

		ctrl |= ((curr_frm->length & 0x7ff) << 21);
		
		/* Apply user options/flags */
		ctrl |= (curr_frm->flags & GRAES_FLAGS_MASK);
		
		/* Is this Frame going to be an interrupt Frame? */
		if ( (--pDev->enable_cnt_curr) <= 0 ){
			if ( pDev->config.enable_cnt == 0 ){
				pDev->enable_cnt_curr = 0x3fffffff;
			}else{
				pDev->enable_cnt_curr = pDev->config.enable_cnt;
				ctrl |= GRAES_BD_IE;
			}
		}

		/* Enable descriptor */
		curr_bd->bd->ctrl = ctrl;

#ifdef DEBUG
		printk(" add bd: 0x%08x @ 0x%08x [0x%08x",ctrl,curr_bd->bd, daddr);
		if(oaddr != -1)
			printk(",o:0x%08x",oaddr);
		if(iaddr != -1)
			printk(",i:0x%08x",iaddr);
		if(kaddr != -1)
			printk(",k:0x%08x",kaddr);
		printk("] [");
		for (j = 0; j < 6; j++) {
			printk(" 0x%08x",((unsigned int *)curr_bd->bd)[j]);
		}
		printk("]\n");
		
#endif
		
		last_frm = curr_frm;
		curr_bd = curr_bd->next;
		cnt++;
		
		/* Get Next Frame from Ready Queue */
		if ( curr_frm == pDev->ready.tail ){
			/* Handled all in ready queue. */
			curr_frm = NULL;
			break;
		}
		curr_frm = curr_frm->next;
	}
	
	/* Has frames have been scheduled? */
	if ( cnt > 0 ){
		/* Make last frame mark end of chain, probably pointless... */
		last_frm->next = NULL;

		/* Insert scheduled packets into scheduled queue */
		if ( !pDev->scheduled.head ){
			/* empty scheduled queue */
			pDev->scheduled.head = pDev->ready.head;
			pDev->scheduled.tail = last_frm;
		}else{
			pDev->scheduled.tail->next = pDev->ready.head;
			pDev->scheduled.tail = last_frm;
		}

		/* Remove scheduled packets from ready queue */
		pDev->ready.head = curr_frm;
		if ( !curr_frm ){
			pDev->ready.tail = NULL;
		}

		/* Update TX ring posistion */
		pDev->ring = curr_bd;
		if ( !ints_off ) {
			IRQ_GLOBAL_DISABLE(oldLevel);
		}

		/* Make hardware aware of the newly enabled descriptors */
		dmactrl = READ_REG(&pDev->regs->dma_ctrl);
		dmactrl |= GRAES_DMA_CTRL_EN;
		pDev->regs->dma_ctrl = dmactrl;
		
		if ( !ints_off ) {
			IRQ_GLOBAL_ENABLE(oldLevel);
		}
	}
	return cnt;
}

static void graes_printchain(struct graes_priv *pDev, struct graes_print_status *ps, struct graes_list *chain) 
{
	struct graes_block *curr;
	curr = chain->head;
	while(curr){
		printk(" 0x%08x: [@0x%08x]\n",curr,curr->payload);
		if (curr == chain->tail)
			break;
		curr = curr->next;
	}
}

static void graes_printstatus(struct graes_priv *pDev, struct graes_print_status *ps) 
{
	int oldLevel;
	
	IRQ_GLOBAL_DISABLE(oldLevel);
	
	printk("processed    : %d\n",(int)pDev->stats.blocks_processed);
	
	printk("ready_cnt    : %d\n",pDev->ready_cnt);
	printk("scheduled_cnt: %d\n",pDev->scheduled_cnt);
	printk("sent_cnt     : %d\n",pDev->sent_cnt);
	
	printk(" ready:\n");
	graes_printchain(pDev, ps, &pDev->ready);
	printk(" scheduled:\n");
	graes_printchain(pDev, ps, &pDev->scheduled);
	printk(" sent:\n");
	graes_printchain(pDev, ps, &pDev->sent);
	IRQ_GLOBAL_ENABLE(oldLevel);
}


static rtems_device_driver graes_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	struct graes_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;
	rtems_libio_ioctl_args_t *ioarg = (rtems_libio_ioctl_args_t *)arg;
	unsigned int *data = ioarg->buffer;
	int status;
	struct graes_ioc_config *cfg;
	struct graes_ioc_hw_status *hwregs;
	IRQ_GLOBAL_PREPARE(oldLevel);
	struct graes_list *chain;
	struct graes_block *curr;
	struct graes_ioc_hw *hwimpl;
	struct graes_ioc_stats *stats;
	struct graes_print_status *ps;
	int num,ret;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&graes_drv_info.general, minor, &dev) ) {
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct graes_priv *)dev->priv;

	if (!ioarg)
		return RTEMS_INVALID_NAME;

	ioarg->ioctl_return = 0;
	switch(ioarg->command) {
		case GRAES_IOC_START:
		if ( pDev->running ) {
			return RTEMS_RESOURCE_IN_USE; /* EBUSY */
		}
		if ( (status=graes_start(pDev)) != RTEMS_SUCCESSFUL ){
			return status;
		}
		/* Enable interrupt */
		rtems_drvmgr_interrupt_enable(dev, 0, graes_interrupt, pDev);

		/* Read and write are now open... */
		break;

		case GRAES_IOC_STOP:
		if ( !pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}

		/* Disable interrupts */
		rtems_drvmgr_interrupt_disable(dev, 0, graes_interrupt, pDev);
		graes_stop(pDev);
		pDev->running = 0;
		break;

		case GRAES_IOC_ISSTARTED:
		if ( !pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}
		break;

		case GRAES_IOC_SET_BLOCKING_MODE:
		if ( (unsigned int)data > GRAES_BLKMODE_BLK ) {
			return RTEMS_INVALID_NAME;
		}
		DBG("GRAES: Set blocking mode: %d\n",(unsigned int)data);
		pDev->config.blocking = (unsigned int)data;
		break;

		case GRAES_IOC_SET_TIMEOUT:
		DBG("GRAES: Timeout: %d\n",(unsigned int)data);
		pDev->config.timeout = (rtems_interval)data;
		break;

		case GRAES_IOC_SET_CONFIG:
		cfg = (struct graes_ioc_config *)data;
		if ( !cfg ) {
			return RTEMS_INVALID_NAME;
		}
		
		if ( pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}
		
		pDev->config = *cfg;
		break;

		case GRAES_IOC_GET_STATS:
		stats = (struct graes_ioc_stats *)data;
		if ( !stats ) {
			return RTEMS_INVALID_NAME;
		}
		memcpy(stats,&pDev->stats,sizeof(struct graes_ioc_stats));
		break;

		case GRAES_IOC_CLR_STATS:
		memset(&pDev->stats,0,sizeof(struct graes_ioc_stats));
		break;

		case GRAES_IOC_GET_CONFIG:
		cfg = (struct graes_ioc_config *)data;
		if ( !cfg ) {
			return RTEMS_INVALID_NAME;
		}

		*cfg = pDev->config;
		break;

		case GRAES_IOC_GET_HW_IMPL:
		hwimpl = (struct graes_ioc_hw *)data;
		if ( !hwimpl ) {
			return RTEMS_INVALID_NAME;
		}
		*hwimpl = pDev->hw_avail;
		break;

		case GRAES_IOC_GET_HW_STATUS:
		hwregs = (struct graes_ioc_hw_status *)data;
		if ( !hwregs ) {
			return RTEMS_INVALID_NAME;
		}
		/* We disable interrupt in order to get a snapshot of the registers */
		IRQ_GLOBAL_DISABLE(oldLevel);
		IRQ_GLOBAL_ENABLE(oldLevel);
		break;

		case GRAES_IOC_PRINT_STATUS:
		ps = (struct graes_print_status *)data;
		graes_printstatus(pDev, ps);
		break;
		
		/* Put a chain of frames at the back of the "Ready frames" queue. This 
		 * triggers the driver to put frames from the Ready queue into unused 
		 * available descriptors. (Ready -> Scheduled)
		 */

		case GRAES_IOC_ENCRYPT:
		if ( !pDev->running ){
			return RTEMS_RESOURCE_IN_USE;
		}
		num=0;

		/* Get pointer to frame chain wished be sent */
		chain = (struct graes_list *)ioarg->buffer;
		if ( !chain ){
			/* No new frames to send ==> just trigger hardware
			 * to send previously made ready frames to be sent.
			 */
			pDev->handling_transmission = 1;
			goto trigger_transmission;
		}
		if ( !chain->tail || !chain->head ){
			return RTEMS_INVALID_NAME;
		}


		/* Mark ready frames unsent by clearing GRAES_FLAGS_PROCESSED of all frames */

		curr = chain->head;
		while(curr != chain->tail){
			curr->flags = curr->flags & ~(GRAES_FLAGS_PROCESSED|GRRM_FLAGS_ERR);
			curr = curr->next;
			num++;
		}
		curr->flags = curr->flags & ~(GRAES_FLAGS_PROCESSED|GRRM_FLAGS_ERR);
		num++;

		DBG("GRAES_ENCRYPT: head: 0x%x, tail: 0x%x num: %d\n",chain->head,chain->tail, num);
		
		pDev->handling_transmission = 1;
		/* 1. Put frames into ready queue 
		 *    (New Frames->READY)
		 */
		if ( pDev->ready.head ){
			/* Frames already on ready queue (no free descriptors previously) ==>
			 * Put frames at end of ready queue
			 */
			pDev->ready.tail->next = chain->head;
			pDev->ready.tail = chain->tail;
			chain->tail->next = NULL;
		}else{
			/* All frames is put into the ready queue for later processing */
			pDev->ready.head = chain->head;
			pDev->ready.tail = chain->tail;
			chain->tail->next = NULL;
		}
		pDev->ready_cnt += num;	/* Added 'num' frames to ready queue */
trigger_transmission:
		/* 2. Free used descriptors and put the sent frame into the "Sent queue"  
		 *    (SCHEDULED->SENT)
		 */
		num = graes_free_encrypted(pDev);
		pDev->scheduled_cnt -= num;
		pDev->sent_cnt += num;

		/* 3. Use all available free descriptors there are frames for
		 *    in the ready queue.
		 *    (READY->SCHEDULED)
		 */
		num = graes_schedule_ready(pDev,0);
		pDev->ready_cnt -= num;
		pDev->scheduled_cnt += num;
	
		pDev->handling_transmission = 0;
		break;

		/* Take all available sent frames from the "Sent frames" queue.
		 * If no frames has been sent, the thread may get blocked if in blocking
		 * mode. The blocking mode is not available if driver is not in running mode.
		 *
		 * Note this ioctl may return success even if the driver is not in STARTED mode.
		 * This is because in case of a error (link error of similar) and the driver switch
		 * from START to STOP mode we must still be able to get our frames back.
		 * 
		 * Note in case the driver fails to send a frame for some reason (link error),
		 * the sent flag is set to 0 indicating a failure.
		 *
		 */
		case GRAES_IOC_RECLAIM:
		/* Get pointer to were to place reaped chain */
		chain = (struct graes_list *)ioarg->buffer;
		if ( !chain ){
			return RTEMS_INVALID_NAME;
		}

		/* Lock out interrupt handler */
		pDev->handling_transmission = 1;

		do {
			/* Move sent frames from descriptors to Sent queue. This makes more 
			 * descriptors (BDs) available.
			 */
			num = graes_free_encrypted(pDev);
			pDev->scheduled_cnt -= num;
			pDev->sent_cnt += num;
			

			if ( pDev->running ){
				/* Fill descriptors with as many frames from the ready list 
				 * as possible.
				 */
				num = graes_schedule_ready(pDev,0);
				pDev->ready_cnt -= num;
				pDev->scheduled_cnt += num;
			}

			/* Are there any frames on the sent queue waiting to be 
			 * reclaimed?
			 */

			if ( !pDev->sent.head ){
				/* No frames to reclaim - no frame in sent queue.
				 * Instead we block thread until frames have been sent 
				 * if in blocking mode.
				 */
				if ( pDev->running && pDev->config.blocking ){
					ret = rtems_semaphore_obtain(pDev->sem_rx,RTEMS_WAIT,pDev->config.timeout);
					if ( ret == RTEMS_TIMEOUT ) {
						pDev->handling_transmission = 0;
						return RTEMS_TIMEOUT;
					} else if ( ret == RTEMS_SUCCESSFUL ) {
						/* There might be frames available, go check */
						continue;
					} else {
						/* any error (driver closed, internal error etc.) */
						pDev->handling_transmission = 0;
						return RTEMS_UNSATISFIED;
					}

				}else{
					/* non-blocking mode, we quit */
					chain->head = NULL;
					chain->tail = NULL;
					/* do not lock out interrupt handler any more */
					pDev->handling_transmission = 0;
					return RTEMS_TIMEOUT;
				}
			}else{
				/* Take all sent framess from sent queue to userspace queue */
				chain->head = pDev->sent.head;
				chain->tail = pDev->sent.tail;
				chain->tail->next = NULL; /* Just for sure */

				/* Mark no Sent */
				graes_list_clr(&pDev->sent);

				DBG("TX_RECLAIM: head: 0x%x, tail: 0x%x\n",chain->head,chain->tail);
				break;
			}

		}while(1);
		
		/* do not lock out interrupt handler any more */
		pDev->handling_transmission = 0;
		break;

		default:
		return RTEMS_NOT_DEFINED;
	}
	return RTEMS_SUCCESSFUL;
}

static void graes_interrupt(int irq, void *arg)
{
	struct graes_priv *pDev = arg;
	struct graes_regs *regs = pDev->regs;
	unsigned int status;
	int num;

	/* Clear interrupt by reading it */
	status = READ_REG(&regs->dma_status);


	DBG("GRAES: irq 0x%x\n",(unsigned int)status);
	
	/* Spurious Interrupt? */
	if ( !pDev->running )
		return;

	if ( status )
		regs->dma_status = status;


	if ( 1 ){
		
		if ( pDev->config.isr_desc_proc && !pDev->handling_transmission ) {
			/* Free used descriptors and put the sent frame into the "Sent queue"  
			 *   (SCHEDULED->SENT)
			 */
			num = graes_free_encrypted(pDev);
			pDev->scheduled_cnt -= num;
			pDev->sent_cnt += num;

			/* Use all available free descriptors there are frames for
			 * in the ready queue.
			 *   (READY->SCHEDULED)
			 */
			num = graes_schedule_ready(pDev,1);
			pDev->ready_cnt -= num;
			pDev->scheduled_cnt += num;

#if 0
			if ( (pDev->config.blocking==GRAES_BLKMODE_COMPLETE) && pDev->timeout ){
				/* Signal to thread only if enough data is available */
				if ( pDev->wait_for_frames > graes_data_avail(pDev) ){
					/* Not enough data available */
					goto procceed_processing_interrupts;
				}

				/* Enough number of frames has been transmitted which means that
				 * the waiting thread should be woken up.
				 */
				rtems_semaphore_release(pDev->sem_rx);
			}
#endif
		}

		if ( pDev->config.blocking == GRAES_BLKMODE_BLK ) {
			/* Blocking mode */

#if 0
			/* Disable further Interrupts until handled by waiting task. */
			regs->dma_ctrl = READ_REG(&regs->dma_ctrl) & ~GRAES_DMA_CTRL_IE;
#endif
		
			/* Signal Semaphore to wake waiting thread in ioctl(ENCRYPT|RECLAIM) */
			rtems_semaphore_release(pDev->sem_rx);
		}

	}

procceed_processing_interrupts:
	;
}

static rtems_device_driver graes_initialize(
  rtems_device_major_number major, 
  rtems_device_minor_number unused,
  void *arg
  )
{
	/* Device Semaphore created with count = 1 */
	if ( rtems_semaphore_create(rtems_build_name('G', 'R', 'P', 'R'),
		1,
		RTEMS_FIFO|RTEMS_NO_INHERIT_PRIORITY|RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING,
		0,
		&graes_dev_sem) != RTEMS_SUCCESSFUL ) {
		return RTEMS_INTERNAL_ERROR;
	}

	return RTEMS_SUCCESSFUL;
}
