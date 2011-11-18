/* GRTM CCSDS Telemetry Encoder driver
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
 *  2009-11-23, Daniel Hellstrom <daniel@gaisler.com>
 *    Created from on-chip GRTM driver
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
#include <grtm.h>


#ifndef IRQ_GLOBAL_PREPARE
 #define IRQ_GLOBAL_PREPARE(level) rtems_interrupt_level level
#endif

#ifndef IRQ_GLOBAL_DISABLE
 #define IRQ_GLOBAL_DISABLE(level) rtems_interrupt_disable(level)
#endif

#ifndef IRQ_GLOBAL_ENABLE
 #define IRQ_GLOBAL_ENABLE(level) rtems_interrupt_enable(level)
#endif

/**** START: RMAP STUFF ****/
#define DESCRIPTOR_MAX 128
#define WRITE_REG(pDev, adr, value) rtems_drvmgr_write_io32(pDev->dev->parent->dev, (uint32_t *)adr, (uint32_t)value)
#define READ_REG_(pDev, adr, dstadr) rtems_drvmgr_read_io32(pDev->dev->parent->dev, (uint32_t *)adr, dstadr)
#define READ_REG(pDev, adr) grtm_rmap_read_reg(pDev, (uint32_t *)adr)
#define TRANSFER_FRM(pDev, dstadr, srcadr, length) rtems_drvmgr_write_mem(pDev->dev->parent->dev, dstadr, srcadr, length)

/* This call will take 128 bytes of buffer at stack */
#define MEMSET(pDev, adr, c, length) rtems_drvmgr_write_memset(pDev->dev->parent->dev, adr, c, length)

/**** END: RMAP STUFF ****/

/*
#define DEBUG
#define DEBUGFUNCS
*/

#include <debug_defs.h>

/* GRTM register map */
struct grtm_regs {
	volatile unsigned int	dma_ctrl;	/* DMA Control Register (0x00) */
	volatile unsigned int	dma_status;	/* DMA Status Register (0x04) */
	volatile unsigned int	dma_len;	/* DMA Length Register (0x08) */	
	volatile unsigned int	dma_bd;		/* DMA Descriptor Pointer Register (0x0c) */

	volatile unsigned int	dma_cfg;	/* DMA Configuration Register (0x10) */
	volatile unsigned int	revision;	/* GRTM Revision Register (0x14) */

	int unused00[2];

	volatile unsigned int	ext_ctrl;	/* External Control Register (0x20) */

	int unused01[2];

	volatile unsigned int	ext_bd;		/* External Descriptor Register (0x20) */

	int unused0[(0x80-0x30)/4];

	volatile unsigned int	ctrl;		/* TM Control Register (0x80) */
	volatile unsigned int	status;		/* TM Status Register (0x84) */
	volatile unsigned int	cfg;		/* TM Configuration Register (0x88) */
	volatile unsigned int	size;		/* TM Size Register (0x8c) */

	volatile unsigned int	phy;		/* TM Physical Layer Register (0x90) */
	volatile unsigned int	code;		/* TM Coding Sub-Layer Register (0x94) */
	volatile unsigned int	asmr;		/* TM Attached Synchronization Marker Register (0x98) */

	int unused1;

	volatile unsigned int	all_frm;	/* TM All Frames Generation Register (0xa0) */
	volatile unsigned int	mst_frm;	/* TM Master Channel Frame Generation Register (0xa4) */
	volatile unsigned int	idle_frm;	/* TM Idle Frame Generation Register (0xa8) */

	int unused2[(0xc0-0xac)/4];

	volatile unsigned int	fsh[4];		/* TM FSH/Insert Zone Registers (0xc0..0xcc) */

	volatile unsigned int	ocf;		/* TM Operational Control Field Register (0xd0) */
};

/* DMA Control Register (0x00) */
#define GRTM_DMA_CTRL_EN_BIT	0
#define GRTM_DMA_CTRL_IE_BIT	1
#define GRTM_DMA_CTRL_TXRST_BIT	2
#define GRTM_DMA_CTRL_RST_BIT	3
#define GRTM_DMA_CTRL_TFIE_BIT	4

#define GRTM_DMA_CTRL_EN	(1<<GRTM_DMA_CTRL_EN_BIT)
#define GRTM_DMA_CTRL_IE	(1<<GRTM_DMA_CTRL_IE_BIT)
#define GRTM_DMA_CTRL_TXRST	(1<<GRTM_DMA_CTRL_TXRST_BIT)
#define GRTM_DMA_CTRL_RST	(1<<GRTM_DMA_CTRL_RST_BIT)
#define GRTM_DMA_CTRL_TFIE	(1<<GRTM_DMA_CTRL_TFIE_BIT)

/* DMA Status Register (0x04) */
#define GRTM_DMA_STS_TE_BIT	0
#define GRTM_DMA_STS_TI_BIT	1
#define GRTM_DMA_STS_TA_BIT	2
#define GRTM_DMA_STS_TFF_BIT	3
#define GRTM_DMA_STS_TFS_BIT	4

#define GRTM_DMA_STS_TE		(1<<GRTM_DMA_STS_TE_BIT)
#define GRTM_DMA_STS_TI		(1<<GRTM_DMA_STS_TI_BIT)
#define GRTM_DMA_STS_TA		(1<<GRTM_DMA_STS_TA_BIT)
#define GRTM_DMA_STS_TFF	(1<<GRTM_DMA_STS_TFF_BIT)
#define GRTM_DMA_STS_TFS	(1<<GRTM_DMA_STS_TFS_BIT)
#define GRTM_DMA_STS_ALL	0x1f

/* DMA Length Register (0x08) */
#define GRTM_DMA_LEN_LEN_BIT	0
#define GRTM_DMA_LEN_LIM_BIT	16

#define GRTM_DMA_LEN_LEN	(0x7ff<<GRTM_DMA_LEN_LEN_BIT)
#define GRTM_DMA_LEN_LIM	(0x3ff<<GRTM_DMA_LEN_LIM_BIT)

/* DMA Descriptor Pointer Register (0x0c) */
#define GRTM_DMA_BD_INDEX_BIT	0
#define GRTM_DMA_BD_BASE_BIT	10

#define GRTM_DMA_BD_INDEX	(0x3ff<<GRTM_DMA_BD_INDEX_BIT)
#define GRTM_DMA_BD_BASE	(0xfffffc<<GRTM_DMA_BD_BASE_BIT)

/* DMA Configuration Register (0x10) */
#define GRTM_DMA_CFG_BLKSZ_BIT	0
#define GRTM_DMA_CFG_FIFOSZ_BIT	16

#define GRTM_DMA_CFG_BLKSZ	(0xffff<<GRTM_DMA_CFG_BLKSZ_BIT)
#define GRTM_DMA_CFG_FIFOSZ	(0xffff<<GRTM_DMA_CFG_FIFOSZ_BIT)

/* TM Control Register (0x80) */
#define GRTM_CTRL_EN_BIT	0

#define GRTM_CTRL_EN		(1<<GRTM_CTRL_EN_BIT)

/* TM Status Register (0x84) - Unused */

/* TM Configuration Register (0x88) */
#define GRTM_CFG_SC_BIT		0
#define GRTM_CFG_SP_BIT		1
#define GRTM_CFG_CE_BIT		2
#define GRTM_CFG_NRZ_BIT	3
#define GRTM_CFG_PSR_BIT	4
#define GRTM_CFG_TE_BIT		5
#define GRTM_CFG_RSDEP_BIT	6
#define GRTM_CFG_RS_BIT		9
#define GRTM_CFG_AASM_BIT	11
#define GRTM_CFG_FECF_BIT	12
#define GRTM_CFG_OCF_BIT	13
#define GRTM_CFG_EVC_BIT	14
#define GRTM_CFG_IDLE_BIT	15
#define GRTM_CFG_FSH_BIT	16
#define GRTM_CFG_MCG_BIT	17
#define GRTM_CFG_IZ_BIT		18
#define GRTM_CFG_FHEC_BIT	19
#define GRTM_CFG_AOS_BIT	20
#define GRTM_CFG_CIF_BIT	21
#define GRTM_CFG_OCFB_BIT	22

#define GRTM_CFG_SC		(1<<GRTM_CFG_SC_BIT)
#define GRTM_CFG_SP		(1<<GRTM_CFG_SP_BIT)
#define GRTM_CFG_CE		(1<<GRTM_CFG_CE_BIT)
#define GRTM_CFG_NRZ		(1<<GRTM_CFG_NRZ_BIT)
#define GRTM_CFG_PSR		(1<<GRTM_CFG_PSR_BIT)
#define GRTM_CFG_TE		(1<<GRTM_CFG_TE_BIT)
#define GRTM_CFG_RSDEP		(0x7<<GRTM_CFG_RSDEP_BIT)
#define GRTM_CFG_RS		(0x3<<GRTM_CFG_RS_BIT)
#define GRTM_CFG_AASM		(1<<GRTM_CFG_AASM_BIT)
#define GRTM_CFG_FECF		(1<<GRTM_CFG_FECF_BIT)
#define GRTM_CFG_OCF		(1<<GRTM_CFG_OCF_BIT)
#define GRTM_CFG_EVC		(1<<GRTM_CFG_EVC_BIT)
#define GRTM_CFG_IDLE		(1<<GRTM_CFG_IDLE_BIT)
#define GRTM_CFG_FSH		(1<<GRTM_CFG_FSH_BIT)
#define GRTM_CFG_MCG		(1<<GRTM_CFG_MCG_BIT)
#define GRTM_CFG_IZ		(1<<GRTM_CFG_IZ_BIT)
#define GRTM_CFG_FHEC		(1<<GRTM_CFG_FHEC_BIT)
#define GRTM_CFG_AOS		(1<<GRTM_CFG_AOS_BIT)
#define GRTM_CFG_CIF		(1<<GRTM_CFG_CIF_BIT)
#define GRTM_CFG_OCFB		(1<<GRTM_CFG_OCFB_BIT)

/* TM Size Register (0x8c) */
#define GRTM_SIZE_BLKSZ_BIT	0
#define GRTM_SIZE_FIFOSZ_BIT	8
#define GRTM_SIZE_LEN_BIT	20

#define GRTM_SIZE_BLKSZ		(0xff<<GRTM_SIZE_BLKSZ_BIT)
#define GRTM_SIZE_FIFOSZ	(0xfff<<GRTM_SIZE_FIFOSZ_BIT)
#define GRTM_SIZE_LEN		(0xfff<<GRTM_SIZE_LEN_BIT)

/* TM Physical Layer Register (0x90) */
#define GRTM_PHY_SUB_BIT	0
#define GRTM_PHY_SCF_BIT	15
#define GRTM_PHY_SYM_BIT	16
#define GRTM_PHY_SF_BIT		31

#define GRTM_PHY_SUB		(0x7fff<<GRTM_PHY_SUB_BIT)
#define GRTM_PHY_SCF		(1<<GRTM_PHY_SCF_BIT)
#define GRTM_PHY_SYM		(0x7fff<<GRTM_PHY_SYM_BIT)
#define GRTM_PHY_SF		(1<<GRTM_PHY_SF_BIT)

/* TM Coding Sub-Layer Register (0x94) */
#define GRTM_CODE_SC_BIT	0
#define GRTM_CODE_SP_BIT	1
#define GRTM_CODE_CERATE_BIT	2
#define GRTM_CODE_CE_BIT	5
#define GRTM_CODE_NRZ_BIT	6
#define GRTM_CODE_PSR_BIT	7
#define GRTM_CODE_RS8_BIT	11
#define GRTM_CODE_RSDEP_BIT	12
#define GRTM_CODE_RS_BIT	15
#define GRTM_CODE_AASM_BIT	16
#define GRTM_CODE_CSEL_BIT	17

#define GRTM_CODE_SC		(1<<GRTM_CODE_SC_BIT)
#define GRTM_CODE_SP		(1<<GRTM_CODE_SP_BIT)
#define GRTM_CODE_CERATE	(0x7<<GRTM_CODE_CERATE_BIT)
#define GRTM_CODE_CE		(1<<GRTM_CODE_CE_BIT)
#define GRTM_CODE_NRZ		(1<<GRTM_CODE_NRZ_BIT)
#define GRTM_CODE_PSR		(1<<GRTM_CODE_PSR_BIT)
#define GRTM_CODE_RS8		(1<<GRTM_CODE_RS8_BIT)
#define GRTM_CODE_RSDEP		(0x7<<GRTM_CODE_RSDEP_BIT)
#define GRTM_CODE_RS		(1<<GRTM_CODE_RS_BIT)
#define GRTM_CODE_AASM		(1<<GRTM_CODE_AASM_BIT)
#define GRTM_CODE_CSEL		(0x3<<GRTM_CODE_CSEL_BIT)

/* TM Attached Synchronization Marker Register (0x98) */
#define GRTM_ASM_BIT		0

#define GRTM_ASM		0xffffffff

/* TM All Frames Generation Register (0xa0) */
#define GRTM_ALL_LEN_BIT	0
#define GRTM_ALL_VER_BIT	12
#define GRTM_ALL_FHEC_BIT	14
#define GRTM_ALL_FECF_BIT	15
#define GRTM_ALL_IZ_BIT		16
#define GRTM_ALL_IZLEN_BIT	17

#define GRTM_ALL_LEN		(0x7ff<<GRTM_ALL_LEN_BIT)
#define GRTM_ALL_VER		(0x3<<GRTM_ALL_VER_BIT)
#define GRTM_ALL_FHEC		(1<<GRTM_ALL_FHEC_BIT)
#define GRTM_ALL_FECF		(1<<GRTM_ALL_FECF_BIT)
#define GRTM_ALL_IZ		(1<<GRTM_ALL_IZ_BIT)
#define GRTM_ALL_IZLEN		(0x1f<<GRTM_ALL_IZLEN_BIT)

/* TM Master Channel Frame Generation Register (0xa4) */
#define GRTM_MST_OW_BIT		0
#define GRTM_MST_OCF_BIT	1
#define GRTM_MST_FSH_BIT	2
#define GRTM_MST_MC_BIT		3
#define GRTM_MST_MCCNTR_BIT	24

#define GRTM_MST_OW		(1<<GRTM_MST_OW_BIT)
#define GRTM_MST_OCF		(1<<GRTM_MST_OCF_BIT)
#define GRTM_MST_FSH		(1<<GRTM_MST_FSH_BIT)
#define GRTM_MST_MC		(0xff<<GRTM_MST_MC_BIT)

/* TM Idle Frame Generation Register (0xa8) */
#define GRTM_IDLE_SCID_BIT	0
#define GRTM_IDLE_VCID_BIT	10
#define GRTM_IDLE_MC_BIT	16
#define GRTM_IDLE_VCC_BIT	17
#define GRTM_IDLE_FSH_BIT	18
#define GRTM_IDLE_EVC_BIT	19
#define GRTM_IDLE_OCF_BIT	20
#define GRTM_IDLE_IDLE_BIT	21
#define GRTM_IDLE_MCCNTR_BIT	24

#define GRTM_IDLE_SCID		(0x3ff<<GRTM_IDLE_SCID_BIT)
#define GRTM_IDLE_VCID		(0x3f<<GRTM_IDLE_VCID_BIT)
#define GRTM_IDLE_MC		(1<<GRTM_IDLE_MC_BIT)
#define GRTM_IDLE_VCC		(1<<GRTM_IDLE_VCC_BIT)
#define GRTM_IDLE_FSH		(1<<GRTM_IDLE_FSH_BIT)
#define GRTM_IDLE_EVC		(1<<GRTM_IDLE_EVC_BIT)
#define GRTM_IDLE_OCF		(1<<GRTM_IDLE_OCF_BIT)
#define GRTM_IDLE_IDLE		(1<<GRTM_IDLE_IDLE_BIT)
#define GRTM_IDLE_MCCNTR	(0xff<<GRTM_IDLE_MCCNTR_BIT)

/* TM FSH/Insert Zone Registers (0xc0..0xcc) */
#define GRTM_FSH_DATA_BIT	0

#define GRTM_FSH_DATA		0xffffffff


/* TM Operational Control Field Register (0xd0) */
#define GRTM_OCF_CLCW_BIT	0

#define GRTM_OCF_CLCW		0xffffffff


/* GRTM Revision 0 */
#define GRTM_REV0_DMA_CTRL_TXRDY_BIT	5
#define GRTM_REV0_DMA_CTRL_TXRDY	(1<<GRTM_REV0_DMA_CTRL_TXRDY_BIT)

/* GRTM Revision 1 */
#define GRTM_REV1_DMA_STS_TXRDY_BIT	6
#define GRTM_REV1_DMA_STS_TXSTAT_BIT	7
#define GRTM_REV1_DMA_STS_TXRDY		(1<<GRTM_REV1_DMA_STS_TXRDY_BIT)
#define GRTM_REV1_DMA_STS_TXSTAT	(1<<GRTM_REV1_DMA_STS_TXSTAT_BIT)

#define GRTM_REV1_REV_SREV_BIT		0
#define GRTM_REV1_REV_MREV_BIT		8
#define GRTM_REV1_REV_TIRQ_BIT		16
#define GRTM_REV1_REV_SREV		(0xff<<GRTM_REV1_REV_SREV_BIT)
#define GRTM_REV1_REV_MREV		(0xff<<GRTM_REV1_REV_MREV_BIT)
#define GRTM_REV1_REV_TIRQ		(1<<GRTM_REV1_REV_TIRQ_BIT)


/* GRTM transmit descriptor (0x400 Alignment need) */
struct grtm_bd {
	volatile unsigned int	ctrl;
	unsigned int		address;
};

#define GRTM_BD_EN_BIT		0
#define GRTM_BD_WR_BIT		1
#define GRTM_BD_IE_BIT		2
#define GRTM_BD_FECFB_BIT	3
#define GRTM_BD_IZB_BIT		4
#define GRTM_BD_FHECB_BIT	5
#define GRTM_BD_OCFB_BIT	6
#define GRTM_BD_FSHB_BIT	7
#define GRTM_BD_MCB_BIT		8
#define GRTM_BD_VCE_BIT		9
#define GRTM_BD_TS_BIT		14
#define GRTM_BD_UE_BIT		15

#define GRTM_BD_EN		(1<<GRTM_BD_EN_BIT)
#define GRTM_BD_WR		(1<<GRTM_BD_WR_BIT)
#define GRTM_BD_IE		(1<<GRTM_BD_IE_BIT)
#define GRTM_BD_FECFB		(1<<GRTM_BD_FECFB_BIT)
#define GRTM_BD_IZB		(1<<GRTM_BD_IZB_BIT)
#define GRTM_BD_FHECB		(1<<GRTM_BD_FHECB_BIT)
#define GRTM_BD_OCFB		(1<<GRTM_BD_OCFB_BIT)
#define GRTM_BD_FSHB		(1<<GRTM_BD_FSHB_BIT)
#define GRTM_BD_MCB		(1<<GRTM_BD_MCB_BIT)
#define GRTM_BD_VCE		(1<<GRTM_BD_VCE_BIT)
#define GRTM_BD_TS		(1<<GRTM_BD_TS_BIT)
#define GRTM_BD_UE		(1<<GRTM_BD_UE_BIT)

/* Load register */

/*#define READ_REG(address)	(*(volatile unsigned int *)address)*/

/* Driver functions */
static rtems_device_driver grtm_initialize(rtems_device_major_number  major, rtems_device_minor_number  minor, void *arg);
static rtems_device_driver grtm_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver grtm_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver grtm_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver grtm_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver grtm_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);

#define GRTM_DRIVER_TABLE_ENTRY { grtm_initialize, grtm_open, grtm_close, grtm_read, grtm_write, grtm_ioctl }

static rtems_driver_address_table grtm_driver = GRTM_DRIVER_TABLE_ENTRY;

/* Structure that connects BD with SoftWare Frame */
struct grtm_ring {
	struct grtm_ring	*next;
	struct grtm_bd		*bd;
	struct grtm_frame	*frm;
	void			*buf;	/* Frame buffer in remote address */
};

struct grtm_priv {
	struct rtems_drvmgr_dev_info	*dev;		/* Driver manager device */
	char			devName[32];	/* Device Name */
	struct grtm_regs	*regs;
	int			irq;
	int			minor;
	int			subrev;		/* GRTM Revision */

	int			open;
	int			running;
	
	int			alloc_part_bd;	/* Partition to allocate descriptors from */
	int			alloc_part_frm;	/* Partition to allocate Frame buffers from */

	struct grtm_bd		*bds;
	void			*_bds;
	unsigned char		*_frame_buffers;
	int			frame_length_max;

	/* Interrupt generation */
	int			enable_cnt_curr;/* Down counter, when 0 the interrupt bit is set for next descriptor */
	rtems_id		handling_transmission;	/* SEMAPHORE: Tells ISR if user are active changing descriptors/queues */

	struct grtm_ring 	*_ring;		/* Root of ring */
	struct grtm_ring 	*ring;		/* Next ring to use for new frames to be transmitted */
	struct grtm_ring 	*ring_end;	/* Oldest activated ring used */

	/* Collections of frames Ready to sent/ Scheduled for transmission/Sent 
	 * frames waiting for the user to reclaim 
	 */
	struct grtm_list	ready;		/* Frames Waiting for free BDs */
	struct grtm_list	scheduled;	/* Frames in BDs beeing transmitted */
	struct grtm_list	sent;		/* Sent Frames waiting for user to reclaim and reuse */

	/* Number of frames in the lists */
	int			ready_cnt;	/* Number of ready frames */
	int			scheduled_cnt;	/* Number of scheduled frames */
	int			sent_cnt;	/* Number of sent frames */

	struct grtm_ioc_hw	hw_avail;	/* Hardware support available */
	struct grtm_ioc_config	config;
	struct grtm_ioc_stats	stats;

	rtems_id		sem_tx;
};

/* Prototypes */
static void *grtm_memalign(unsigned int boundary, unsigned int length, void *realbuf);
static void grtm_hw_reset(struct grtm_priv *pDev);
static void grtm_interrupt(int irq, void *arg);

/* Common Global Variables */
static rtems_id grtm_dev_sem;
static int grtm_driver_io_registered = 0;
static rtems_device_major_number grtm_driver_io_major = 0;

/******************* Driver manager interface ***********************/

/* Driver prototypes */
static int grtm_register_io(rtems_device_major_number *m);
static int grtm_device_init(struct grtm_priv *pDev);

static int grtm_init2(struct rtems_drvmgr_dev_info *dev);
static int grtm_init3(struct rtems_drvmgr_dev_info *dev);

static struct rtems_drvmgr_drv_ops grtm_ops = 
{
	{NULL, grtm_init2, grtm_init3, NULL},
	NULL,
	NULL
};

static struct amba_dev_id grtm_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_GRTM},
	{0, 0}		/* Mark end of table */
};

static struct amba_drv_info grtm_rmap_drv_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_GRTM_ID,	/* Driver ID */
		"GRTM_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP_RMAP,	/* Bus Type */
		&grtm_ops,
		0,				/* No devices yet */
	},
	&grtm_ids[0]
};

void grtm_rmap_register_drv (void)
{
	DBG("Registering RMAP-GRTM driver\n");
	rtems_drvmgr_drv_register(&grtm_rmap_drv_info.general);
}

static uint32_t grtm_rmap_read_reg(struct grtm_priv *priv, uint32_t *adr)
{
	uint32_t result = 0;
	rtems_drvmgr_read_io32(priv->dev->parent->dev, adr, &result);
	return result;
}

static int grtm_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct grtm_priv *priv;

	DBG("GRTM[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);
	priv = dev->priv = malloc(sizeof(struct grtm_priv));
	if ( !priv )
		return DRVMGR_NOMEM;
	memset(priv, 0, sizeof(*priv));
	priv->dev = dev;

	/* This core will not find other cores, so we wait for init2() */

	return DRVMGR_OK;
}

static int grtm_init3(struct rtems_drvmgr_dev_info *dev)
{
	struct grtm_priv *priv;
	char prefix[16];
	rtems_status_code status;

	priv = dev->priv;

	/* Do initialization */

	if ( grtm_driver_io_registered == 0) {
		/* Register the I/O driver only once for all cores */
		if ( grtm_register_io(&grtm_driver_io_major) ) {
			/* Failed to register I/O driver */
			dev->priv = NULL;
			return DRVMGR_FAIL;
		}

		grtm_driver_io_registered = 1;
	}

	/* I/O system registered and initialized 
	 * Now we take care of device initialization.
	 */
	if ( grtm_device_init(priv) ) {
		return DRVMGR_FAIL;
	}

	/* Get Filesystem name prefix */
	prefix[0] = '\0';
	if ( rtems_drvmgr_get_dev_prefix(dev, prefix) ) {
		/* Failed to get prefix, make sure of a unique FS name
		 * by using the driver minor.
		 */
		sprintf(priv->devName, "/dev/grtm%d", dev->minor_drv);
	} else {
		/* Got special prefix, this means we have a bus prefix
		 * And we should use our "bus minor"
		 */
		sprintf(priv->devName, "/dev/%sgrtm%d", prefix, dev->minor_bus);
	}

	/* Register Device */
	status = rtems_io_register_name(priv->devName, grtm_driver_io_major, dev->minor_drv);
	if (status != RTEMS_SUCCESSFUL) {
		return DRVMGR_FAIL;
	}

	return DRVMGR_OK;
}

/******************* Driver Implementation ***********************/

static int grtm_register_io(rtems_device_major_number *m)
{
	rtems_status_code r;

	if ((r = rtems_io_register_driver(0, &grtm_driver, m)) == RTEMS_SUCCESSFUL) {
		DBG("GRTM driver successfully registered, major: %d\n", *m);
	} else {
		switch(r) {
		case RTEMS_TOO_MANY:
			printk("GRTM rtems_io_register_driver failed: RTEMS_TOO_MANY\n");
			return -1;
		case RTEMS_INVALID_NUMBER:  
			printk("GRTM rtems_io_register_driver failed: RTEMS_INVALID_NUMBER\n");
			return -1;
		case RTEMS_RESOURCE_IN_USE:
			printk("GRTM rtems_io_register_driver failed: RTEMS_RESOURCE_IN_USE\n");
			return -1;
		default:
			printk("GRTM rtems_io_register_driver failed\n");
			return -1;
		}
	}
	return 0;
}

static int grtm_device_init(struct grtm_priv *pDev)
{
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;
	union rtems_drvmgr_key_value *value;

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)pDev->dev->businfo;
	if ( ambadev == NULL ) {
		return -1;
	}
	pnpinfo = &ambadev->info;
	pDev->irq = pnpinfo->irq;
	pDev->regs = (struct grtm_regs *)pnpinfo->apb_slv->start;
	pDev->minor = pDev->dev->minor_drv;
	pDev->open = 0;
	pDev->running = 0;

	/* Create Binary RX Semaphore with count = 0 */
	if ( rtems_semaphore_create(rtems_build_name('G', 'R', 'M', '0' + pDev->minor),
		0,
		RTEMS_FIFO|RTEMS_SIMPLE_BINARY_SEMAPHORE|RTEMS_NO_INHERIT_PRIORITY|\
		RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING, 
		0,
		&pDev->sem_tx) != RTEMS_SUCCESSFUL ) {
		return -1;
	}

	/* Create ISR protection Counting Semaphore with count = 1 */
	if ( rtems_semaphore_create(rtems_build_name('G', 'T', 'M', '0' + pDev->minor),
		1,
		RTEMS_FIFO | RTEMS_COUNTING_SEMAPHORE | RTEMS_PRIORITY | \
		RTEMS_LOCAL | RTEMS_NO_PRIORITY_CEILING,
		0,
		&pDev->handling_transmission) != RTEMS_SUCCESSFUL ) {
		return -1;
	}

	/* Override default 2k MAX FRAME length if available from bus resource */
	pDev->frame_length_max = 2*1024;
	value = rtems_drvmgr_dev_key_get(pDev->dev, "maxFrameLength", KEY_TYPE_INT);
	if ( value )
		pDev->frame_length_max = value->i;

	/* Default to allocate from partition 0 */
	pDev->alloc_part_bd = 0;
	pDev->alloc_part_frm = 0;
	value = rtems_drvmgr_dev_key_get(pDev->dev, "bdAllocPartition", KEY_TYPE_INT);
	if ( value )
		pDev->alloc_part_bd = value->i;
	value = rtems_drvmgr_dev_key_get(pDev->dev, "frameAllocPartition", KEY_TYPE_INT);
	if ( value )
		pDev->alloc_part_frm = value->i;

	/* Allocate Memory for Descriptors */
#ifdef REMOTE_DESCRIPTORS
	pDev->bds = 0xA0000000;
	pDev->_bds = 0xA0000000;
#else
	pDev->_bds = pDev->bds = (struct grtm_bd *)
		ambapp_rmap_partition_memalign(pDev->dev, pDev->alloc_part_bd, 0x400, 0x400);
#endif
	if ( !pDev->bds ) {
		DBG("GRTM: Failed to allocate descriptor table\n");
		return -1;
	}
	MEMSET(pDev, pDev->bds, 0, 0x400);

	pDev->_ring = malloc(sizeof(struct grtm_ring) * 128);
	if ( !pDev->_ring ) {
		return -1;
	}
	pDev->_frame_buffers = (unsigned char *) ambapp_rmap_partition_memalign(
		pDev->dev, pDev->alloc_part_frm, 0x4, DESCRIPTOR_MAX * pDev->frame_length_max);
	if ( !pDev->_frame_buffers ) {
		return -1;
	}

	/* Reset Hardware before attaching IRQ handler */
	grtm_hw_reset(pDev);

	/* Read SUB revision number, ignore  */
	pDev->subrev = (READ_REG(pDev, &pDev->regs->revision) & GRTM_REV1_REV_SREV)
			>> GRTM_REV1_REV_SREV_BIT;

	/* Register interrupt handler */
	if ( rtems_drvmgr_interrupt_register(pDev->dev, 0, grtm_interrupt, pDev) ) {
		return -1;
	}

	return 0;
}


static inline void grtm_list_clr(struct grtm_list *list)
{
	list->head = NULL;
	list->tail = NULL;
}

static void grtm_hw_reset(struct grtm_priv *pDev)
{
	/* Reset Core */
	WRITE_REG(pDev, &pDev->regs->dma_ctrl, GRTM_DMA_CTRL_RST);
}

static void grtm_hw_get_implementation(struct grtm_priv *pDev, struct grtm_ioc_hw *hwcfg)
{
	struct grtm_regs *regs = pDev->regs;
	unsigned int cfg = READ_REG(pDev, &regs->cfg);

	hwcfg->cs	= (cfg & GRTM_CFG_SC)	? 1:0;
	hwcfg->sp	= (cfg & GRTM_CFG_SP)	? 1:0;
	hwcfg->ce	= (cfg & GRTM_CFG_CE)	? 1:0;
	hwcfg->nrz	= (cfg & GRTM_CFG_NRZ)	? 1:0;
	hwcfg->psr	= (cfg & GRTM_CFG_PSR)	? 1:0;
	hwcfg->te	= (cfg & GRTM_CFG_TE)	? 1:0;
	hwcfg->rsdep	= (cfg & GRTM_CFG_RSDEP)>>GRTM_CFG_RSDEP_BIT;
	hwcfg->rs	= (cfg & GRTM_CFG_RS)>>GRTM_CFG_RS_BIT;
	hwcfg->aasm	= (cfg & GRTM_CFG_AASM)	? 1:0;
	hwcfg->fecf	= (cfg & GRTM_CFG_FECF)	? 1:0;
	hwcfg->ocf	= (cfg & GRTM_CFG_OCF)	? 1:0;
	hwcfg->evc	= (cfg & GRTM_CFG_EVC)	? 1:0;
	hwcfg->idle	= (cfg & GRTM_CFG_IDLE)	? 1:0;
	hwcfg->fsh	= (cfg & GRTM_CFG_FSH)	? 1:0;
	hwcfg->mcg	= (cfg & GRTM_CFG_MCG)	? 1:0;
	hwcfg->iz	= (cfg & GRTM_CFG_IZ)	? 1:0;
	hwcfg->fhec	= (cfg & GRTM_CFG_FHEC)	? 1:0;
	hwcfg->aos	= (cfg & GRTM_CFG_AOS)	? 1:0;
	hwcfg->cif	= (cfg & GRTM_CFG_CIF)	? 1:0;
	hwcfg->ocfb	= (cfg & GRTM_CFG_OCFB)	? 1:0;
	

	cfg = READ_REG(pDev, &regs->dma_cfg);
	hwcfg->blk_size	= (cfg & GRTM_DMA_CFG_BLKSZ) >> GRTM_DMA_CFG_BLKSZ_BIT;
	hwcfg->fifo_size= (cfg & GRTM_DMA_CFG_FIFOSZ) >> GRTM_DMA_CFG_FIFOSZ_BIT;
}

#warning Extra: Implement proper default calculation from hardware configuration
static void grtm_hw_get_default_modes(struct grtm_ioc_config *cfg, struct grtm_ioc_hw *hwcfg)
{
	cfg->mode = GRTM_MODE_TM;
	cfg->frame_length = 223;
	cfg->limit = 0; /* Make driver auto configure it on START, user may override with non-zero value */
	cfg->as_marker = 0x1ACFFC1D;

	/* Physical */
	cfg->phy_subrate = 1;
	cfg->phy_symbolrate = 1;
	cfg->phy_opts = 0;

	/* Coding Layer */
	cfg->code_rsdep = 1;
	cfg->code_ce_rate = 0;
	cfg->code_csel = 0;
	cfg->code_opts = 0;

	/* All Frame Generation */
	cfg->all_izlen = 0;
	cfg->all_opts = GRTM_IOC_ALL_FECF;

	/* Master Channel Frame Generation */
	if ( hwcfg->mcg ) {
		cfg->mf_opts = GRTM_IOC_MF_MC;
	} else {
		cfg->mf_opts = 0;
	}

	/* Idle Frame Generation */
	cfg->idle_scid = 0;
	cfg->idle_vcid = 0;
	if ( hwcfg->idle ) {
		cfg->idle_opts = GRTM_IOC_IDLE_EN;
	} else {
		cfg->idle_opts = 0;
	}

	/* Interrupt options */
	cfg->blocking = 0;	/* non-blocking mode is default */
	cfg->enable_cnt = 16;	/* generate interrupt every 16 descriptor */
	cfg->isr_desc_proc = 1;	/* Enable interrupt handler from doing descriptor processing, this
				 * is not efficient since a semaphore must be used to lock the free_sent()
				 * and grtm_schedule_ready() functions, because they are locking when
				 * depending on the RMAP stack */
	cfg->timeout = RTEMS_NO_TIMEOUT;
	
}

static int grtm_hw_set_config(struct grtm_priv *pDev, struct grtm_ioc_config *cfg, struct grtm_ioc_hw *hwcfg)
{
	struct grtm_regs *regs = pDev->regs;
	unsigned int tmp;
	unsigned int limit;

	/* Check that the buffers that we have allocated allows the new frame length */
	if ( cfg->frame_length > pDev->frame_length_max )
		return -1;

	if ( cfg->limit == 0 ) {
		/* Calculate Limit */
		if ( cfg->frame_length > hwcfg->blk_size ) {
			limit = hwcfg->blk_size*2;
		} else {
			limit = cfg->frame_length;
		}
	} else {
		/* Use user configured limit */
		limit = cfg->limit;
	}

	/* Frame Length and Limit */
	tmp = (((limit-1) << GRTM_DMA_LEN_LIM_BIT) & GRTM_DMA_LEN_LIM)|
			(((cfg->frame_length-1) << GRTM_DMA_LEN_LEN_BIT) & GRTM_DMA_LEN_LEN);
	WRITE_REG(pDev, &regs->dma_len, tmp);

	/* Physical layer options */
	tmp =	(cfg->phy_opts & (GRTM_IOC_PHY_SCF|GRTM_IOC_PHY_SF)) | 
		(((cfg->phy_symbolrate-1)<<GRTM_PHY_SYM_BIT) & GRTM_PHY_SYM) | (((cfg->phy_subrate-1)<<GRTM_PHY_SUB_BIT) & GRTM_PHY_SUB);
	WRITE_REG(pDev, &regs->phy, tmp);

	/* Coding Sub-layer Options */
	tmp =	(cfg->code_opts & GRTM_IOC_CODE_ALL) | ((cfg->code_csel<<GRTM_CODE_CSEL_BIT) & GRTM_CODE_CSEL) |
		(((cfg->code_rsdep-1)<<GRTM_CODE_RSDEP_BIT) & GRTM_CODE_RSDEP) | ((cfg->code_ce_rate<<GRTM_CODE_CERATE_BIT) & GRTM_CODE_CERATE);
	WRITE_REG(pDev, &regs->code, tmp);

	/* Attached synchronization marker register */
	WRITE_REG(pDev, &regs->asmr, cfg->as_marker);

	/* All Frames Generation */
	tmp =	((cfg->all_opts & GRTM_IOC_ALL_ALL)<<14) | 
		((cfg->all_izlen<<GRTM_ALL_IZLEN_BIT) & GRTM_ALL_IZLEN) |
		((cfg->mode<<GRTM_ALL_VER_BIT) & GRTM_ALL_VER);
	WRITE_REG(pDev, &regs->all_frm, tmp);

	/* Master Frame Generation */
	WRITE_REG(pDev, &regs->mst_frm, cfg->mf_opts & GRTM_IOC_MF_ALL);

	/* Idle frame Generation */
	tmp =	((cfg->idle_opts & GRTM_IOC_IDLE_ALL) << 16) |
		((cfg->idle_vcid << GRTM_IDLE_VCID_BIT) & GRTM_IDLE_VCID) |
		((cfg->idle_scid << GRTM_IDLE_SCID_BIT) & GRTM_IDLE_SCID);
	WRITE_REG(pDev, &regs->idle_frm, tmp);

	return 0;
}

static int grtm_start(struct grtm_priv *pDev)
{
	struct grtm_regs *regs = pDev->regs;
	int i;
	struct grtm_ioc_config *cfg = &pDev->config;
	volatile unsigned int *txrdy_reg;
	unsigned int txrdy_mask;
	unsigned int frame_buffer_ofs;

	/* Clear Descriptors */
	MEMSET(pDev, pDev->bds, 0, 0x400);

	/* Clear stats */
	memset(&pDev->stats,0,sizeof(struct grtm_ioc_stats));

	/* Init Descriptor Ring */
	memset(pDev->_ring, 0, sizeof(struct grtm_ring)*DESCRIPTOR_MAX);
	frame_buffer_ofs = 0;
	for(i=0; i<DESCRIPTOR_MAX-1; i++){
		pDev->_ring[i].next = &pDev->_ring[i+1];
		pDev->_ring[i].bd = &pDev->bds[i];
		pDev->_ring[i].frm = NULL;
		pDev->_ring[i].buf = &pDev->_frame_buffers[frame_buffer_ofs];
		WRITE_REG(pDev, &pDev->_ring[i].bd->address, pDev->_ring[i].buf); /* INIT BD ADDRESS */
		frame_buffer_ofs += pDev->frame_length_max;
	}
	pDev->_ring[DESCRIPTOR_MAX-1].next = &pDev->_ring[0];
	pDev->_ring[DESCRIPTOR_MAX-1].bd = &pDev->bds[DESCRIPTOR_MAX-1];
	pDev->_ring[DESCRIPTOR_MAX-1].frm = NULL;
	pDev->_ring[DESCRIPTOR_MAX-1].buf = &pDev->_frame_buffers[frame_buffer_ofs];
	WRITE_REG(pDev, &pDev->_ring[DESCRIPTOR_MAX-1].bd->address, pDev->_ring[DESCRIPTOR_MAX-1].buf); /* INIT BD ADDRESS */

	pDev->ring = &pDev->_ring[0];
	pDev->ring_end = &pDev->_ring[0];

	/* Clear Scheduled, Ready and Sent list */
	grtm_list_clr(&pDev->ready);
	grtm_list_clr(&pDev->scheduled);
	grtm_list_clr(&pDev->sent);

	/* Software init */
	/*pDev->handling_transmission = 0;*/

	/* Reset the transmitter */
	WRITE_REG(pDev, &regs->dma_ctrl, GRTM_DMA_CTRL_TXRST);
	WRITE_REG(pDev, &regs->dma_ctrl, 0);	/* Leave Reset */

	/* Clear old interrupts */
	WRITE_REG(pDev, &regs->dma_status, GRTM_DMA_STS_ALL);

	/* Set Descriptor Pointer Base register to point to first descriptor */
	WRITE_REG(pDev, &regs->dma_bd, pDev->bds);
	/*rtems_drvmgr_mmap_translate(pDev->dev, 0, (void *)pDev->bds, (void **)&regs->dma_bd);*/
	/*regs->dma_bd = (unsigned int)pDev->bds;*/

	/* Set hardware options as defined by config */
	if ( grtm_hw_set_config(pDev, cfg, &pDev->hw_avail) ) {
		return RTEMS_IO_ERROR;
	}

	/* Enable TM Transmitter */
	WRITE_REG(pDev, &regs->ctrl, GRTM_CTRL_EN);

	/* Wait for TXRDY to be cleared */
	i=1000;
	while( i > 0 ) {
		asm volatile ("nop"::);
		i--;
	}

	/* Location of TXRDY Bit is different for different revisions */
	if ( pDev->subrev == 0 ) {
		txrdy_reg = &regs->dma_ctrl;
		txrdy_mask = GRTM_REV0_DMA_CTRL_TXRDY;
	} else {
		txrdy_reg = &regs->dma_status;
		txrdy_mask = GRTM_REV1_DMA_STS_TXRDY;
	}

	/* Check transmitter startup OK */
	i=0;
	while( !(READ_REG(pDev, txrdy_reg) & txrdy_mask) && (i<1000) ){
		i++;
	}
	if ( !(READ_REG(pDev, txrdy_reg) & txrdy_mask) ){
		/* Reset Failed */
		DBG("GRTM: start: Reseting transmitter failed (%d)\n",i);
		return RTEMS_IO_ERROR;
	}
	DBG("GRTM: reset time %d\n",i);

	/* Everything is configured, the TM transmitter is started
	 * and idle frames has been sent.
	 */

	/* Mark running before enabling the DMA transmitter */
	pDev->running = 1;

	/* Enable interrupts (Error and DMA TX) */
	WRITE_REG(pDev, &regs->dma_ctrl, GRTM_DMA_CTRL_IE);

	DBG("GRTM: STARTED\n");

	return RTEMS_SUCCESSFUL;
}

static void grtm_stop(struct grtm_priv *pDev)
{
	struct grtm_regs *regs = pDev->regs;

	/* Disable the transmitter & Interrupts */
	WRITE_REG(pDev, &regs->dma_ctrl, 0);

	/* Clear any pending interrupt  */
	WRITE_REG(pDev, &regs->dma_status, GRTM_DMA_STS_ALL);

	DBG("GRTM: STOPPED\n");

	/* Flush semaphore in case a thread is stuck waiting for TX Interrupts */
	rtems_semaphore_flush(pDev->sem_tx);
}

static rtems_device_driver grtm_open(
	rtems_device_major_number major, 
	rtems_device_minor_number minor, 
	void *arg)
{
	struct grtm_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&grtm_rmap_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct grtm_priv *)dev->priv;
	
	/* Wait until we get semaphore */
	if ( rtems_semaphore_obtain(grtm_dev_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT) != RTEMS_SUCCESSFUL ){
		return RTEMS_INTERNAL_ERROR;
	}

	/* Is device in use? */
	if ( pDev->open ){
		rtems_semaphore_release(grtm_dev_sem);
		return RTEMS_RESOURCE_IN_USE;
	}
	
	/* Mark device taken */
	pDev->open = 1;
	
	rtems_semaphore_release(grtm_dev_sem);
	
	DBG("grtm_open: OPENED minor %d (pDev: 0x%x)\n",pDev->minor,(unsigned int)pDev);
	
	/* Set defaults */
	pDev->config.timeout = RTEMS_NO_TIMEOUT;	/* no timeout (wait forever) */
	pDev->config.blocking = 0;			/* polling mode */
	
	pDev->running = 0;				/* not in running mode yet */

	memset(&pDev->config,0,sizeof(pDev->config));
	
	/* The core has been reset when we execute here, so it is possible
	 * to read out what HW is implemented from core.
	 */
	grtm_hw_get_implementation(pDev, &pDev->hw_avail);

	/* Get default modes */
	grtm_hw_get_default_modes(&pDev->config,&pDev->hw_avail);
	
	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver grtm_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	struct grtm_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&grtm_rmap_drv_info.general, minor, &dev) ) {
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct grtm_priv *)dev->priv;

	if ( pDev->running ){
		grtm_stop(pDev);
		pDev->running = 0;
	}
	
	/* Reset core */
	grtm_hw_reset(pDev);

	/* Clear descriptor area just for sure */
	MEMSET(pDev, pDev->bds, 0, 0x400);

	/* Mark not open */
	pDev->open = 0;

	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver grtm_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	FUNCDBG();
	return RTEMS_NOT_IMPLEMENTED;
}

static rtems_device_driver grtm_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
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
static int grtm_free_sent(struct grtm_priv *pDev)
{
	struct grtm_ring *curr;
	struct grtm_frame *last_frm, *first_frm;
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
	while ( curr->frm && !((ctrl=READ_REG(pDev, &curr->bd->ctrl)) & GRTM_BD_EN) ){
		/* Handle one sent Frame */
		
		/* Remember last handled frame so that insertion/removal from
		 * frames lists go fast.
		 */
		last_frm = curr->frm;
		
		/* 1. Set flags to indicate error(s) and other information */
		last_frm->flags |= GRTM_FLAGS_SENT; /* Mark sent */
		
		/* Update Stats */
		pDev->stats.frames_sent++;
    
		/* Did packet encounter link error? */
		if ( ctrl & GRTM_BD_UE ) {
			pDev->stats.err_underrun++;
			last_frm->flags |= GRRM_FLAGS_ERR;
		}

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


/* Moves as many frames in the ready queue (as there are free descriptors for)
 * to the scheduled queue. The free descriptors are then assigned one frame
 * each and enabled for transmission.
 * 
 * Return Value
 * Returns number of frames moved from ready to scheduled queue 
 */
static int grtm_schedule_ready(struct grtm_priv *pDev, int ints_off)
{
	int cnt;
	unsigned int ctrl, dmactrl, oldLevel;
	struct grtm_ring *curr_bd;
	struct grtm_frame *curr_frm, *last_frm;

	if ( !pDev->ready.head ){
		return 0;
	}

	cnt=0;
	curr_frm = pDev->ready.head;
	curr_bd = pDev->ring;
	while( !curr_bd->frm ){
		/* Assign frame to descriptor */
		curr_bd->frm = curr_frm;

#warning REMOVE TRANSLATE FLAG?
		/* Is frame located at SpaceWire target or at the local RAM? */
		if ( curr_frm->flags & GRTM_FLAGS_COPY_DATA ) {
			/* Transfer frame to the target via SpW, we make sure that is aligned to a 4byte
			 * boundary, this is good practice and does not cost anything.
			 */
			TRANSFER_FRM(pDev, curr_bd->buf, curr_frm->payload, (pDev->config.frame_length + 3) & ~0x3);

			/* That the BD->address is not constant */
			WRITE_REG(pDev, &curr_bd->bd->address, curr_bd->buf);
		} else {
			/* The Frame has already been copied to the SpaceWire target, we simply
			 * write the address of the frame.
			 */
			WRITE_REG(pDev, &curr_bd->bd->address, curr_frm->payload);
		}
#if 0
		/* Prepare descriptor address. Three cases:
		 *  - GRTM core on same bus as CPU ==> no translation (Address used by CPU = address used by GRTM)
		 *  - GRTM core on remote bus, and payload address given as used by CPU ==> Translation needed
		 *  - GRTM core on remote bus, and payload address given as used by GRTM ==> no translation  [ USER does custom translation]
		 */
		if ( curr_frm->flags & (GRTM_FLAGS_TRANSLATE|GRTM_FLAGS_TRANSLATE_AND_REMEMBER) ) {
			/* Do translation */
			rtems_drvmgr_mmap_translate(pDev->dev, 0, (void *)curr_frm->payload, (void **)&curr_bd->bd->address);
			if ( curr_frm->flags & GRTM_FLAGS_TRANSLATE_AND_REMEMBER ) {
				if ( curr_frm->payload != curr_bd->bd->address ) {
					/* Translation needed */
					curr_frm->flags &= ~GRTM_FLAGS_TRANSLATE_AND_REMEMBER;
					curr_frm->flags |= GRTM_FLAGS_TRANSLATE;
				} else {
					/* No Trnaslation needed */
					curr_frm->flags &= ~(GRTM_FLAGS_TRANSLATE|GRTM_FLAGS_TRANSLATE_AND_REMEMBER);
				}
			}
		} else {
			/* Custom translation or no translation needed */
			curr_bd->bd->address = (unsigned int)curr_frm->payload;
		}
#endif

		ctrl = GRTM_BD_EN;
		if ( curr_bd->next == pDev->_ring ){
			ctrl |= GRTM_BD_WR; /* Wrap around */
		}
		/* Apply user options/flags */
		ctrl |= (curr_frm->flags & GRTM_FLAGS_MASK);

		/* Is this Frame going to be an interrupt Frame? */
		if ( (--pDev->enable_cnt_curr) <= 0 ){
			if ( pDev->config.enable_cnt == 0 ){
				pDev->enable_cnt_curr = 0x3fffffff;
			}else{
				pDev->enable_cnt_curr = pDev->config.enable_cnt;
				ctrl |= GRTM_BD_IE;
			}
		}

		/* Enable descriptor */
		WRITE_REG(pDev, &curr_bd->bd->ctrl, ctrl);

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
		dmactrl = READ_REG(pDev, &pDev->regs->dma_ctrl);
		dmactrl &= ~(GRTM_DMA_CTRL_TXRST | GRTM_DMA_CTRL_RST);
		dmactrl |= GRTM_DMA_CTRL_EN;
		WRITE_REG(pDev, &pDev->regs->dma_ctrl, dmactrl);
		
		if ( !ints_off ) {
			IRQ_GLOBAL_ENABLE(oldLevel);
		}
	}
	return cnt;
}


static rtems_device_driver grtm_ioctl(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	struct grtm_priv *pDev;
	struct rtems_drvmgr_dev_info *dev;
	rtems_libio_ioctl_args_t *ioarg = (rtems_libio_ioctl_args_t *)arg;
	unsigned int *data = ioarg->buffer;
	int status;
	struct grtm_ioc_config *cfg;
	struct grtm_ioc_hw_status *hwregs;
	IRQ_GLOBAL_PREPARE(oldLevel);
	struct grtm_list *chain;
	struct grtm_frame *curr;
	struct grtm_ioc_hw *hwimpl;
	struct grtm_ioc_stats *stats;
	int num,ret;

	FUNCDBG();

	if ( rtems_drvmgr_get_dev(&grtm_rmap_drv_info.general, minor, &dev) ) {
		return RTEMS_INVALID_NUMBER;
	}
	pDev = (struct grtm_priv *)dev->priv;

	if (!ioarg)
		return RTEMS_INVALID_NAME;

	ioarg->ioctl_return = 0;
	switch(ioarg->command) {
		case GRTM_IOC_START:
		if ( pDev->running ) {
			return RTEMS_RESOURCE_IN_USE; /* EBUSY */
		}
		if ( (status=grtm_start(pDev)) != RTEMS_SUCCESSFUL ){
			return status;
		}
		/* Enable interrupt */
		rtems_drvmgr_interrupt_enable(dev, 0, grtm_interrupt, pDev);

		/* Read and write are now open... */
		break;

		case GRTM_IOC_STOP:
		if ( !pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}

		/* Disable interrupts */
		rtems_drvmgr_interrupt_disable(dev, 0, grtm_interrupt, pDev);
		grtm_stop(pDev);
		pDev->running = 0;
		break;

		case GRTM_IOC_ISSTARTED:
		if ( !pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}
		break;

		case GRTM_IOC_SET_BLOCKING_MODE:
		if ( (unsigned int)data > GRTM_BLKMODE_BLK ) {
			return RTEMS_INVALID_NAME;
		}
		DBG("GRTM: Set blocking mode: %d\n",(unsigned int)data);
		pDev->config.blocking = (unsigned int)data;
		break;

		case GRTM_IOC_SET_TIMEOUT:
		DBG("GRTM: Timeout: %d\n",(unsigned int)data);
		pDev->config.timeout = (rtems_interval)data;
		break;

		case GRTM_IOC_SET_CONFIG:
		cfg = (struct grtm_ioc_config *)data;
		if ( !cfg ) {
			return RTEMS_INVALID_NAME;
		}
		
		if ( pDev->running ) {
			return RTEMS_RESOURCE_IN_USE;
		}

		pDev->config = *cfg;
		break;

		case GRTM_IOC_GET_STATS:
		stats = (struct grtm_ioc_stats *)data;
		if ( !stats ) {
			return RTEMS_INVALID_NAME;
		}
		memcpy(stats,&pDev->stats,sizeof(struct grtm_ioc_stats));
		break;

		case GRTM_IOC_CLR_STATS:
		memset(&pDev->stats,0,sizeof(struct grtm_ioc_stats));
		break;

		case GRTM_IOC_GET_CONFIG:
		cfg = (struct grtm_ioc_config *)data;
		if ( !cfg ) {
			return RTEMS_INVALID_NAME;
		}

		*cfg = pDev->config;
		break;

		case GRTM_IOC_GET_OCFREG:
		if ( !pDev->hw_avail.ocf ) {
			/* Hardware does not implement the OCF register */
			return RTEMS_NOT_DEFINED;
		}
		if ( !data ) {
			return RTEMS_INVALID_NAME;
		}
#warning THIS IOCTL COPY THE REMOTE ADDRESS
		*(unsigned int **)data = (unsigned int *)&pDev->regs->ocf;
		break;

		case GRTM_IOC_GET_HW_IMPL:
		hwimpl = (struct grtm_ioc_hw *)data;
		if ( !hwimpl ) {
			return RTEMS_INVALID_NAME;
		}
		*hwimpl = pDev->hw_avail;
		break;

		case GRTM_IOC_GET_HW_STATUS:
		hwregs = (struct grtm_ioc_hw_status *)data;
		if ( !hwregs ) {
			return RTEMS_INVALID_NAME;
		}
		/* We disable interrupt in order to get a snapshot of the registers */
		IRQ_GLOBAL_DISABLE(oldLevel);
#warning IMPLEMENT HWREGS HERE
		IRQ_GLOBAL_ENABLE(oldLevel);
		break;

		/* Put a chain of frames at the back of the "Ready frames" queue. This 
		 * triggers the driver to put frames from the Ready queue into unused 
		 * available descriptors. (Ready -> Scheduled)
		 */

		case GRTM_IOC_SEND:
		if ( !pDev->running ){
			return RTEMS_RESOURCE_IN_USE;
		}
		num=0;

		/* Get pointer to frame chain wished be sent */
		chain = (struct grtm_list *)ioarg->buffer;
		if ( !chain ){
			/* No new frames to send ==> just trigger hardware
			 * to send previously made ready frames to be sent.
			 */
			rtems_semaphore_obtain(pDev->handling_transmission, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
			goto trigger_transmission;
		}
		if ( !chain->tail || !chain->head ){
			return RTEMS_INVALID_NAME;
		}

		DBG("GRTM_SEND: head: 0x%x, tail: 0x%x\n",chain->head,chain->tail);

		/* Mark ready frames unsent by clearing GRTM_FLAGS_SENT of all frames */

		curr = chain->head;
		while(curr != chain->tail){
			curr->flags = curr->flags & ~(GRTM_FLAGS_SENT|GRRM_FLAGS_ERR);
			curr = curr->next;
			num++;
		}
		curr->flags = curr->flags & ~(GRTM_FLAGS_SENT|GRRM_FLAGS_ERR);
		num++;

		rtems_semaphore_obtain(pDev->handling_transmission, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
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
		num = grtm_free_sent(pDev);
		pDev->scheduled_cnt -= num;
		pDev->sent_cnt += num;

		/* 3. Use all available free descriptors there are frames for
		 *    in the ready queue.
		 *    (READY->SCHEDULED)
		 */
		num = grtm_schedule_ready(pDev,0);
		pDev->ready_cnt -= num;
		pDev->scheduled_cnt += num;
	
		rtems_semaphore_release(pDev->handling_transmission);
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
		case GRTM_IOC_RECLAIM:
		/* Get pointer to were to place reaped chain */
		chain = (struct grtm_list *)ioarg->buffer;
		if ( !chain ){
			return RTEMS_INVALID_NAME;
		}

		/* Lock out interrupt handler */
		rtems_semaphore_obtain(pDev->handling_transmission, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

		do {
			/* Move sent frames from descriptors to Sent queue. This makes more 
			 * descriptors (BDs) available.
			 */
			num = grtm_free_sent(pDev);
			pDev->scheduled_cnt -= num;
			pDev->sent_cnt += num;
			

			if ( pDev->running ){
				/* Fill descriptors with as many frames from the ready list 
				 * as possible.
				 */
				num = grtm_schedule_ready(pDev,0);
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
					ret = rtems_semaphore_obtain(pDev->sem_tx,RTEMS_WAIT,pDev->config.timeout);
					if ( ret == RTEMS_TIMEOUT ) {
						/* do not lock out interrupt handler any more */
						rtems_semaphore_release(pDev->handling_transmission);
						return RTEMS_TIMEOUT;
					} else if ( ret == RTEMS_SUCCESSFUL ) {
						/* There might be frames available, go check */
						continue;
					} else {
						/* any error (driver closed, internal error etc.) */
						rtems_semaphore_release(pDev->handling_transmission);
						return RTEMS_UNSATISFIED;
					}

				}else{
					/* non-blocking mode, we quit */
					chain->head = NULL;
					chain->tail = NULL;
					/* do not lock out interrupt handler any more */
					rtems_semaphore_release(pDev->handling_transmission);
					return RTEMS_TIMEOUT;
				}
			}else{
				/* Take all sent framess from sent queue to userspace queue */
				chain->head = pDev->sent.head;
				chain->tail = pDev->sent.tail;
				chain->tail->next = NULL; /* Just for sure */

				/* Mark no Sent */
				grtm_list_clr(&pDev->sent);

				DBG("TX_RECLAIM: head: 0x%x, tail: 0x%x\n",chain->head,chain->tail);
				break;
			}

		}while(1);
		
		/* do not lock out interrupt handler any more */
		rtems_semaphore_release(pDev->handling_transmission);
		break;

		default:
		return RTEMS_NOT_DEFINED;
	}
	return RTEMS_SUCCESSFUL;
}

static void grtm_interrupt(int irq, void *arg)
{
	struct grtm_priv *pDev = arg;
	struct grtm_regs *regs = pDev->regs;
	unsigned int status;
	int num;

	/* Clear interrupt by reading it */
	status = READ_REG(pDev, &regs->dma_status);

	/* Spurious Interrupt? */
	if ( !pDev->running )
		return;

	if ( status )
		WRITE_REG(pDev, &regs->dma_status, status);

	if ( status & GRTM_DMA_STS_TFF ){
		pDev->stats.err_transfer_frame++;
	}

	if ( status & GRTM_DMA_STS_TA ){
		pDev->stats.err_ahb++;
	}

	if ( status & GRTM_DMA_STS_TE ){
		pDev->stats.err_tx++;
	}

	if ( status & GRTM_DMA_STS_TI ){

		if ( pDev->config.isr_desc_proc && 
		     (rtems_semaphore_obtain(pDev->handling_transmission, RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) ) {
			/* Free used descriptors and put the sent frame into the "Sent queue"  
			 *   (SCHEDULED->SENT)
			 */
			num = grtm_free_sent(pDev);
			pDev->scheduled_cnt -= num;
			pDev->sent_cnt += num;

			/* Use all available free descriptors there are frames for
			 * in the ready queue.
			 *   (READY->SCHEDULED)
			 */
			num = grtm_schedule_ready(pDev,1);
			pDev->ready_cnt -= num;
			pDev->scheduled_cnt += num;

			rtems_semaphore_release(pDev->handling_transmission);

#if 0
			if ( (pDev->config.blocking==GRTM_BLKMODE_COMPLETE) && pDev->timeout ){
				/* Signal to thread only if enough data is available */
				if ( pDev->wait_for_frames > grtm_data_avail(pDev) ){
					/* Not enough data available */
					goto procceed_processing_interrupts;
				}

				/* Enough number of frames has been transmitted which means that
				 * the waiting thread should be woken up.
				 */
				rtems_semaphore_release(pDev->sem_tx);
			}
#endif
		}

		if ( pDev->config.blocking == GRTM_BLKMODE_BLK ) {
			/* Blocking mode */

#if 0
			/* Disable further Interrupts until handled by waiting task. */
			regs->dma_ctrl = READ_REG(pDev, &regs->dma_ctrl) & ~GRTM_DMA_CTRL_IE;
#endif

			/* Signal Semaphore to wake waiting thread in ioctl(SEND|RECLAIM) */
			rtems_semaphore_release(pDev->sem_tx);
		}

	}

procceed_processing_interrupts:
	;
}

static rtems_device_driver grtm_initialize(
  rtems_device_major_number major, 
  rtems_device_minor_number unused,
  void *arg
  )
{
	/* Device Semaphore created with count = 1 */
	if ( rtems_semaphore_create(rtems_build_name('G', 'R', 'T', 'M'),
		1,
		RTEMS_FIFO|RTEMS_NO_INHERIT_PRIORITY|RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING,
		0,
		&grtm_dev_sem) != RTEMS_SUCCESSFUL ) {
		return RTEMS_INTERNAL_ERROR;
	}

	return RTEMS_SUCCESSFUL;
}
