/*  GR-RASTA-TMTC PCI Target driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the GR-RASTA-TMTC interface PCI board.
 *  This driver provides a AMBA PnP bus by using the general part
 *  of the AMBA PnP bus driver (ambapp_bus.c).
 *
 *  Driver resources for the AMBA PnP bus provided can be set by overriding
 *  the defaults by declaring gr_rasta_tmtc_resources[].
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2008-12-05, Daniel Hellstrom <daniel@gaisler.com>
 *   Created
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <bsp.h>
#include <rtems/bspIo.h>
#include <pci.h>

#include <ambapp.h>
#include <grlib.h>
#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include <drvmgr/pci_bus.h>
#include <genirq.h>

#include <gr_rasta_tmtc.h>

/* Determines which PCI address the AHB masters will access, it should be
 * set so that the masters can access the CPU RAM. Default is base of CPU RAM,
 * CPU RAM is mapped 1:1 to PCI space.
 */
extern unsigned int _RAM_START;
#define AHBMST2PCIADR (_RAM_START & 0xf0000000)

#define GAISLER_GPIO         0x01a
#define AHB1_BASE_ADDR 0x80000000
#define AHB1_IOAREA_BASE_ADDR 0x80200000

/* #define DEBUG 1 */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

int gr_rasta_tmtc_init1(struct rtems_drvmgr_dev_info *dev);
int gr_rasta_tmtc_init2(struct rtems_drvmgr_dev_info *dev);

struct grpci_regs {
	volatile unsigned int cfg_stat;
	volatile unsigned int bar0;
	volatile unsigned int page0;
	volatile unsigned int bar1;
	volatile unsigned int page1;
	volatile unsigned int iomap;
	volatile unsigned int stat_cmd;
};

struct grgpio_regs {
	volatile unsigned int io_data;		/* 0x00 I/O port data register */
	volatile unsigned int io_output;	/* 0x04 I/O port output register */
	volatile unsigned int io_dir;		/* 0x08 I/O port direction register */
	volatile unsigned int imask;		/* 0x0C Interrupt mask register */
	volatile unsigned int ipol;		/* 0x10 Interrupt polarity register */
	volatile unsigned int iedge;		/* 0x14 Interrupt edge register */
	volatile unsigned int ibypass;		/* 0x18 Bypass register */
};

struct gr_rasta_tmtc_ver {
	const unsigned int	amba_freq_hz;	/* The frequency */
	const unsigned int	amba_ioarea;	/* The address where the PnP IOAREA starts at */
};

/* Private data structure for driver */
struct gr_rasta_tmtc_priv {
	/* Driver management */
	struct rtems_drvmgr_dev_info	*dev;
	char				prefix[20];

	/* PCI */
	pci_dev_t			pcidev;
	struct pci_dev_info		*devinfo;
	uint32_t			ahbmst2pci_map;

	/* IRQ */
	genirq_t			genirq;

	/* GR-RASTA-TMTC */
	struct gr_rasta_tmtc_ver	*version;
	LEON3_IrqCtrl_Regs_Map		*irq;
	struct grpci_regs		*grpci;
	struct grgpio_regs		*gpio;
	struct rtems_drvmgr_mmap_entry	bus_maps[4];

	/* AMBA Plug&Play information on GR-RASTA-TMTC */
	struct ambapp_bus		abus;
	struct ambapp_mmap		amba_maps[4];
        struct ambapp_config		config;
};

struct gr_rasta_tmtc_ver gr_rasta_tmtc_ver0 = {
	.amba_freq_hz		= 30000000,
	.amba_ioarea		= AHB1_IOAREA_BASE_ADDR,
};

int ambapp_rasta_tmtc_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg);
int ambapp_rasta_tmtc_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg);
int ambapp_rasta_tmtc_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_tmtc_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_tmtc_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_tmtc_get_params(
	struct rtems_drvmgr_dev_info *dev,
	struct rtems_drvmgr_bus_params *params);

struct ambapp_ops ambapp_rasta_tmtc_ops = {
	.int_register = ambapp_rasta_tmtc_int_register,
	.int_unregister = ambapp_rasta_tmtc_int_unregister,
	.int_enable = ambapp_rasta_tmtc_int_enable,
	.int_disable = ambapp_rasta_tmtc_int_disable,
	.int_clear = ambapp_rasta_tmtc_int_clear,
	.get_params = ambapp_rasta_tmtc_get_params
};

struct rtems_drvmgr_drv_ops gr_rasta_tmtc_ops = 
{
	.init = {gr_rasta_tmtc_init1, gr_rasta_tmtc_init2, NULL, NULL},
	.remove = NULL,
	.info = NULL,
};

struct pci_dev_id_match gr_rasta_tmtc_ids[] =
{
	PCIID_DEVVEND(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_TMTC),
	PCIID_END_TABLE /* Mark end of table */
};

struct pci_drv_info gr_rasta_tmtc_info =
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_PCI_GAISLER_RASTATMTC_ID,/* Driver ID */
		"GR-RASTA-TMTC_DRV",		/* Driver Name */
		DRVMGR_BUS_TYPE_PCI,		/* Bus Type */
		&gr_rasta_tmtc_ops,
		0,				/* No devices yet */
		sizeof(struct gr_rasta_tmtc_priv) /* Let drvmgr alloc private */
	},
	&gr_rasta_tmtc_ids[0]
};

/* Driver resources configuration for the AMBA bus on the GR-RASTA-TMTC board.
 * It is declared weak so that the user may override it from the project file,
 * if the default settings are not enough.
 *
 * The configuration consists of an array of configuration pointers, each
 * pointer determine the configuration of one GR-RASTA-TMTC board. Pointer
 * zero is for board0, pointer 1 for board1 and so on.
 *
 * The array must end with a NULL pointer.
 */
struct rtems_drvmgr_bus_res *gr_rasta_tmtc_resources[] __attribute__((weak)) =
{
	NULL,
};
int gr_rasta_tmtc_resources_cnt = 0;

void gr_rasta_tmtc_register_drv(void)
{
	DBG("Registering GR-RASTA-TMTC PCI driver\n");
	rtems_drvmgr_drv_register(&gr_rasta_tmtc_info.general);
}

void gr_rasta_tmtc_isr (int irqno, void *arg)
{
	struct gr_rasta_tmtc_priv *priv = arg;
	unsigned int status, tmp;
	int irq;
	tmp = status = priv->irq->ipend;

	/* printk("GR-RASTA-TMTC: IRQ 0x%x\n",status); */

	for(irq=0; irq<32; irq++) {
		if ( status & (1<<irq) ) {
			genirq_doirq(priv->genirq, irq);
			priv->irq->iclear = (1<<irq);
			status &= ~(1<<irq);
			if ( status == 0 )
				break;
		}
	}

	/* ACK interrupt, this is because PCI is Level, so the IRQ Controller still drives the IRQ. */
	if ( tmp )
		rtems_drvmgr_interrupt_clear(priv->dev, 0, gr_rasta_tmtc_isr, (void *)priv);

	DBG("RASTA-TMTC-IRQ: 0x%x\n", tmp);
}

/* AMBA PP find routines */
int gr_rasta_tmtc_dev_find(struct ambapp_dev *dev, int index, int maxdepth, void *arg)
{
	/* Found IRQ/GRPCI controller, stop */
	*(struct ambapp_dev **)arg = dev;
	return 1;
}

int gr_rasta_tmtc_hw_init(struct gr_rasta_tmtc_priv *priv)
{
	uint32_t data;
	unsigned int *page0 = NULL;
	unsigned char ver;
	struct ambapp_dev *tmp;
	int status;
	struct ambapp_ahb_info *ahb;
	unsigned int pci_freq_hz;
	pci_dev_t pcidev = priv->pcidev;
	struct pci_dev_info *devinfo = priv->devinfo;
	uint32_t bar0, bar0_size;

	/* Select version of GR-RASTA-TMTC board */
	switch (devinfo->rev) {
		case 0:
			priv->version = &gr_rasta_tmtc_ver0;
			break;
		default:
			return -2;
	}

	bar0 = devinfo->resources[0].address;
	bar0_size = devinfo->resources[0].size;
	page0 = (unsigned int *)(bar0 + bar0_size/2); 

	/* Point PAGE0 to start of Plug and Play information */
	*page0 = priv->version->amba_ioarea & 0xf0000000;

#if 0
	/* set parity error response */
	pci_cfg_r32(pcidev, PCI_COMMAND, &data);
	pci_cfg_w32(pcidev, PCI_COMMAND, (data|PCI_COMMAND_PARITY));
#endif

	/* Scan AMBA Plug&Play */

	/* AMBA MAP bar0 (in CPU) ==> 0x80000000(remote amba address) */
	priv->amba_maps[0].size = 0x10000000;
	priv->amba_maps[0].local_adr = bar0;
	priv->amba_maps[0].remote_adr = AHB1_BASE_ADDR;

	/* AMBA MAP bar1 (in CPU) ==> 0x40000000(remote amba address) */
	priv->amba_maps[1].size = devinfo->resources[1].size;
	priv->amba_maps[1].local_adr = devinfo->resources[1].address;
	priv->amba_maps[1].remote_adr = 0x40000000;

	/* Addresses not matching with map be untouched */
	priv->amba_maps[2].size = 0xfffffff0;
	priv->amba_maps[2].local_adr = 0;
	priv->amba_maps[2].remote_adr = 0;

	/* Mark end of table */
	priv->amba_maps[3].size=0;
	priv->amba_maps[3].local_adr = 0;
	priv->amba_maps[3].remote_adr = 0;

	/* Start AMBA PnP scan at first AHB bus */
	ambapp_scan(&priv->abus,
		bar0 + (priv->version->amba_ioarea & ~0xf0000000),
		NULL, &priv->amba_maps[0]);

	/* Frequency is the same as the PCI bus frequency */
	rtems_drvmgr_freq_get(priv->dev, 0, &pci_freq_hz);

	/* Initialize Frequency of AMBA bus */
	ambapp_freq_init(&priv->abus, NULL, pci_freq_hz);

	/* Point PAGE0 to start of APB area */
	*page0 = AHB1_BASE_ADDR;

	/* Find GRPCI controller */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_PCIFBRG, 10, gr_rasta_tmtc_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -3;
	}
	priv->grpci = (struct grpci_regs *)((struct ambapp_apb_info *)tmp->devinfo)->start;

	/* Set GRPCI mmap so that AMBA masters can access CPU-RAM over
	 * the PCI window.
	 */
	priv->grpci->cfg_stat = (priv->grpci->cfg_stat & 0x0fffffff) |
				(priv->ahbmst2pci_map & 0xf0000000);
	priv->grpci->page1 = 0x40000000;

	/* Find IRQ controller, Clear all current IRQs */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_IRQMP, 10, gr_rasta_tmtc_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -4;
	}
	priv->irq = (LEON3_IrqCtrl_Regs_Map *)(((struct ambapp_apb_info *)tmp->devinfo)->start);
	/* Set up GR-RASTA-TMTC irq controller */
	priv->irq->mask[0] = 0;
	priv->irq->iclear = 0xffffffff;
	priv->irq->ilevel = 0;

	/* Find First GPIO controller */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_GPIO, 10, gr_rasta_tmtc_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -5;
	}
	priv->gpio = (struct grgpio_regs *) (((struct ambapp_apb_info *)tmp->devinfo)->start);
	/* Clear GR-RASTA-TMTC GPIO controller */
	priv->gpio->imask = 0;
	priv->gpio->ipol = 0;
	priv->gpio->iedge = 0;
	priv->gpio->ibypass = 0;
	/* Set up GR-RASTA-TMTC GPIO controller to select GRTM and GRTC */
	priv->gpio->io_output = (GR_TMTC_GPIO_GRTM_SEL|GR_TMTC_GPIO_TRANSP_CLK) | (GR_TMTC_GPIO_TC_BIT_LOCK|GR_TMTC_GPIO_TC_RF_AVAIL|GR_TMTC_GPIO_TC_ACTIVE_HIGH|GR_TMTC_GPIO_TC_RISING_CLK);
	priv->gpio->io_dir = 0xffffffff;
	DBG("GR-TMTC GPIO: 0x%x\n", (unsigned int)priv->gpio);

	/* Enable DMA by enabling PCI target as master */
	pci_master_enable(pcidev);

	priv->bus_maps[0].map_size = priv->amba_maps[0].size;
	priv->bus_maps[0].local_adr = priv->amba_maps[0].local_adr;
	priv->bus_maps[0].remote_adr = priv->amba_maps[0].remote_adr;

	priv->bus_maps[1].map_size = priv->amba_maps[1].size;
	priv->bus_maps[1].local_adr = priv->amba_maps[1].local_adr;
	priv->bus_maps[1].remote_adr = priv->amba_maps[1].remote_adr;

	/* Find GRPCI controller AHB Slave interface */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_AHB_SLVS), VENDOR_GAISLER, GAISLER_PCIFBRG, 10, gr_rasta_tmtc_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -6;
	}
	ahb = (struct ambapp_ahb_info *)tmp->devinfo;
	priv->bus_maps[2].map_size = ahb->mask[0]; /* AMBA->PCI Window on GR-RASTA-TMTC board */
	priv->bus_maps[2].local_adr = 0x40000000;
	priv->bus_maps[2].remote_adr = ahb->start[0];

	priv->bus_maps[3].map_size = 0;
	priv->bus_maps[3].local_adr = 0;
	priv->bus_maps[3].remote_adr = 0;

	/* Successfully registered the RASTA board */
	return 0;
}

void gr_rasta_tmtc_hw_init2(struct gr_rasta_tmtc_priv *priv)
{
	/* Enable DMA by enabling PCI target as master */
	pci_master_enable(priv->pcidev);
}

/* Called when a PCI target is found with the PCI device and vendor ID 
 * given in gr_rasta_tmtc_ids[].
 */
int gr_rasta_tmtc_init1(struct rtems_drvmgr_dev_info *dev)
{
	struct gr_rasta_tmtc_priv *priv;
	struct pci_dev_info *devinfo;
	int status;
	uint32_t bar0, bar1, bar0_size, bar1_size;
	union rtems_drvmgr_key_value *value;

	priv = dev->priv;
	if (!priv)
		return DRVMGR_NOMEM;
	priv->dev = dev;

	/* Determine number of configurations */
	if ( gr_rasta_tmtc_resources_cnt == 0 ) {
		while ( gr_rasta_tmtc_resources[gr_rasta_tmtc_resources_cnt] )
			gr_rasta_tmtc_resources_cnt++;
	}

	/* Generate Device prefix */

	strcpy(priv->prefix, "/dev/rastatmtc0");
	priv->prefix[14] += dev->minor_drv;
	mkdir(priv->prefix, S_IRWXU | S_IRWXG | S_IRWXO);
	priv->prefix[15] = '/';
	priv->prefix[16] = '\0';

	priv->devinfo = devinfo = (struct pci_dev_info *)dev->businfo;
	priv->pcidev = devinfo->pcidev;
	bar0 = devinfo->resources[0].address;
	bar0_size = devinfo->resources[0].size;
	bar1 = devinfo->resources[1].address;
	bar1_size = devinfo->resources[1].size;
	printf("\n\n--- GR-RASTA-TMTC[%d] ---\n", dev->minor_drv);
	printf(" PCI BUS: 0x%x, SLOT: 0x%x, FUNCTION: 0x%x\n",
		PCI_DEV_EXPAND(priv->pcidev));
	printf(" PCI VENDOR: 0x%04x, DEVICE: 0x%04x\n",
		devinfo->id.vendor, devinfo->id.device);
	printf(" PCI BAR[0]: 0x%lx - 0x%lx\n", bar0, bar0 + bar0_size - 1);
	printf(" PCI BAR[1]: 0x%lx - 0x%lx\n", bar1, bar1 + bar1_size - 1);
	printf(" IRQ: %d\n\n\n", devinfo->irq);

	/* all neccessary space assigned to GR-RASTA-IO target? */
	if ((bar0_size == 0) || (bar1_size == 0))
		return DRVMGR_ENORES;

	/* Let user override which PCI address the AHB masters of the
	 * GR-RASTA-TMTC board access when doing DMA to CPU RAM. The AHB masters
	 * access the PCI Window of the AMBA bus, the MSB 4-bits of that address
	 * is translated according this config option before the address
	 * goes out on the PCI bus.
	 * Only the 4 MSB bits have an effect;
	 */
	value = rtems_drvmgr_dev_key_get(priv->dev, "ahbmst2pci", KEY_TYPE_INT);
	if (value)
		priv->ahbmst2pci_map = value->i;
	else
		priv->ahbmst2pci_map = AHBMST2PCIADR; /* default */	

	priv->genirq = genirq_init(32);
	if ( priv->genirq == NULL )
		return DRVMGR_FAIL;

	if ( status = gr_rasta_tmtc_hw_init(priv) ) {
		genirq_destroy(priv->genirq);
		printf(" Failed to initialize GR-RASTA-TMTC HW: %d\n", status);
		return DRVMGR_FAIL;
	}

	/* Init amba bus */
	priv->config.abus = &priv->abus;
	priv->config.ops = &ambapp_rasta_tmtc_ops;
	priv->config.mmaps = &priv->bus_maps[0];
	if ( priv->dev->minor_drv < gr_rasta_tmtc_resources_cnt ) {
		priv->config.resources = gr_rasta_tmtc_resources[priv->dev->minor_drv];
	} else {
		priv->config.resources = NULL;
	}

	return ambapp_bus_register(dev, &priv->config);
}

int gr_rasta_tmtc_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct gr_rasta_tmtc_priv *priv = dev->priv;

	/* install interrupt vector */
	rtems_drvmgr_interrupt_register(priv->dev, 0, gr_rasta_tmtc_isr, (void *)priv);

	/* Clear any old interrupt requests */
	rtems_drvmgr_interrupt_clear(priv->dev, 0, gr_rasta_tmtc_isr, (void *)priv);

	/* Enable System IRQ so that GR-RASTA-TMTC PCI target interrupt goes
	 * through.
	 *
	 * It is important to enable it in stage init2. If interrupts were
	 * enabled in init1 this might hang the system when more than one
	 * PCI target is connected, this is because PCI interrupts might 
	 * be shared and PCI board 2 have not initialized and
	 * might therefore drive interrupt already when entering init1().
	 */
	rtems_drvmgr_interrupt_enable(dev, 0, gr_rasta_tmtc_isr, (void *)priv);

	gr_rasta_tmtc_hw_init2(priv);

	return DRVMGR_OK;
}

int ambapp_rasta_tmtc_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_register(priv->genirq, irq, handler, arg);
	if ( status == 0 ) {
		/* Disable and clear IRQ for first registered handler */
		priv->irq->iclear = (1<<irq);
		priv->irq->mask[0] &= ~(1<<irq); /* mask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_rasta_tmtc_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_unregister(priv->genirq, irq, isr, arg);
	if ( status != 0 )
		status = -1;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_rasta_tmtc_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_enable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Enable IRQ for first enabled handler only */
		priv->irq->mask[0] |= (1<<irq); /* unmask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_rasta_tmtc_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	DBG("RASTA-ADCDAC IRQ %d: disable\n", irq);

	rtems_interrupt_disable(level);

	status = genirq_disable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Disable IRQ only when no enabled handler exists */
		priv->irq->mask[0] &= ~(1<<irq); /* mask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_rasta_tmtc_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;

	if ( genirq_check(priv->genirq, irq) )
		return -1;

	priv->irq->iclear = (1<<irq);

	return 0;
}

int ambapp_rasta_tmtc_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	struct gr_rasta_tmtc_priv *priv = dev->parent->dev->priv;

	/* Device name prefix pointer, skip /dev */
	params->dev_prefix = &priv->prefix[5];

	return 0;
}

void gr_rasta_tmtc_print_dev(struct rtems_drvmgr_dev_info *dev, int options)
{
	struct gr_rasta_tmtc_priv *priv = dev->priv;
	struct pci_dev_info *devinfo = priv->devinfo;
	uint32_t bar0, bar1, bar0_size, bar1_size;

	/* Print */
	printf("--- GR-RASTA-TMTC [bus 0x%x, dev 0x%x, fun 0x%x] ---\n",
		PCI_DEV_EXPAND(priv->pcidev));

	bar0 = devinfo->resources[0].address;
	bar0_size = devinfo->resources[0].size;
	bar1 = devinfo->resources[1].address;
	bar1_size = devinfo->resources[1].size;

	printf(" PCI BAR[0]: 0x%lx - 0x%lx\n", bar0, bar0 + bar0_size - 1);
	printf(" PCI BAR[1]: 0x%lx - 0x%lx\n", bar1, bar1 + bar1_size - 1);
	printf(" IRQ:             %d\n", devinfo->irq);
	printf(" PCI REVISION:    %d\n", devinfo->rev);
	printf(" FREQ:            %d Hz\n", priv->version->amba_freq_hz);
	printf(" IMASK:           0x%08x\n", priv->irq->mask[0]);
	printf(" IPEND:           0x%08x\n", priv->irq->ipend);

	/* Print amba config */
	if ( options & RASTA_TMTC_OPTIONS_AMBA ) {
		ambapp_print(&priv->abus, 10);
	}

#if 0
	/* Print IRQ handlers and their arguments */
	if ( options & RASTA_TMTC_OPTIONS_IRQ ) {
		int i;
		for(i=0; i<16; i++) {
			printf(" IRQ[%02d]:         0x%x, arg: 0x%x\n", 
				i, (unsigned int)priv->isrs[i].handler, (unsigned int)priv->isrs[i].arg);
		}
	}
#endif
}

void gr_rasta_tmtc_print(int options)
{
	struct pci_drv_info *drv = &gr_rasta_tmtc_info;
	struct rtems_drvmgr_dev_info *dev;

	dev = drv->general.dev;
	while(dev) {
		gr_rasta_tmtc_print_dev(dev, options);
		dev = dev->next_in_drv;
	}
}