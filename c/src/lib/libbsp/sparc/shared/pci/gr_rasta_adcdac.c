/*  GR-RASTA-ADCDAC PCI Target driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the GR-RASTA-ADCDAC interface PCI board.
 *  This driver provides a AMBA PnP bus by using the general part
 *  of the AMBA PnP bus driver (ambapp_bus.c).
 *
 *  Driver resources for the AMBA PnP bus provided can be set using
 *  gr_rasta_adcdac_set_resources().
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

#include <gr_rasta_adcdac.h>

/*#define DEBUG 1*/

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

/* PCI ID */
#define PCIID_VENDOR_GAISLER		0x1AC8
#define PCIID_DEVICE_GR_RASTA_ADCDAC	0x0014

int gr_rasta_adcdac_init1(struct rtems_drvmgr_dev_info *dev);
int gr_rasta_adcdac_init2(struct rtems_drvmgr_dev_info *dev);

struct grpci_regs {
	volatile unsigned int cfg_stat;
	volatile unsigned int bar0;
	volatile unsigned int page0;
	volatile unsigned int bar1;
	volatile unsigned int page1;
	volatile unsigned int iomap;
	volatile unsigned int stat_cmd;
};

struct gr_rasta_adcdac_ver {
	const unsigned int	amba_freq_hz;	/* The frequency */
	const unsigned int	amba_ioarea;	/* The address where the PnP IOAREA starts at */
};

/* Private data structure for driver */
struct gr_rasta_adcdac_priv {
	/* Driver management */
	struct rtems_drvmgr_dev_info	*dev;
	char				prefix[20];

	/* PCI */
	unsigned int			bar0;
	unsigned int			bar1;

	/* IRQ */
	unsigned char			irqno;            /* GR-RASTA-ADCDAC System IRQ */
	genirq_t			genirq;

	/* GR-RASTA-ADCDAC */
	struct gr_rasta_adcdac_ver	*version;
	LEON3_IrqCtrl_Regs_Map		*irq;
	struct grpci_regs		*grpci;
	struct rtems_drvmgr_mmap_entry	bus_maps[4];

	/* AMBA Plug&Play information on GR-RASTA-ADCDAC */
	struct ambapp_bus		abus;
	struct ambapp_mmap		amba_maps[4];
        struct ambapp_config		config;
};

struct gr_rasta_adcdac_ver gr_rasta_adcdac_ver0 = {
	.amba_freq_hz		= 50000000,
	.amba_ioarea		= 0x80100000,
};

int ambapp_rasta_adcdac_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg);
int ambapp_rasta_adcdac_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_adcdac_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_adcdac_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_adcdac_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_rasta_adcdac_get_params(
	struct rtems_drvmgr_dev_info *dev,
	struct rtems_drvmgr_bus_params *params);

struct ambapp_ops ambapp_rasta_adcdac_ops = {
	.int_register = ambapp_rasta_adcdac_int_register,
	.int_unregister = ambapp_rasta_adcdac_int_unregister,
	.int_enable = ambapp_rasta_adcdac_int_enable,
	.int_disable = ambapp_rasta_adcdac_int_disable,
	.int_clear = ambapp_rasta_adcdac_int_clear,
	.get_params = ambapp_rasta_adcdac_get_params
};

struct rtems_drvmgr_drv_ops gr_rasta_adcdac_ops = 
{	.init = {gr_rasta_adcdac_init1, gr_rasta_adcdac_init2, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

struct pci_dev_id gr_rasta_adcdac_ids[] = 
{
	{PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_ADCDAC},
	{0, 0}		/* Mark end of table */
};

struct pci_drv_info gr_rasta_adcdac_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_PCI_GAISLER_RASTAADCDAC_ID,/* Driver ID */
		"GR-RASTA-ADCDAC_DRV",		/* Driver Name */
		DRVMGR_BUS_TYPE_PCI,		/* Bus Type */
		&gr_rasta_adcdac_ops,
		0,				/* No devices yet */
	},
	&gr_rasta_adcdac_ids[0]
};

/* Driver resources configuration for the AMBA bus on the GR-RASTA-ADCDAC board.
 * It is declared weak so that the user may override it from the project file,
 * if the default settings are not enough.
 *
 * The configuration consists of an array of configuration pointers, each
 * pointer determine the configuration of one GR-RASTA-ADCDAC board. Pointer
 * zero is for board0, pointer 1 for board1 and so on.
 *
 * The array must end with a NULL pointer.
 */
struct rtems_drvmgr_drv_res *gr_rasta_adcdac_resources[] __attribute__((weak)) =
{
	NULL
};
int gr_rasta_adcdac_resources_cnt = 0;

void gr_rasta_adcdac_register_drv(void)
{
	DBG("Registering GR-RASTA-ADCDAC PCI driver\n");
	rtems_drvmgr_drv_register(&gr_rasta_adcdac_info.general);
}

void gr_rasta_adcdac_isr (int irqno, void *arg)
{
	struct gr_rasta_adcdac_priv *priv = arg;
	unsigned int status, tmp;
	int irq;
	tmp = status = priv->irq->ipend;

	/* DBG("GR-RASTA-ADCDAC: IRQ 0x%x\n",status); */

	for(irq=0; irq<16; irq++) {
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
		rtems_drvmgr_interrupt_clear(priv->dev, 0, gr_rasta_adcdac_isr, (void *)priv);

	DBG("RASTA-ADCDAC-IRQ: 0x%x\n", tmp);
}

/* AMBA PP find routines */
int gr_rasta_adcdac_dev_find(struct ambapp_dev *dev, int index, int maxdepth, void *arg)
{
	/* Found IRQ/GRPCI controller, stop */
	*(struct ambapp_dev **)arg = dev;
	return 1;
}

int gr_rasta_adcdac_hw_init1(struct gr_rasta_adcdac_priv *priv)
{
	unsigned int data;
	unsigned int *page0 = NULL;
	int bus, dev, fun;
	struct pci_dev_info *devinfo;
	unsigned char ver;
	struct ambapp_dev *tmp;
	int status;
	struct ambapp_ahb_info *ahb;
	unsigned int tmpbar, bar0size;

	devinfo = (struct pci_dev_info *)priv->dev->businfo;

	bus = devinfo->bus;
	dev = devinfo->dev;
	fun = devinfo->func;

	pci_read_config_dword(bus, dev, fun, PCI_BASE_ADDRESS_0, &priv->bar0);
	pci_read_config_dword(bus, dev, fun, PCI_BASE_ADDRESS_1, &priv->bar1);
	pci_read_config_byte(bus, dev, fun, PCI_INTERRUPT_LINE, &priv->irqno);

	if ( (priv->bar0 == 0) || (priv->bar1 == 0) ) {
		/* Not all neccessary space assigned to GR-RASTA-ADCDAC target */
		return -1;
	}

	/* Figure out size of BAR0 */
	pci_write_config_dword(bus, dev, fun, PCI_BASE_ADDRESS_0, 0xffffffff);
	pci_read_config_dword(bus, dev, fun, PCI_BASE_ADDRESS_0, &tmpbar);
	pci_write_config_dword(bus, dev, fun, PCI_BASE_ADDRESS_0, priv->bar0);
	tmpbar &= ~0xff; /* Remove lsbits */
	bar0size = (~tmpbar) + 1;

	/* Select version of GR-RASTA-ADCDAC board */
	pci_read_config_byte(bus, dev, fun, PCI_REVISION_ID, &ver);
	switch (ver) {
		case 0:
			priv->version = &gr_rasta_adcdac_ver0;
			break;
		default:
			return -2;
	}

	printf(" PCI BAR[0]: 0x%x, BAR[1]: 0x%x, IRQ: %d\n\n\n", priv->bar0, priv->bar1, priv->irqno);

	page0 = (unsigned int *)(priv->bar0 + bar0size/2); 

	/* Point PAGE0 to start of Plug and Play information */
	*page0 = priv->version->amba_ioarea & 0xf0000000;

#if 0
	/* set parity error response */
	pci_read_config_dword(bus, dev, fun, PCI_COMMAND, &data);
	pci_write_config_dword(bus, dev, fun, PCI_COMMAND, (data|PCI_COMMAND_PARITY));
#endif

	/* Scan AMBA Plug&Play */

	/* AMBA MAP bar0 (in CPU) ==> 0x80000000(remote amba address) */
	priv->amba_maps[0].size = 0x10000000;
	priv->amba_maps[0].local_adr = priv->bar0;
	priv->amba_maps[0].remote_adr = 0x80000000;

	/* AMBA MAP bar1 (in CPU) ==> 0x40000000(remote amba address) */
	priv->amba_maps[1].size = 0x10000000;
	priv->amba_maps[1].local_adr = priv->bar1;
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
	/*ambapp_scan(priv->bar0 + (priv->version->amba_ioarea & ~0xf0000000),
		NULL, &priv->amba_maps[0], NULL, &priv->abus.root, NULL);*/
	ambapp_scan(&priv->abus,
		priv->bar0 + (priv->version->amba_ioarea & ~0xf0000000),
		NULL, &priv->amba_maps[0]);

	/* Initialize Frequency of AMBA bus */
	ambapp_freq_init(&priv->abus, NULL, priv->version->amba_freq_hz);

	/* Point PAGE0 to start of APB area */
	*page0 = 0x80000000;	

	/* Find GRPCI controller */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_PCIFBRG, 10, gr_rasta_adcdac_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -3;
	}
	priv->grpci = (struct grpci_regs *)((struct ambapp_apb_info *)tmp->devinfo)->start;

	priv->grpci->cfg_stat = (priv->grpci->cfg_stat & 0x0fffffff) | 0x40000000;    /* Set GRPCI mmap 0x4 */
	priv->grpci->page1 = 0x40000000;

	/* Find IRQ controller */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_IRQMP, 10, gr_rasta_adcdac_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -4;
	}
	priv->irq = (LEON3_IrqCtrl_Regs_Map *)(((struct ambapp_apb_info *)tmp->devinfo)->start);
	/* Set up GR-RASTA-ADCDAC irq controller */
	priv->irq->iclear = 0xffff;
	priv->irq->ilevel = 0;
	priv->irq->mask[0] = 0;

	priv->bus_maps[0].map_size = priv->amba_maps[0].size;
	priv->bus_maps[0].local_adr = priv->amba_maps[0].local_adr;
	priv->bus_maps[0].remote_adr = priv->amba_maps[0].remote_adr;

	priv->bus_maps[1].map_size = priv->amba_maps[1].size;
	priv->bus_maps[1].local_adr = priv->amba_maps[1].local_adr;
	priv->bus_maps[1].remote_adr = priv->amba_maps[1].remote_adr;

	/* Find GRPCI controller AHB Slave interface */
	tmp = NULL;
	status = ambapp_for_each(priv->abus.root, (OPTIONS_ALL|OPTIONS_AHB_SLVS), VENDOR_GAISLER, GAISLER_PCIFBRG, 10, gr_rasta_adcdac_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -5;
	}
	ahb = (struct ambapp_ahb_info *)tmp->devinfo;
	priv->bus_maps[2].map_size = ahb->mask[0]; /* AMBA->PCI Window on GR-RASTA-ADCDAC board */
	priv->bus_maps[2].local_adr = 0x40000000;
	priv->bus_maps[2].remote_adr = ahb->start[0];

	priv->bus_maps[3].map_size = 0;
	priv->bus_maps[3].local_adr = 0;
	priv->bus_maps[3].remote_adr = 0;

	/* Successfully registered the RASTA board */
	return 0;
}

int gr_rasta_adcdac_hw_init2(struct gr_rasta_adcdac_priv *priv)
{
	int bus, dev, fun;
	struct pci_dev_info *devinfo;

	devinfo = (struct pci_dev_info *)priv->dev->businfo;

	bus = devinfo->bus;
	dev = devinfo->dev;
	fun = devinfo->func;

	/* Enable DMA by enabling PCI target as master */
	pci_master_enable(bus, dev, fun);

	return DRVMGR_OK;
}

/* Called when a PCI target is found with the PCI device and vendor ID 
 * given in gr_rasta_adcdac_ids[].
 */
int gr_rasta_adcdac_init1(struct rtems_drvmgr_dev_info *dev)
{
	struct gr_rasta_adcdac_priv *priv;
	struct pci_dev_info *devinfo;
	int status;

	priv = malloc(sizeof(struct gr_rasta_adcdac_priv));
	if ( !priv )
		return DRVMGR_NOMEM;

	memset(priv, 0, sizeof(*priv));
	dev->priv = priv;
	priv->dev = dev;

	/* Determine number of configurations */
	if ( gr_rasta_adcdac_resources_cnt == 0 ) {
		while ( gr_rasta_adcdac_resources[gr_rasta_adcdac_resources_cnt] )
			gr_rasta_adcdac_resources_cnt++;
	}

	/* Generate Device prefix */

	strcpy(priv->prefix, "/dev/rastaadcdac0");
	priv->prefix[16] += dev->minor_drv;
	mkdir(priv->prefix, S_IRWXU | S_IRWXG | S_IRWXO);
	priv->prefix[17] = '/';
	priv->prefix[18] = '\0';

	devinfo = (struct pci_dev_info *)dev->businfo;
	printf("\n\n--- GR-RASTA-ADCDAC[%d] ---\n", dev->minor_drv);
	printf(" PCI BUS: %d, SLOT: %d, FUNCTION: %d\n", devinfo->bus, devinfo->dev, devinfo->func);
	printf(" PCI VENDOR: 0x%04x, DEVICE: 0x%04x\n", devinfo->id.vendor, devinfo->id.device);

	priv->genirq = genirq_init(16);
	if ( priv->genirq == NULL ) {
		free(priv);
		dev->priv = NULL;
		return DRVMGR_FAIL;
	}

	if ( (status = gr_rasta_adcdac_hw_init1(priv)) != 0 ) {
		genirq_destroy(priv->genirq);
		free(priv);
		dev->priv = NULL;
		printf(" Failed to initialize GR-RASTA-ADCDAC HW: %d\n", status);
		return DRVMGR_FAIL;
	}

	/* Init amba bus */
	priv->config.abus = &priv->abus;
	priv->config.ops = &ambapp_rasta_adcdac_ops;
	priv->config.mmaps = &priv->bus_maps[0];
	if ( priv->dev->minor_drv < gr_rasta_adcdac_resources_cnt ) {
		priv->config.resources = gr_rasta_adcdac_resources[priv->dev->minor_drv];
	} else {
		priv->config.resources = NULL;
	}

	/* Create and register AMBA PnP bus. */
	return ambapp_bus_register(dev, &priv->config);
}

int gr_rasta_adcdac_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct gr_rasta_adcdac_priv *priv = dev->priv;

	/* install interrupt vector */
	rtems_drvmgr_interrupt_register(
		priv->dev,
		0,
		gr_rasta_adcdac_isr,
		(void *)priv);

	/* Enable System IRQ so that GR-RASTA-ADCDAC PCI target interrupt 
	 * goes through.
	 *
	 * It is important to enable it in stage init2. If interrupts were
	 * enabled in init1 this might hang the system when more than one 
	 * PCI board is connected, this is because PCI interrupts might 
	 * be shared and PCI board 2 have not initialized and might
	 * therefore drive interrupt already when entering init1().
	 */
	rtems_drvmgr_interrupt_enable(
		priv->dev,
		0,
		gr_rasta_adcdac_isr, 
		(void *)priv);

	return gr_rasta_adcdac_hw_init2(priv);;
}

int ambapp_rasta_adcdac_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;
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

int ambapp_rasta_adcdac_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_unregister(priv->genirq, irq, isr, arg);
	if ( status != 0 )
		status = -1;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_rasta_adcdac_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	DBG("RASTA-ADCDAC IRQ %d: enable\n", irq);

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

int ambapp_rasta_adcdac_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;
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

int ambapp_rasta_adcdac_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;

	if ( genirq_check(priv->genirq, irq) )
		return -1;

	priv->irq->iclear = (1<<irq);

	return 0;
}

int ambapp_rasta_adcdac_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	struct gr_rasta_adcdac_priv *priv = dev->parent->dev->priv;

	/* Device name prefix pointer, skip /dev */
	params->dev_prefix = &priv->prefix[5];

	return 0;
}

void gr_rasta_adcdac_print_dev(struct rtems_drvmgr_dev_info *dev, int options)
{
	struct gr_rasta_adcdac_priv *priv = dev->priv;
	struct pci_dev_info *devinfo;
	int bus, device, fun;
	int i;

	devinfo = (struct pci_dev_info *)priv->dev->businfo;

	bus = devinfo->bus;
	device = devinfo->dev;
	fun = devinfo->func;

	/* Print */
	printf("--- GR-RASTA-ADCDAC [bus %d, dev %d, fun %d] ---\n", bus, device, fun);
	printf(" PCI BAR0:        0x%x\n", priv->bar0);
	printf(" PCI BAR1:        0x%x\n", priv->bar1);
	printf(" IRQ REGS:        0x%x\n", (unsigned int)priv->irq);
	printf(" IRQ:             %d\n", priv->irqno);
	printf(" FREQ:            %d Hz\n", priv->version->amba_freq_hz);
	printf(" IMASK:           0x%08x\n", priv->irq->mask[0]);
	printf(" IPEND:           0x%08x\n", priv->irq->ipend);

	/* Print amba config */
	if ( options & RASTA_ADCDAC_OPTIONS_AMBA ) {
		ambapp_print(&priv->abus, 10);
	}
#if 0
	/* Print IRQ handlers and their arguments */
	if ( options & RASTA_ADCDAC_OPTIONS_IRQ ) {
		for(i=0; i<16; i++) {
			printf(" IRQ[%02d]:         0x%x, arg: 0x%x\n", 
				i, (unsigned int)priv->isrs[i].handler, (unsigned int)priv->isrs[i].arg);
		}
	}
#endif
}

void gr_rasta_adcdac_print(int options)
{
	struct pci_drv_info *drv = &gr_rasta_adcdac_info;
	struct rtems_drvmgr_dev_info *dev;

	dev = drv->general.dev;
	while(dev) {
		gr_rasta_adcdac_print_dev(dev, options);
		dev = dev->next_in_drv;
	}
}
