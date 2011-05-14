/*  GR-701 PCI Target driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the GR-701 interface PCI board.
 *  This driver provides a AMBA PnP bus by using the general part
 *  of the AMBA PnP bus driver (ambapp_bus.c).
 *
 *  Driver resources for the AMBA PnP bus provided can be set using
 *  gr701_set_resources().
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
#include <pci/access.h>

#include <ambapp.h>

#include <ambapp.h>
#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include <drvmgr/pci_bus.h>
#include <genirq.h>

#include <gr_701.h>

/* Offset from 0x80000000 (dual bus version) */
#define AHB1_BASE_ADDR 0x80000000
#define AHB1_IOAREA_BASE_ADDR 0x80100000

/* #define DEBUG 1 */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

int gr701_init1(struct rtems_drvmgr_dev_info *dev);
int gr701_init2(struct rtems_drvmgr_dev_info *dev);

#define READ_REG(address) (*(volatile unsigned int *)address)

/* PCI bride reg layout on AMBA side */
struct amba_bridge_regs {
	volatile unsigned int bar0;
	volatile unsigned int bar1;
	volatile unsigned int bar2;
	volatile unsigned int bar3;
	volatile unsigned int bar4;/* 0x10 */
	
	volatile unsigned int unused[4*3-1];
	
	volatile unsigned int ambabars[1]; /* 0x40 */
};

/* PCI bride reg layout on PCI side */
struct pci_bridge_regs {
	volatile unsigned int bar0;
	volatile unsigned int bar1;
	volatile unsigned int bar2;
	volatile unsigned int bar3;
	volatile unsigned int bar4; /* 0x10 */

	volatile unsigned int ilevel;
	volatile unsigned int ipend;
	volatile unsigned int iforce;
	volatile unsigned int istatus;
	volatile unsigned int iclear;
	volatile unsigned int imask;
};

/* Private data structure for driver */
struct gr701_priv {
	/* Driver management */
	struct rtems_drvmgr_dev_info	*dev;
	char				prefix[16];

	struct pci_bridge_regs		*pcib;
	struct amba_bridge_regs		*ambab;

	/* PCI */
	pci_dev_t			pcidev;
	struct pci_dev_info		*devinfo;	

	/* IRQ */
	genirq_t			genirq;
	int				interrupt_cnt;

	/* GR-701 */
	struct rtems_drvmgr_mmap_entry	bus_maps[3];

	/* AMBA Plug&Play information on GR-701 */
	struct ambapp_bus		abus;
	struct ambapp_mmap		amba_maps[3];
        struct ambapp_config		config;
};

int ambapp_gr701_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg);
int ambapp_gr701_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_gr701_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_gr701_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_gr701_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg);
int ambapp_gr701_get_params(
	struct rtems_drvmgr_dev_info *dev,
	struct rtems_drvmgr_bus_params *params);

struct ambapp_ops ambapp_gr701_ops = {
	.int_register = ambapp_gr701_int_register,
	.int_unregister = ambapp_gr701_int_unregister,
	.int_enable = ambapp_gr701_int_enable,
	.int_disable = ambapp_gr701_int_disable,
	.int_clear = ambapp_gr701_int_clear,
	.get_params = ambapp_gr701_get_params
};

struct rtems_drvmgr_drv_ops gr701_ops = 
{
	.init = {gr701_init1, gr701_init2, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

struct pci_dev_id_match gr701_ids[] = 
{
	PCIID_DEVVEND(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_701),
	PCIID_END_TABLE /* Mark end of table */
};

struct pci_drv_info gr701_info =
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_PCI_GAISLER_GR701_ID,	/* Driver ID */
		"GR-701_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_PCI,		/* Bus Type */
		&gr701_ops,
		0,				/* No devices yet */
	},
	&gr701_ids[0]
};

/* Driver resources configuration for the AMBA bus on the GR-701 board.
 * It is declared weak so that the user may override it from the project file,
 * if the default settings are not enough.
 *
 * The configuration consists of an array of configuration pointers, each
 * pointer determine the configuration of one GR-701 board. Pointer
 * zero is for board0, pointer 1 for board1 and so on.
 *
 * The array must end with a NULL pointer.
 */
struct rtems_drvmgr_drv_res *gr701_resources[] __attribute__((weak)) =
{
	NULL
};
int gr701_resources_cnt = 0;

void gr701_register_drv(void)
{
	DBG("Registering GR-701 PCI driver\n");
	rtems_drvmgr_drv_register(&gr701_info.general);
}

void gr701_interrupt(int irqno, struct gr701_priv *priv)
{
	unsigned int status;
	struct gr701_isr *isr;
	int irq = 0;

	while ( (status=priv->pcib->istatus) != 0 ) {
		priv->interrupt_cnt++;	/* An interrupt was generated */
		irq = status;
		genirq_doirq(priv->genirq, irq);
		/* ACK interrupt */
		priv->pcib->istatus = 0;
	}

	/* ACK interrupt, this is because PCI is Level, so the IRQ Controller still drives the IRQ. */
	if ( irq )
		rtems_drvmgr_interrupt_clear(priv->dev, 0, gr701_interrupt, (void *)priv);
}

/* AMBA PP find routines */
int gr701_dev_find(struct ambapp_dev_hdr *dev, int index, int maxdepth, void *arg)
{
	/* Found IRQ/PCI controller, stop */
	*(struct ambapp_dev_hdr **)arg = dev;
	return 1;
}

int gr701_hw_init(struct gr701_priv *priv)
{
	unsigned int com1;
	struct pci_bridge_regs *pcib;
	struct amba_bridge_regs *ambab;
	int mst;
	unsigned int pci_freq_hz;
	pci_dev_t pcidev = priv->pcidev;
	struct pci_dev_info *devinfo = priv->devinfo;

	/* Set up PCI ==> AMBA */
	priv->pcib = pcib = (void *)devinfo->resources[0].address;
	pcib->bar0 = 0xfc000000;

	/* Set up GR701 AMBA Masters connection to PCI */
	priv->ambab = ambab = (struct amba_bridge_regs *)(
		devinfo->resources[1].address + 0x400);

	/* Init all msters, max 16 */
	for (mst=0; mst<16; mst++) {
		ambab->ambabars[mst] = 0x40000000;
		if (READ_REG(&ambab->ambabars[mst]) != 0x40000000)
			break;
	}

	priv->amba_maps[0].size = 0x04000000;
	priv->amba_maps[0].local_adr = devinfo->resources[1].address;
	priv->amba_maps[0].remote_adr = 0xfc000000;

	/* Mark end of table */
	priv->amba_maps[1].size=0;
	priv->amba_maps[1].local_adr = 0;
	priv->amba_maps[1].remote_adr = 0;

	priv->bus_maps[0].map_size = priv->amba_maps[0].size;
	priv->bus_maps[0].local_adr = priv->amba_maps[0].local_adr;
	priv->bus_maps[0].remote_adr = priv->amba_maps[0].remote_adr;

	/* Mark end of table */
	priv->bus_maps[1].map_size = 0;
	priv->bus_maps[1].local_adr = 0;
	priv->bus_maps[1].remote_adr = 0;

	/* Enable I/O and Mem accesses */
	pci_cfg_r32(pcidev, PCI_COMMAND, &com1);
	com1 |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
	pci_cfg_w32(pcidev, PCI_COMMAND, com1);

	/* Start AMBA PnP scan at first AHB bus */
	/*ambapp_scan(priv->bars[1] + 0x3f00000,
		NULL, &priv->amba_maps[0], NULL, &priv->abus.root, NULL);*/
	ambapp_scan(&priv->abus, devinfo->resources[1].address + 0x3f00000,
			NULL, &priv->amba_maps[0]);

	/* Frequency is the same as the PCI bus frequency */
	rtems_drvmgr_freq_get(priv->dev, NULL, &pci_freq_hz);

	/* Initialize Frequency of AMBA bus */
	ambapp_freq_init(&priv->abus, NULL, pci_freq_hz);

	/* Init IRQ controller (avoid IRQ generation) */
	pcib->imask = 0x0000;
	pcib->ipend = 0;
	pcib->iclear = 0xffff;
	pcib->iforce = 0;
	pcib->ilevel = 0x0;

	/* Successfully registered the GR-701 board */
	return 0;
}

void gr701_hw_init2(struct gr701_priv *priv)
{
	/* install PCI interrupt routine */
	rtems_drvmgr_interrupt_register(priv->dev, 0, gr701_interrupt, priv);

	/* Clear any old interrupt requests */
	rtems_drvmgr_interrupt_clear(priv->dev, 0, gr701_interrupt, priv);

	/* Enable System IRQ so that GR-701 PCI target interrupt goes through.
	 *
	 * It is important to enable it in stage init2. If interrupts were
	 * enabled in init1 this might hang the system when more than one PCI
	 * board is connected, this is because PCI interrupts might be shared
	 * and PCI target 2 have not initialized and might therefore drive
	 * interrupt already when entering init1().
	 */
	rtems_drvmgr_interrupt_enable(priv->dev, 0, gr701_interrupt, priv);

	/* Enable PCI Master (for DMA) */
	pci_master_enable(priv->pcidev);
}

/* Called when a PCI target is found with the PCI device and vendor ID 
 * given in gr701_ids[].
 */
int gr701_init1(struct rtems_drvmgr_dev_info *dev)
{
	struct gr701_priv *priv;
	struct pci_dev_info *devinfo;
	uint32_t bar0, bar1, bar0_size, bar1_size;

	priv = malloc(sizeof(struct gr701_priv));
	if ( !priv )
		return DRVMGR_NOMEM;

	memset(priv, 0, sizeof(*priv));
	dev->priv = priv;
	priv->dev = dev;

	/* Determine number of configurations */
	if ( gr701_resources_cnt == 0 ) {
		while ( gr701_resources[gr701_resources_cnt] )
			gr701_resources_cnt++;
	}

	/* Generate Device prefix */

	strcpy(priv->prefix, "/dev/gr701_0");
	priv->prefix[11] += dev->minor_drv;
	mkdir(priv->prefix, S_IRWXU | S_IRWXG | S_IRWXO);
	priv->prefix[12] = '/';
	priv->prefix[13] = '\0';

	priv->devinfo = devinfo = (struct pci_dev_info *)dev->businfo;
	priv->pcidev = devinfo->pcidev;
	bar0 = devinfo->resources[0].address;
	bar0_size = devinfo->resources[0].size;
	bar1 = devinfo->resources[1].address;
	bar1_size = devinfo->resources[1].size;
	printf("\n\n--- GR-701[%d] ---\n", dev->minor_drv);
	printf(" PCI BUS: 0x%x, SLOT: 0x%x, FUNCTION: 0x%x\n",
		PCI_DEV_EXPAND(priv->pcidev));
	printf(" PCI VENDOR: 0x%04x, DEVICE: 0x%04x\n\n\n",
		devinfo->id.vendor, devinfo->id.device);
	printf(" PCI BAR[0]: 0x%x - 0x%x\n", bar0, bar0 + bar0_size - 1);
	printf(" PCI BAR[1]: 0x%x - 0x%x\n", bar1, bar1 + bar1_size - 1);
	printf(" IRQ: %d\n\n\n", devinfo->irq);

	/* all neccessary space assigned to GR-701 target? */
	if ((bar0_size == 0) || (bar1_size == 0))
		return DRVMGR_ENORES;

	priv->genirq = genirq_init(16);
	if ( priv->genirq == NULL ) {
		free(priv);
		dev->priv = NULL;
		return DRVMGR_FAIL;
	}

	if ( gr701_hw_init(priv) ) {
		genirq_destroy(priv->genirq);
		free(priv);
		dev->priv = NULL;
		printf(" Failed to initialize GR-701 HW\n");
		return DRVMGR_FAIL;
	}

	/* Init amba bus */
	priv->config.abus = &priv->abus;
	priv->config.ops = &ambapp_gr701_ops;
	priv->config.mmaps = &priv->bus_maps[0];
	if ( priv->dev->minor_drv < gr701_resources_cnt ) {
		priv->config.resources = gr701_resources[priv->dev->minor_drv];
	} else {
		priv->config.resources = NULL;
	}

	/* Create and register AMBA PnP bus. */
	return ambapp_bus_register(dev, &priv->config);
}

/* Called when a PCI target is found with the PCI device and vendor ID 
 * given in gr701_ids[].
 */
int gr701_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct gr701_priv *priv = dev->priv;

	/* Register Interrupt handler and clear IRQ at interrupt controller */
	gr701_hw_init2(priv);

	return DRVMGR_OK;
}

int ambapp_gr701_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr handler,
	void *arg)
{
	struct gr701_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_register(priv->genirq, irq, handler, arg);
	if ( status == 0 ) {
		/* Disable and clear IRQ for first registered handler */
		priv->pcib->iclear = (1<<irq);
		priv->pcib->imask &= ~(1<<irq); /* mask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_gr701_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr701_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	rtems_interrupt_disable(level);

	status = genirq_unregister(priv->genirq, irq, isr, arg);
	if ( status != 0 )
		status = -1;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_gr701_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr701_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	DBG("GR-701 IRQ %d: enable\n", irq);

	rtems_interrupt_disable(level);

	status = genirq_enable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Enable IRQ for first enabled handler only */
		priv->pcib->imask |= (1<<irq); /* unmask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_gr701_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr701_priv *priv = dev->parent->dev->priv;
	rtems_interrupt_level level;
	int status;

	DBG("GR-701 IRQ %d: disable\n", irq);

	rtems_interrupt_disable(level);

	status = genirq_disable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Disable IRQ only when no enabled handler exists */
		priv->pcib->imask &= ~(1<<irq); /* mask interrupt source */
	} else if ( status == 1 )
		status = 0;

	rtems_interrupt_enable(level);

	return status;
}

int ambapp_gr701_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int irq,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct gr701_priv *priv = dev->parent->dev->priv;

	if ( genirq_check(priv->genirq, irq) )
		return -1;

	priv->pcib->iclear = (1<<irq);

	return 0;
}

int ambapp_gr701_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	struct gr701_priv *priv = dev->parent->dev->priv;

	/* Device name prefix pointer, skip /dev */
	params->dev_prefix = &priv->prefix[5];

	return 0;
}

void gr701_print_dev(struct rtems_drvmgr_dev_info *dev, int options)
{
	struct gr701_priv *priv = dev->priv;
	struct pci_dev_info *devinfo = priv->devinfo;
	int i;
	unsigned int freq_hz;
	uint32_t bar0, bar1, bar0_size, bar1_size;

	/* Print */
	printf("--- GR-701 [bus 0x%x, dev 0x%x, fun 0x%x] ---\n",
		PCI_DEV_EXPAND(priv->pcidev));
	bar0 = devinfo->resources[0].address;
	bar0_size = devinfo->resources[0].size;
	bar1 = devinfo->resources[1].address;
	bar1_size = devinfo->resources[1].size;

	printf(" PCI BAR[0]: 0x%x - 0x%x\n", bar0, bar0 + bar0_size - 1);
	printf(" PCI BAR[1]: 0x%x - 0x%x\n", bar1, bar1 + bar1_size - 1);
	printf(" IRQ:             %d\n", devinfo->irq);

	/* Frequency is the same as the PCI bus frequency */
	rtems_drvmgr_freq_get(dev, 0, &freq_hz);

	printf(" FREQ:            %d Hz\n", freq_hz);
	printf(" IMASK:           0x%08x\n", priv->pcib->imask);
	printf(" IPEND:           0x%08x\n", priv->pcib->ipend);

	/* Print amba config */
	if ( options & GR701_OPTIONS_AMBA ) {
		ambapp_print(&priv->abus, 10);
	}

#if 0
	/* Print IRQ handlers and their arguments */
	if ( options & GR701_OPTIONS_IRQ ) {
		for(i=0; i<16; i++) {
			printf(" IRQ[%02d]:         0x%x, arg: 0x%x\n", 
				i, (unsigned int)priv->isrs[i].handler, (unsigned int)priv->isrs[i].arg);
		}
	}
#endif
}

void gr701_print(int options)
{
	struct pci_drv_info *drv = &gr701_info;
	struct rtems_drvmgr_dev_info *dev;

	dev = drv->general.dev;
	while(dev) {
		gr701_print_dev(dev, options);
		dev = dev->next_in_drv;
	}
}
