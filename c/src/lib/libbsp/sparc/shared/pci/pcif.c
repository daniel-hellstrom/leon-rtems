/*  GRLIB PCIF PCI HOST driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the PCIF core and initialize,
 *   - the PCI Library (pci.c)
 *   - the general part of the PCI Bus driver (pci_bus.c)
 *  
 *  System interrupt assigned to PCI interrupt (INTA#..INTD#) is by
 *  default taken from Plug and Play, but may be overridden by the 
 *  driver resources INTA#..INTD#.
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2008-12-06, Daniel Hellstrom <daniel@gaisler.com>
 *   Created
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include <ambapp.h>
#include <drvmgr/pci_bus.h>

#include <pci.h>
#include <rtems/bspIo.h>
#include <string.h>

/* Configuration options */
#define SYSTEM_MAINMEM_START 0x40000000

/* Interrupt assignment. Set to other value than 0xff in order to 
 * override defaults and plug&play information
 */
#ifndef PCIF_INTA_SYSIRQ
 #define PCIF_INTA_SYSIRQ 0xff
#endif
#ifndef PCIF_INTB_SYSIRQ
 #define PCIF_INTB_SYSIRQ 0xff
#endif
#ifndef PCIF_INTC_SYSIRQ
 #define PCIF_INTC_SYSIRQ 0xff
#endif
#ifndef PCIF_INTD_SYSIRQ
 #define PCIF_INTD_SYSIRQ 0xff
#endif

/*#define DEBUG 1  */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

/*
 * Bit encode for PCI_CONFIG_HEADER_TYPE register
 */
struct pcif_regs {
	volatile unsigned int bars[4];  /* 0x00-0x10 */
	volatile unsigned int bus;      /* 0x10 */
	volatile unsigned int map_io;   /* 0x14 */
	volatile unsigned int status;   /* 0x18 */
	volatile unsigned int intr;     /* 0x1c */
	int unused[(0x40-0x20)/4];      /* 0x20-0x40 */
	volatile unsigned int maps[(0x80-0x40)/4];   /* 0x40-0x80*/
};

struct pcif_priv *pcifpriv = NULL;
static int pcif_minor = 0;

/* PCI Interrupt assignment. Connects an PCI interrupt pin (INTA#..INTD#)
 * to a system interrupt number.
 */
unsigned char pcif_pci_irq_table[4] =
{
	/* INTA# */	PCIF_INTA_SYSIRQ,
	/* INTB# */	PCIF_INTB_SYSIRQ,
	/* INTC# */	PCIF_INTC_SYSIRQ,
	/* INTD# */	PCIF_INTD_SYSIRQ
};

/* Driver private data struture */
struct pcif_priv {
	struct rtems_drvmgr_dev_info	*dev;
	struct pcif_regs		*regs;
	int				irq;
	int				minor;
	int				irq_mask;

	unsigned int			pci_area;
	unsigned int			pci_area_end;
	unsigned int			pci_io;    
	unsigned int			pci_conf;
	unsigned int			pci_conf_end;

	unsigned int			devVend; /* PCI Device and Vendor ID of Host */
};

int pcif_init1(struct rtems_drvmgr_dev_info *dev);
int pcif_init3(struct rtems_drvmgr_dev_info *dev);

/* PCIF DRIVER */

struct rtems_drvmgr_drv_ops pcif_ops = 
{
	.init = {pcif_init1, NULL, pcif_init3, NULL},
	.remove = NULL,
	.info = NULL
};

struct amba_dev_id pcif_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_PCIF},
	{0, 0}		/* Mark end of table */
};

struct amba_drv_info pcif_info =
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_PCIF_ID,	/* Driver ID */
		"PCIF_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&pcif_ops,
		0,				/* No devices yet */
		sizeof(struct pcif_priv),	/* Let drvmgr alloc private */
	},
	&pcif_ids[0]
};

void pcif_register_drv(void)
{
	DBG("Registering PCIF driver\n");
	rtems_drvmgr_drv_register(&pcif_info.general);
}


/*  The configuration access functions uses the DMA functionality of the
 *  GRLIB PCIF PCI controller to be able access all slots
 */

int
pcif_read_config_dword(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char offset,
  unsigned int *val
)
{
	struct pcif_priv *priv = pcifpriv;
	volatile unsigned int *pci_conf;

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	if (slot > 21) {
		*val = 0xffffffff;
		return PCIBIOS_SUCCESSFUL;
	}

	priv->regs->bus = bus << 16;

	pci_conf = (volatile unsigned int *) (priv->pci_conf +
		((slot<<11) | (function<<8) | offset));

	*val = *pci_conf;

	if (priv->regs->status & 0x30000000) {
		*val = 0xffffffff;
	}

	DBG("pci_read - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset,  (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), *val); 

	return PCIBIOS_SUCCESSFUL;
}

int 
pcif_read_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short *val)
{
	unsigned int v = 0;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	pcif_read_config_dword(bus, slot, function, offset&~3, &v);
	*val = 0xffff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}

int 
pcif_read_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char *val)
{
	unsigned int v = 0;

	pcif_read_config_dword(bus, slot, function, offset&~3, &v);

	*val = 0xff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}

int
pcif_write_config_dword(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned int val)
{
	struct pcif_priv *priv = pcifpriv;
	volatile unsigned int *pci_conf;
	unsigned int value;

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	priv->regs->bus = bus << 16;

	pci_conf = (volatile unsigned int *) (priv->pci_conf +
	         ((slot<<11) | (function<<8) | (offset & ~3)));

	value = val;

	*pci_conf = value;

	DBG("pci write - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset, (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), value); 

	return PCIBIOS_SUCCESSFUL;
}

int 
pcif_write_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short val) {
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	pcif_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xffff << (8*(offset&3)))) | ((0xffff&val) << (8*(offset&3)));

	return pcif_write_config_dword(bus, slot, function, offset&~3, v);
}

int 
pcif_write_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char val) {
	unsigned int v;

	pcif_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xff << (8*(offset&3)))) | ((0xff&val) << (8*(offset&3)));

	return pcif_write_config_dword(bus, slot, function, offset&~3, v);
}

/* Return the assigned system IRQ number that corresponds to the PCI "Interrupt Pin"
 * information from configuration space.
 *
 * The IRQ information is stored in the pcif_pci_irq_table configurable
 * by the user.
 *
 * Returns the "system IRQ" for the PCI INTA#..INTD# pin in irq_pin. Returns
 * 0xff if not assigned.
 */
unsigned char pcif_get_assigned_irq(
	unsigned char bus,
	unsigned char slot,
	unsigned char func,
	unsigned char irq_pin
	)
{
	unsigned char sysIrqNr = 0xff; /* not assigned */

	if ( (irq_pin >= 1) && (irq_pin <= 4) ) {
		/* Valid PCI "Interrupt Pin" number */
		sysIrqNr = pcif_pci_irq_table[irq_pin-1];
	}
	return sysIrqNr;
}

/* Access functions used by pci_ functions to access core specific 
 * functions.
 */
const pci_config_access_functions pcif_access_functions = {
	pcif_read_config_byte,
	pcif_read_config_word,
	pcif_read_config_dword,
	pcif_write_config_byte,
	pcif_write_config_word,
	pcif_write_config_dword,
	pcif_get_assigned_irq
};

/* Initializes the PCIF core hardware
 *
 */
int pcif_hw_init(struct pcif_priv *priv)
{
	struct pcif_regs *regs;
	unsigned int data;
	int mst;

	regs = priv->regs;

	/* Mask PCI interrupts */
	regs->intr = 0;

	/* Get the PCIF Host PCI ID */
	pcif_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &priv->devVend);

	/* set 1:1 mapping between AHB -> PCI memory space, for all Master cores */
	for ( mst=0; mst<16; mst++) {
		regs->maps[mst] = priv->pci_area;

		/* Check if this register is implemented */
		if ( regs->maps[mst] != priv->pci_area )
			break;
	}

	/* and map system RAM at pci address SYSTEM_MAINMEM_START. This way
	 * PCI targets can do DMA directly into CPU main memory.
	 */
	regs->bars[0] = SYSTEM_MAINMEM_START;
	regs->bars[1] = 0;
	regs->bars[2] = 0;
	regs->bars[3] = 0;

	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_0, 0);
	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_1, SYSTEM_MAINMEM_START);
	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_2, 0);
	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_3, 0);
	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_4, 0);
	pcif_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_5, 0);

	/* set as bus master and enable pci memory responses */  
	pcif_read_config_dword(0, 0, 0, PCI_COMMAND, &data);
	data |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	pcif_write_config_dword(0, 0, 0, PCI_COMMAND, data);

	/* Successful */
	return 0;
}

/* Initializes the PCIF core and driver, must be called before calling init_pci() 
 *
 * Return values
 *  0             Successful initalization
 *  -1            Error during initialization, for example "PCI core not found".
 *  -2            Error PCI controller not HOST (targets not supported)
 *  -3            Error due to PCIF hardware initialization
 *  -4            Error registering driver to PCI layer
 */
int pcif_init(struct pcif_priv *priv)
{
	struct ambapp_apb_info *apb;
	struct ambapp_ahb_info *ahb;
	int pin;
	pci_mem_config pci_mem_cfg;
	pci_config pci_drv_cfg;
	union rtems_drvmgr_key_value *value;
	char keyname[6];
	struct amba_dev_info *ainfo = priv->dev->businfo;

	/* Find PCI core from Plug&Play information */
	apb = ainfo->info.apb_slv;
	ahb = ainfo->info.ahb_slv;

	/* Found PCI core, init private structure */
	priv->irq = apb->irq;
	priv->regs = (struct pcif_regs *)apb->start;

	/* Calculate the PCI windows 
	 *  AMBA->PCI Window:                       AHB SLAVE AREA0
	 *  AMBA->PCI I/O cycles Window:            AHB SLAVE AREA1 Lower half
	 *  AMBA->PCI Configuration cycles Window:  AHB SLAVE AREA1 Upper half
	 */
	priv->pci_area     = ahb->start[0];
	priv->pci_area_end = ahb->start[0] + ahb->mask[0];
	priv->pci_io       = ahb->start[1];
	priv->pci_conf     = ahb->start[1] + (ahb->mask[1] >> 1);
	priv->pci_conf_end = ahb->start[1] + ahb->mask[1];

	/* On systems where PCI I/O area and configuration area is apart of the "PCI Window" 
	 * the PCI Window stops at the start of the PCI I/O area
	 */
	if ( (priv->pci_io > priv->pci_area) && (priv->pci_io < (priv->pci_area_end-1)) ) {
		priv->pci_area_end = priv->pci_io;
	}

	/* Init PCI interrupt assignment table to all use the interrupt routed through
	 * the PCIF core.
	 */
	strcpy(keyname, "INTX#");
	for (pin=1; pin<5; pin++) {
		if ( pcif_pci_irq_table[pin-1] == 0xff ) {
			pcif_pci_irq_table[pin-1] = priv->irq;

			/* User may override Plug & Play IRQ */
			keyname[3] = 'A' + (pin-1);
			value = rtems_drvmgr_dev_key_get(priv->dev, keyname, KEY_TYPE_INT);
			if ( value )
				pcif_pci_irq_table[pin-1] = value->i;
		}
	}

	priv->irq_mask = 0xf;
	value = rtems_drvmgr_dev_key_get(priv->dev, "", KEY_TYPE_INT);
	if ( value )
		priv->irq_mask = value->i & 0xf;

	/* This driver only support HOST systems, we check for HOST */
	if ( priv->regs->status & 0x00000001 ) {
		/* Target not supported */
		return -2;
	}

	/* Init the PCI Core */
	if ( pcif_hw_init(priv) ) {
		return -3;
	}

	/* Register the PCI core at the PCI layer */

	/* Prepare PCI driver description */
	memset(&pci_drv_cfg, 0, sizeof(pci_drv_cfg));
	pci_drv_cfg.pci_config_addr = 0;
	pci_drv_cfg.pci_config_data = 0;
	pci_drv_cfg.pci_functions = &pcif_access_functions;

	/* Prepare memory MAP */
	memset(&pci_mem_cfg, 0, sizeof(pci_mem_cfg));
	pci_mem_cfg.pci_mem_start = priv->pci_area;
	pci_mem_cfg.pci_mem_size = priv->pci_area_end - priv->pci_area;
	pci_mem_cfg.pci_io_start = priv->pci_io;
	pci_mem_cfg.pci_io_size = priv->pci_conf - priv->pci_io;

	if ( pci_register_drv(&pci_drv_cfg, &pci_mem_cfg, priv) ) {
		/* Registration failed */
		return -4;
	}

	return 0;
}

/* Called when a core is found with the AMBA device and vendor ID 
 * given in pcif_ids[].
 */
int pcif_init1(struct rtems_drvmgr_dev_info *dev)
{
	int status;
	struct pcif_priv *priv;

	DBG("PCIF[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);

	if ( pcif_minor != 0 ) {
		printf("Driver only supports one PCI core\n");
		return DRVMGR_FAIL;
	}

	priv = dev->priv;
	if ( !priv )
		return DRVMGR_NOMEM;

	dev->priv = priv;
	priv->dev = dev;
	priv->minor = pcif_minor++;

	pcifpriv = priv;
	if ( pcif_init(priv) ) {
		printf("Failed to initialize PCIF driver\n");
		free(priv);
		dev->priv = NULL;
		return DRVMGR_FAIL;
	}

	status = init_pci();
	if ( status ) {
		printf("Failed to initialize PCI sybsystem (%d)\n", status);
		return DRVMGR_FAIL;
	}

	return pcibus_register(dev);
}

int pcif_init3(struct rtems_drvmgr_dev_info *dev)
{
	struct pcif_priv *priv = dev->priv;

	/* Unmask all interrupts, on some sytems this 
	 * might be problematic because all PCI IRQs are
	 * not connected on the PCB or used for something
	 * else. The irqMask driver resource can be used to 
	 * control which PCI IRQs are used to generate the
	 * PCI system IRQ, example:
	 *
	 * 0xf - enable all  (DEFAULT)
	 * 0x8 - enable one PCI irq
	 *
	 * Before unmasking PCI IRQ, all PCI boards must
	 * have been initialized and IRQ turned off to avoid
	 * system hang.
	 */

	priv->regs->intr = priv->irq_mask;

	return DRVMGR_OK;
}
