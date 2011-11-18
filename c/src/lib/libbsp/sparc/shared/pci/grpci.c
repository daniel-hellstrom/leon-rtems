/*  GRLIB GRPCI PCI HOST driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the GRPCI core and initialize,
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

#include <pci.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtems/bspIo.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include <ambapp.h>
#include <drvmgr/pci_bus.h>

#define DMAPCI_ADDR 0x80000500

/* Configuration options */
#define SYSTEM_MAINMEM_START 0x40000000

/* If defined to 1 - byte twisting is enabled by default */
#define DEFAULT_BT_ENABLED 0

/* Interrupt assignment. Set to other value than 0xff in order to 
 * override defaults and plug&play information
 */
#ifndef GRPCI_INTA_SYSIRQ
 #define GRPCI_INTA_SYSIRQ 0xff
#endif
#ifndef GRPCI_INTB_SYSIRQ
 #define GRPCI_INTB_SYSIRQ 0xff
#endif
#ifndef GRPCI_INTC_SYSIRQ
 #define GRPCI_INTC_SYSIRQ 0xff
#endif
#ifndef GRPCI_INTD_SYSIRQ
 #define GRPCI_INTD_SYSIRQ 0xff
#endif

#define PAGE0_BTEN_BIT    0
#define PAGE0_BTEN        (1<<PAGE0_BTEN_BIT)

#define CFGSTAT_HOST_BIT  13
#define CFGSTAT_HOST      (1<<CFGSTAT_HOST_BIT)

/*#define DEBUG 1*/

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

#define PCI_INVALID_VENDORDEVICEID	0xffffffff
#define PCI_MULTI_FUNCTION		0x80

/*
 * Bit encode for PCI_CONFIG_HEADER_TYPE register
 */
struct grpci_regs {
	volatile unsigned int cfg_stat;
	volatile unsigned int bar0;
	volatile unsigned int page0;
	volatile unsigned int bar1;
	volatile unsigned int page1;
	volatile unsigned int iomap;
	volatile unsigned int stat_cmd;
	volatile unsigned int irq;
};

struct grpci_priv *grpcipriv = NULL;
static int grpci_minor = 0;
static unsigned int *pcidma = (unsigned int *)DMAPCI_ADDR;

/* PCI Interrupt assignment. Connects an PCI interrupt pin (INTA#..INTD#)
 * to a system interrupt number.
 */
unsigned char grpci_pci_irq_table[4] =
{
	/* INTA# */	GRPCI_INTA_SYSIRQ,
	/* INTB# */	GRPCI_INTB_SYSIRQ,
	/* INTC# */	GRPCI_INTC_SYSIRQ,
	/* INTD# */	GRPCI_INTD_SYSIRQ
};

/* Driver private data struture */
struct grpci_priv {
	struct rtems_drvmgr_dev_info	*dev;
	struct grpci_regs		*regs;
	int				irq;
	int				minor;

	int				bt_enabled;
	unsigned int			pci_area;
	unsigned int			pci_area_end;
	unsigned int			pci_io;    
	unsigned int			pci_conf;
	unsigned int			pci_conf_end;

	unsigned int			devVend; /* PCI Device and Vendor ID of Host */

	/* PCI Bus layer configuration */
	struct pcibus_config	config;
};

int grpci_init1(struct rtems_drvmgr_dev_info *dev);

/* GRPCI DRIVER */

struct rtems_drvmgr_drv_ops grpci_ops = 
{
	.init = {grpci_init1, NULL, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

struct amba_dev_id grpci_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_PCIFBRG},
	{0, 0}		/* Mark end of table */
};

struct amba_drv_info grpci_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_GRPCI_ID,	/* Driver ID */
		"GRPCI_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&grpci_ops,
		0,				/* No devices yet */
		sizeof(struct grpci_priv),	/* Make drvmgr alloc private */
	},
	&grpci_ids[0]
};

void grpci_register_drv(void)
{
	DBG("Registering GRPCI driver\n");
	rtems_drvmgr_drv_register(&grpci_info.general);
}

/*  The configuration access functions uses the DMA functionality of the
 *  AT697 pci controller to be able access all slots
 */
 
static inline unsigned int flip_dword (unsigned int l)
{
	return ((l&0xff)<<24) | (((l>>8)&0xff)<<16) | (((l>>16)&0xff)<<8)| ((l>>24)&0xff);
}

int
grpci_read_config_dword(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char offset,
  unsigned int *val
)
{
	struct grpci_priv *priv = grpcipriv;
	volatile unsigned int *pci_conf;

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	if (slot > 21) {
		*val = 0xffffffff;
		return PCIBIOS_SUCCESSFUL;
	}

	/* Select bus */
	priv->regs->cfg_stat = (priv->regs->cfg_stat & ~(0xf<<23)) | (bus<<23);

	pci_conf = (volatile unsigned int *) (priv->pci_conf +
		((slot<<11) | (function<<8) | offset));

	if ( priv->bt_enabled ) {
		*val =  flip_dword(*pci_conf);
	} else {
		*val = *pci_conf;
	}

	if (priv->regs->cfg_stat & 0x100) {
		*val = 0xffffffff;
	}

	DBG("pci_read - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset,  (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), *val); 

	return PCIBIOS_SUCCESSFUL;
}


int 
grpci_read_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short *val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	grpci_read_config_dword(bus, slot, function, offset&~3, &v);
	*val = 0xffff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}


int 
grpci_read_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char *val)
{
	unsigned int v;

	grpci_read_config_dword(bus, slot, function, offset&~3, &v);

	*val = 0xff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}

int
grpci_write_config_dword(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned int val)
{
	struct grpci_priv *priv = grpcipriv;
	volatile unsigned int *pci_conf;
	unsigned int value;

	if (offset & 3 ) return PCIBIOS_BAD_REGISTER_NUMBER;

	/* Select bus */
	priv->regs->cfg_stat = (priv->regs->cfg_stat & ~(0xf<<23)) | (bus<<23);

	pci_conf = (volatile unsigned int *) (priv->pci_conf +
		((slot<<11) | (function<<8) | (offset & ~3)));

	if ( priv->bt_enabled ) {
		value = flip_dword(val);
	} else {
		value = val;
	}

	*pci_conf = value;

	DBG("pci write - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset, (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), value); 

	return PCIBIOS_SUCCESSFUL;
}


int 
grpci_write_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	grpci_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xffff << (8*(offset&3)))) | ((0xffff&val) << (8*(offset&3)));

	return grpci_write_config_dword(bus, slot, function, offset&~3, v);
}


int 
grpci_write_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char val) {
	unsigned int v;

	grpci_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xff << (8*(offset&3)))) | ((0xff&val) << (8*(offset&3)));

	return grpci_write_config_dword(bus, slot, function, offset&~3, v);
}

/* Return the assigned system IRQ number that corresponds to the PCI "Interrupt Pin"
 * information from configuration space.
 *
 * The IRQ information is stored in the grpci_pci_irq_table configurable
 * by the user.
 *
 * Returns the "system IRQ" for the PCI INTA#..INTD# pin in irq_pin. Returns
 * 0xff if not assigned.
 */
unsigned char grpci_get_assigned_irq(
	unsigned char bus,
	unsigned char slot,
	unsigned char func,
	unsigned char irq_pin
	)
{
	unsigned char sysIrqNr = 0xff; /* not assigned */

	if ( (irq_pin >= 1) && (irq_pin <= 4) ) {
		/* Valid PCI "Interrupt Pin" number */
		sysIrqNr = grpci_pci_irq_table[irq_pin-1];
	}
	return sysIrqNr;
}

const pci_config_access_functions grpci_access_functions = {
	grpci_read_config_byte,
	grpci_read_config_word,
	grpci_read_config_dword,
	grpci_write_config_byte,
	grpci_write_config_word,
	grpci_write_config_dword,
	grpci_get_assigned_irq
};

int grpci_hw_init(struct grpci_priv *priv)
{
	volatile unsigned int *mbar0, *page0;
	unsigned int data, addr, mbar0size;

	mbar0 = (volatile unsigned int *)priv->pci_area;

	if ( !priv->bt_enabled && ((priv->regs->page0 & PAGE0_BTEN) == PAGE0_BTEN) ) {
		/* Byte twisting is on, turn it off */
		grpci_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_0, 0xffffffff);
		grpci_read_config_dword(0, 0, 0, PCI_BASE_ADDRESS_0, &addr);
		grpci_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_0, flip_dword(0x80000000));    /* Setup bar0 to nonzero value (grpci considers BAR==0 as invalid) */
		addr = (~flip_dword(addr)+1)>>1;                               /* page0 is accessed through upper half of bar0 */
		mbar0size = addr*2;
		DBG("GRPCI: Size of MBAR0: 0x%x, MBAR0: 0x%x(lower) 0x%x(upper)\n",mbar0size,((unsigned int)mbar0),((unsigned int)mbar0)+mbar0size/2);
		page0 = &mbar0[mbar0size/8];
		DBG("GRPCI: PAGE0 reg address: 0x%x (0x%x)\n",((unsigned int)mbar0)+mbar0size/2,page0);
		priv->regs->cfg_stat = (priv->regs->cfg_stat & (~0xf0000000)) | 0x80000000;    /* Setup mmap reg so we can reach bar0 */ 
		*page0 = 0<<PAGE0_BTEN_BIT;                                         /* Disable bytetwisting ... */
	}

	/* Get the GRPCI Host PCI ID */
	grpci_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &priv->devVend);

	/* set 1:1 mapping between AHB -> PCI memory */
	priv->regs->cfg_stat = (priv->regs->cfg_stat & 0x0fffffff) | priv->pci_area;
	
	/* and map system RAM at pci address 0x40000000 */ 
	grpci_write_config_dword(0, 0, 0, PCI_BASE_ADDRESS_1, SYSTEM_MAINMEM_START);
	priv->regs->page1 = SYSTEM_MAINMEM_START;

	/* Translate I/O accesses 1:1 */
	priv->regs->iomap = priv->pci_io & 0xffff0000;

	/* set as bus master and enable pci memory responses */  
	grpci_read_config_dword(0, 0, 0, PCI_COMMAND, &data);
	data |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	grpci_write_config_dword(0, 0, 0, PCI_COMMAND, data);

	/* unmask all PCI interrupts at PCI Core, not all GRPCI cores support this */
	priv->regs->irq = 0xf0000;

	/* Successful */
	return 0;
}

/* Initializes the GRPCI core and driver, must be called before calling init_pci() 
 *
 * Return values
 *  0             Successful initalization
 *  -1            Error during initialization, for example "PCI core not found".
 *  -2            Error PCI controller not HOST (targets not supported)
 *  -3            Error due to GRPCI hardware initialization
 *  -4            Error registering driver to PCI layer
 */
int grpci_init(struct grpci_priv *priv)
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
	priv->regs = (struct grpci_regs *)apb->start;
	priv->bt_enabled = DEFAULT_BT_ENABLED;

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
	 * the GRPCI core.
	 */
	strcpy(keyname, "INTX#");
	for (pin=1; pin<5; pin++) {
		if ( grpci_pci_irq_table[pin-1] == 0xff ) {
			grpci_pci_irq_table[pin-1] = priv->irq;

			/* User may override Both hardcoded IRQ setup and Plug & Play IRQ */
			keyname[3] = 'A' + (pin-1);
			value = rtems_drvmgr_dev_key_get(priv->dev, keyname, KEY_TYPE_INT);
			if ( value )
				grpci_pci_irq_table[pin-1] = value->i;
		}
	}

	/* User may override DEFAULT_BT_ENABLED to enable/disable byte twisting */
	value = rtems_drvmgr_dev_key_get(priv->dev, "byteTwisting", KEY_TYPE_INT);
	if ( value )
		priv->bt_enabled = value->i;

	/* This driver only support HOST systems, we check for HOST */
	if ( !(priv->regs->cfg_stat & CFGSTAT_HOST) ) {
		/* Target not supported */
		return -2;
	}

	/* Init the PCI Core */
	if ( grpci_hw_init(priv) ) {
		return -3;
	}

	/* Register the PCI core at the PCI layer */

	/* Prepare PCI driver description */
	memset(&pci_drv_cfg, 0, sizeof(pci_drv_cfg));
	pci_drv_cfg.pci_config_addr = 0;
	pci_drv_cfg.pci_config_data = 0;
	pci_drv_cfg.pci_functions = &grpci_access_functions;

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
 * given in grpci_ids[]. IRQ, Console does not work here
 */
int grpci_init1(struct rtems_drvmgr_dev_info *dev)
{
	int status;
	struct grpci_priv *priv;

	DBG("GRPCI[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);

	if ( grpci_minor != 0 ) {
		DBG("Driver only supports one PCI core\n");
		return DRVMGR_FAIL;
	}

	if ( (strcmp(dev->parent->dev->drv->name, "AMBAPP_GRLIB_DRV") != 0) && 
	     (strcmp(dev->parent->dev->drv->name, "AMBAPP_LEON2_DRV") != 0) ) {
		/* We only support GRPCI driver on local bus */
		return DRVMGR_FAIL;
	}

	priv = dev->priv;
	if ( !priv )
		return DRVMGR_NOMEM;

	dev->priv = priv;
	priv->dev = dev;
	priv->minor = grpci_minor++;

	grpcipriv = priv;
	status = grpci_init(priv);
	if ( status ) {
		printf("Failed to initialize grpci driver %d\n", status);

		return -1;
	}

	status = init_pci();
	if ( status ) {
		printf("Failed to initialize GRPCI core (%d)\n", status);
		return DRVMGR_FAIL;
	}

	return pcibus_register(dev, &priv->config);
}

/* DMA functions which uses GRPCIs optional DMA controller (len in words) */
int grpci_dma_to_pci(unsigned int ahb_addr, unsigned int pci_addr, unsigned int len) {
    int ret = 0;

    pcidma[0] = 0x82;
    pcidma[1] = ahb_addr;
    pcidma[2] = pci_addr;
    pcidma[3] = len;
    pcidma[0] = 0x83;        

    while ( (pcidma[0] & 0x4) == 0)
        ;

    if (pcidma[0] & 0x8) { /* error */ 
        ret = -1;
    }

    pcidma[0] |= 0xC; 
    return ret;

}

int grpci_dma_from_pci(unsigned int ahb_addr, unsigned int pci_addr, unsigned int len) {
    int ret = 0;

    pcidma[0] = 0x80;
    pcidma[1] = ahb_addr;
    pcidma[2] = pci_addr;
    pcidma[3] = len;
    pcidma[0] = 0x81;        

    while ( (pcidma[0] & 0x4) == 0)
        ;

    if (pcidma[0] & 0x8) { /* error */ 
        ret = -1;
    }

    pcidma[0] |= 0xC; 
    return ret;

}
