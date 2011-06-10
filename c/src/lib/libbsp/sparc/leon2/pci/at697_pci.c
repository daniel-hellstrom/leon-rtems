/*  LEON2 AT697 PCI Host Driver.
 * 
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler AB.
 *
 *  Configures the AT697 PCI core and initialize,
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
 * Configurable parameters
 * =======================
 *  INT[A..D]#         Select system IRQ (can be tranlated into I/O interrupt)
 *  INT[A..D]#_PIO     Select PIO used to generate I/O interrupt
 *
 * Notes
 * =====
 *  IRQ must not be enabled before all PCI boards have been enabled, the
 *  IRQ is therefore enabled first in init2. The init2() for this driver
 *  is assumed to be executed earlier that all boards and their devices
 *  driver's init2() function.
 *
 */

#include <pci.h>
#include <rtems/bspIo.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/pci_bus.h>
#include <drvmgr/leon2_amba_bus.h>

#include <leon.h>

/* Configuration options */

#define SYSTEM_MAINMEM_START	0x40000000
#define SYSTEM_MAINMEM_START2	0x60000000

/* Interrupt assignment. Set to other value than 0xff in order to 
 * override defaults and plug&play information
 */
#ifndef AT697_INTA_SYSIRQ
 #define AT697_INTA_SYSIRQ 0xff
#endif
#ifndef AT697_INTB_SYSIRQ
 #define AT697_INTB_SYSIRQ 0xff
#endif
#ifndef AT697_INTC_SYSIRQ
 #define AT697_INTC_SYSIRQ 0xff
#endif
#ifndef AT697_INTD_SYSIRQ
 #define AT697_INTD_SYSIRQ 0xff
#endif

#ifndef AT697_INTA_PIO
 #define AT697_INTA_PIO 0xff
#endif
#ifndef AT697_INTB_PIO
 #define AT697_INTB_PIO 0xff
#endif
#ifndef AT697_INTC_PIO
 #define AT697_INTC_PIO 0xff
#endif
#ifndef AT697_INTD_PIO
 #define AT697_INTD_PIO 0xff
#endif


/* AT697 PCI */
#define AT697_PCI_REG_ADR 0x80000100

/* PCI Window used */
#define PCI_MEM_START 0xa0000000
#define PCI_MEM_END   0xf0000000
#define PCI_MEM_SIZE  (PCI_MEM_END - PCI_MEM_START)

/* #define DEBUG 1 */

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...) 
#endif

#define PCI_INVALID_VENDORDEVICEID	0xffffffff
#define PCI_MULTI_FUNCTION		0x80

struct at697pci_regs {
    volatile unsigned int pciid1;        /* 0x80000100 - PCI Device identification register 1         */
    volatile unsigned int pcisc;         /* 0x80000104 - PCI Status & Command                         */
    volatile unsigned int pciid2;        /* 0x80000108 - PCI Device identification register 2         */
    volatile unsigned int pcibhlc;       /* 0x8000010c - BIST, Header type, Cache line size register  */
    volatile unsigned int mbar1;         /* 0x80000110 - Memory Base Address Register 1               */
    volatile unsigned int mbar2;         /* 0x80000114 - Memory Base Address Register 2               */
    volatile unsigned int iobar3;        /* 0x80000118 - IO Base Address Register 3                   */
    volatile unsigned int dummy1[4];     /* 0x8000011c - 0x80000128                                   */ 
    volatile unsigned int pcisid;        /* 0x8000012c - Subsystem identification register            */
    volatile unsigned int dummy2;        /* 0x80000130                                                */
    volatile unsigned int pcicp;         /* 0x80000134 - PCI capabilities pointer register            */
    volatile unsigned int dummy3;        /* 0x80000138                                                */
    volatile unsigned int pcili;         /* 0x8000013c - PCI latency interrupt register               */
    volatile unsigned int pcirt;         /* 0x80000140 - PCI retry, trdy config                       */
    volatile unsigned int pcicw;         /* 0x80000144 - PCI configuration write register             */
    volatile unsigned int pcisa;         /* 0x80000148 - PCI Initiator Start Address                  */
    volatile unsigned int pciiw;         /* 0x8000014c - PCI Initiator Write Register                 */
    volatile unsigned int pcidma;        /* 0x80000150 - PCI DMA configuration register               */
    volatile unsigned int pciis;         /* 0x80000154 - PCI Initiator Status Register                */
    volatile unsigned int pciic;         /* 0x80000158 - PCI Initiator Configuration                  */
    volatile unsigned int pcitpa;        /* 0x8000015c - PCI Target Page Address Register             */   
    volatile unsigned int pcitsc;        /* 0x80000160 - PCI Target Status-Command Register           */
    volatile unsigned int pciite;        /* 0x80000164 - PCI Interrupt Enable Register                */
    volatile unsigned int pciitp;        /* 0x80000168 - PCI Interrupt Pending Register               */
    volatile unsigned int pciitf;        /* 0x8000016c - PCI Interrupt Force Register                 */
    volatile unsigned int pcid;          /* 0x80000170 - PCI Data Register                            */   
    volatile unsigned int pcibe;         /* 0x80000174 - PCI Burst End Register                       */
    volatile unsigned int pcidmaa;       /* 0x80000178 - PCI DMA Address Register                     */
};

/* PCI Interrupt assignment. Connects an PCI interrupt pin (INTA#..INTD#)
 * to a system interrupt number.
 */
unsigned char at697_pci_irq_table[4] =
{
	/* INTA# */	AT697_INTA_SYSIRQ,
	/* INTB# */	AT697_INTB_SYSIRQ,
	/* INTC# */	AT697_INTC_SYSIRQ,
	/* INTD# */	AT697_INTD_SYSIRQ
};

/* PCI Interrupt PIO assignment. Selects which GPIO pin will be used to
 * generate the system IRQ.
 *
 * PCI IRQ -> GPIO -> 4 x I/O select -> System IRQ
 *              ^- pio_table              ^- irq_select
 */
unsigned char at697_pci_irq_pio_table[4] =
{
	/* INTA# */	AT697_INTA_PIO,
	/* INTB# */	AT697_INTB_PIO,
	/* INTC# */	AT697_INTC_PIO,
	/* INTD# */	AT697_INTD_PIO
};

/* Driver private data struture */
struct at697pci_priv {
	struct rtems_drvmgr_dev_info	*dev;
	struct at697pci_regs	*regs;
	int			minor;

	unsigned int		devVend; /* PCI Device and Vendor ID of Host */
};

struct at697pci_priv *at697pcipriv = NULL;
static int at697pci_minor = 0;

int at697pci_init1(struct rtems_drvmgr_dev_info *dev);
int at697pci_init2(struct rtems_drvmgr_dev_info *dev);

/* AT697 PCI DRIVER */

struct rtems_drvmgr_drv_ops at697pci_ops = 
{
	.init = {at697pci_init1, at697pci_init2, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

struct leon2_amba_dev_id at697pci_ids[] = 
{
	{LEON2_AMBA_AT697PCI_ID},
	{0}		/* Mark end of table */
};

struct leon2_amba_drv_info at697pci_info =
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_LEON2_AMBA_AT697PCI,	/* Driver ID */
		"AT697PCI_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_LEON2_AMBA,	/* Bus Type */
		&at697pci_ops,
		0,				/* No devices yet */
	},
	&at697pci_ids[0]
};

void at697pci_register_drv(void)
{
	DBG("Registering AT697 PCI driver\n");
	rtems_drvmgr_drv_register(&at697pci_info.general);
}

/*  The configuration access functions uses the DMA functionality of the
 *  AT697 pci controller to be able access all slots
 */
 
int
at697pci_read_config_dword(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned int *val)
{
	struct at697pci_regs *regs;
	volatile unsigned int data = 0;
	unsigned int address;

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;
	
	regs = at697pcipriv->regs;

	regs->pciitp = 0xff; /* clear interrupts */ 

	if ( bus == 0 ) {
		/* PCI Access - TYPE 0 */
		address = (  1<<(11+slot) ) | ((function & 0x7)<<8) | (offset&0xfc);
	} else {
		/* PCI access - TYPE 1 */
		address = ((bus & 0xff) << 16) | ((slot & 0x1f) << 11) |
				((function & 0x7)<<8) | (offset & 0xfc) | 1;
	}
	regs->pcisa = address;
	regs->pcidma = 0xa01;
	regs->pcidmaa = (unsigned int) &data;

	while (regs->pciitp == 0)
		;

	regs->pciitp = 0xff; /* clear interrupts */ 

	if (regs->pcisc & 0x20000000)  { /* Master Abort */
		regs->pcisc |= 0x20000000;
		*val = 0xffffffff;
	}
	else
		*val = data;

	DBG("pci_read - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset,  (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), *val); 

	return PCIBIOS_SUCCESSFUL;
}


int 
at697pci_read_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short *val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	at697pci_read_config_dword(bus, slot, function, offset&~3, &v);
	*val = 0xffff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}


int 
at697pci_read_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char *val)
{
	unsigned int v;

	at697pci_read_config_dword(bus, slot, function, offset&~3, &v);

	*val = 0xff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}


int
at697pci_write_config_dword(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned int val)
{
	struct at697pci_regs *regs;
	volatile unsigned int tmp_val = val;
	unsigned int address;

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	regs = at697pcipriv->regs;

	regs->pciitp = 0xff; /* clear interrupts */

	if ( bus == 0 ) {
		/* PCI Access - TYPE 0 */
		address = (  1<<(11+slot) ) | ((function & 0x7)<<8) | (offset&0xfc);
	} else {
		/* PCI access - TYPE 1 */
		address = ((bus & 0xff) << 16) | ((slot & 0x1f) << 11) |
				((function & 0x7)<<8) | (offset & 0xfc) | 1;
	}
	regs->pcisa = address;
	regs->pcidma = 0xb01;
	regs->pcidmaa = (unsigned int) &tmp_val;

	while (regs->pciitp == 0)
		;

	if (regs->pcisc & 0x20000000)  { /* Master Abort */
		regs->pcisc |= 0x20000000;
	}

	regs->pciitp = 0xff; /* clear interrupts */

	DBG("pci_write - bus: %d, dev: %d, fn: %d, off: %d => addr: %x, val: %x\n", bus, slot, function, offset, (1<<(11+slot) ) | ((function & 7)<<8) |  (offset&0x3f), val);

	return PCIBIOS_SUCCESSFUL;
}


int 
at697pci_write_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	at697pci_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xffff << (8*(offset&3)))) | ((0xffff&val) << (8*(offset&3)));

	return at697pci_write_config_dword(bus, slot, function, offset&~3, v);
}


int 
at697pci_write_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char val)
{
	unsigned int v;

	at697pci_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xff << (8*(offset&3)))) | ((0xff&val) << (8*(offset&3)));

	return at697pci_write_config_dword(bus, slot, function, offset&~3, v);
}

/* Return the assigned system IRQ number that corresponds to the PCI "Interrupt Pin"
 * information from configuration space.
 *
 * The IRQ information is stored in the at697_pci_irq_table configurable
 * by the user.
 *
 * Returns the "system IRQ" for the PCI INTA#..INTD# pin in irq_pin. Returns
 * 0xff if not assigned.
 */
unsigned char at697_pci_get_assigned_irq(
	unsigned char bus,
	unsigned char slot,
	unsigned char func,
	unsigned char irq_pin
	)
{
	unsigned char sysIrqNr = 0xff; /* not assigned */

	if ( (irq_pin >= 1) && (irq_pin <= 4) ) {
		/* Valid PCI "Interrupt Pin" number */
		sysIrqNr = at697_pci_irq_table[irq_pin-1] & 0xff;
	}
	return sysIrqNr;
}

const pci_config_access_functions at697pci_access_functions = {
	at697pci_read_config_byte,
	at697pci_read_config_word,
	at697pci_read_config_dword,
	at697pci_write_config_byte,
	at697pci_write_config_word,
	at697pci_write_config_dword,
	at697_pci_get_assigned_irq
};

/* Initializes the AT697PCI core hardware
 *
 */
int at697pci_hw_init(struct at697pci_priv *priv)
{
	struct at697pci_regs *regs = priv->regs;
	unsigned short vendor = regs->pciid1 >> 16;

	/* Must match ATMEL or ESA ID */
	if ( !((vendor == 0x1202) || (vendor == 0x1E0F)) ) {
		/* No AT697 PCI, quit */
		return -1;
	}

	/* Reset PCI Core */
	regs->pciic = 0xffffffff;

	/* Mask PCI interrupts */
	regs->pciite = 0;

	/* Map system RAM at pci address 0x40000000 and system SDRAM to pci address 0x60000000  */
	regs->mbar1  = SYSTEM_MAINMEM_START;
	regs->mbar2  = SYSTEM_MAINMEM_START2;
	regs->pcitpa = (SYSTEM_MAINMEM_START & 0xff000000) | ((SYSTEM_MAINMEM_START2>>16) & 0xff00);

	/* Enable PCI master and target memory command response  */
	regs->pcisc |= 0x40 | 0x6;

	/* Set latency timer to 64 */
	regs->pcibhlc = 0x00004000;

	/* Set Inititator configuration so that AHB slave accesses generate memory read/write commands */
	regs->pciic = 0x41;

	/* Get the AT697PCI Host PCI ID */
	at697pci_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &priv->devVend);

	return 0;
}

/* Initializes the AT697PCI core and driver, must be called before calling init_pci() 
 *
 * Return values
 *  0             Successful initalization
 *  -1            Error during initialization.
 */
int at697pci_init(struct at697pci_priv *priv)
{
	int pin;
	pci_mem_config pci_mem_cfg;
	pci_config pci_drv_cfg;
	union rtems_drvmgr_key_value *value;
	char keyname_sysirq[6];
	char keyname_pio[10];

	/* PCI core, init private structure */
	priv->regs = (struct at697pci_regs *) AT697_PCI_REG_ADR;

	/* Init PCI interrupt assignment table to all use the interrupt routed
	 * through the GPIO core.
	 *
	 * INT[A..D]# selects system IRQ (and I/O interrupt)
	 * INT[A..D]#_PIO selects PIO used to generate I/O interrupt
	 */
	strcpy(keyname_sysirq, "INTX#");
	strcpy(keyname_pio, "INTX#_PIO");
	for (pin=1; pin<5; pin++) {
		if ( at697_pci_irq_table[pin-1] == 0xff ) {
			/* User may override hardcoded IRQ setup */
			keyname_sysirq[3] = 'A' + (pin-1);
			value = rtems_drvmgr_dev_key_get(priv->dev,
					keyname_sysirq, KEY_TYPE_INT);
			if ( value )
				at697_pci_irq_table[pin-1] = value->i;
		}
		if ( at697_pci_irq_pio_table[pin-1] == 0xff ) {
			/* User may override hardcoded IRQ setup */
			keyname_pio[3] = 'A' + (pin-1);
			value = rtems_drvmgr_dev_key_get(priv->dev,
						keyname_pio, KEY_TYPE_INT);
			if ( value )
				at697_pci_irq_pio_table[pin-1] = value->i;
		}
	}

	/* Init the PCI Core */
	if ( at697pci_hw_init(priv) ) {
		return -3;
	}

	/* Register the PCI core at the PCI layer */

	/* Prepare PCI driver description */
	memset(&pci_drv_cfg, 0, sizeof(pci_drv_cfg));
	pci_drv_cfg.pci_config_addr = 0;
	pci_drv_cfg.pci_config_data = 0;
	pci_drv_cfg.pci_functions = &at697pci_access_functions;

	/* Prepare memory MAP */
	memset(&pci_mem_cfg, 0, sizeof(pci_mem_cfg));
	pci_mem_cfg.pci_mem_start = PCI_MEM_START;
	pci_mem_cfg.pci_mem_size = PCI_MEM_SIZE;
	pci_mem_cfg.pci_io_start = 0;
	pci_mem_cfg.pci_io_size = 0;

	if ( pci_register_drv(&pci_drv_cfg, &pci_mem_cfg, priv) ) {
		/* Registration failed */
		return -4;
	}

	return 0;
}

/* Called when a core is found with the AMBA device and vendor ID 
 * given in at697pci_ids[].
 */
int at697pci_init1(struct rtems_drvmgr_dev_info *dev)
{
	int status;
	struct at697pci_priv *priv;

	DBG("AT697PCI[%d] on bus %s\n", dev->minor_drv,
		dev->parent->dev->name);

	if ( at697pci_minor != 0 ) {
		DBG("Driver only supports one PCI core\n");
		return DRVMGR_FAIL;
	}

	priv = malloc(sizeof(struct at697pci_priv));
	if ( !priv )
		return DRVMGR_NOMEM;

	memset(priv, 0, sizeof(*priv));
	dev->priv = priv;
	priv->dev = dev;
	priv->minor = at697pci_minor++;

	at697pcipriv = priv;
	if ( at697pci_init(priv) ) {
		DBG("Failed to initialize at697pci driver\n");
		free(priv);
		dev->priv = NULL;
		return DRVMGR_FAIL;
	}

	status = init_pci();
	if ( status ) {
		DBG("Failed to initialize PCI sybsystem (%d)\n", status);
		return DRVMGR_FAIL;
	}

	return pcibus_register(dev);
}

int at697pci_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct at697pci_priv *priv = dev->priv;
	int pin, irq, pio, ioport;
	LEON_Register_Map *regs = (LEON_Register_Map *)0x80000000;

	/* Enable interrupts now that init1 has been reached for all devices
	 * on the bus.
	 */

	for (pin=1; pin<5; pin++) {
		irq = at697_pci_irq_table[pin-1];
		pio = at697_pci_irq_pio_table[pin-1];
		if ( (pio < 16) && (irq >= 4) && (irq <= 7) ) {
			/* AT697 I/O IRQ, we know how to set up this 
			 *
			 * IRQ 4 -> I/O 0
			 * IRQ 5 -> I/O 1
			 * IRQ 6 -> I/O 2
			 * IRQ 7 -> I/O 3
			 */
			ioport = irq - 4;

			/* First disable interrupts */
			regs->PIO_Interrupt &= ~(0xff << (ioport * 8));
			/* Set PIO as input pin */
			regs->PIO_Direction &= ~(1 << pio);
			/* Set Low Level sensitivity */
			regs->PIO_Interrupt |= (0x87<< (ioport *8));
		}
	}

	/* Unmask Interrupt */
	/*priv->regs->pciite = 0xff;*/

	return DRVMGR_OK;
}
