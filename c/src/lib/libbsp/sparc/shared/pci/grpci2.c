/*  GRLIB GRPCI22 PCI HOST driver.
 * 
 *  COPYRIGHT (c) 2011
 *  Aeroflex Gaisler AB.
 *
 *  Configures the GRPCI2 core and initialize,
 *   - the PCI Library (pci.c)
 *   - the general part of the PCI Bus driver (pci_bus.c)
 *  
 *  System interrupt assigned to PCI interrupt (INTA#..INTD#) is by
 *  default taken from Plug and Play, but may be overridden by the 
 *  driver resources INTA#..INTD#. GRPCI2 handles differently depending
 *  on the design (4 different ways).
 *
 *  The license and distribution terms for this file may be
 *  found in found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2011-02-04, Daniel Hellstrom <daniel@gaisler.com>
 *   Created
 *
 *
 * GRPCI2 IRQ implementation notes
 * -------------------------------
 * Since the Driver Manager pci_bus layer implements IRQ by calling
 * pci_interrupt_* which translates into BSP_shared_interrupt_*, and the
 * root-bus also relies on BSP_shared_interrupt_*, it is safe for the GRPCI2
 * driver to use the rtems_drvmgr_interrupt_* routines since they will be
 * accessing the same routines in the end. Otherwise the GRPCI2 driver must
 * have used the pci_interrupt_* routines.
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
#include <grpci2.h>

#ifndef IRQ_GLOBAL_PREPARE
 #define IRQ_GLOBAL_PREPARE(level) rtems_interrupt_level level
#endif

#ifndef IRQ_GLOBAL_DISABLE
 #define IRQ_GLOBAL_DISABLE(level) rtems_interrupt_disable(level)
#endif

#ifndef IRQ_GLOBAL_ENABLE
 #define IRQ_GLOBAL_ENABLE(level) rtems_interrupt_enable(level)
#endif

/* If defined to 1 - byte twisting is enabled by default */
#define DEFAULT_BT_ENABLED 0

/* Interrupt assignment. Set to other value than 0xff in order to 
 * override defaults and plug&play information
 */
#ifndef GRPCI2_INTA_SYSIRQ
 #define GRPCI2_INTA_SYSIRQ 0xff
#endif
#ifndef GRPCI2_INTB_SYSIRQ
 #define GRPCI2_INTB_SYSIRQ 0xff
#endif
#ifndef GRPCI2_INTC_SYSIRQ
 #define GRPCI2_INTC_SYSIRQ 0xff
#endif
#ifndef GRPCI2_INTD_SYSIRQ
 #define GRPCI2_INTD_SYSIRQ 0xff
#endif

/*#define DEBUG 1*/

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

#define PCI_INVALID_VENDORDEVICEID	0xffffffff
#define PCI_MULTI_FUNCTION		0x80

/*
 * GRPCI2 APB Register MAP
 */
struct grpci2_regs {
	volatile unsigned int ctrl;		/* 0x00 */
	volatile unsigned int sts_cap;		/* 0x04 */
	int res1;				/* 0x08 */
	volatile unsigned int io_map;		/* 0x0C */
	volatile unsigned int dma_ctrl;		/* 0x10 */
	volatile unsigned int dma_bdbase;	/* 0x14 */
	int res2[2];				/* 0x18 */
	volatile unsigned int bars[6];		/* 0x20 */
	int res3[2];				/* 0x38 */
	volatile unsigned int ahbmst_map[16];	/* 0x40 */
};

#define CTRL_BUS_BIT 16

#define CTRL_SI (1<<27)
#define CTRL_PE (1<<26)
#define CTRL_EI (1<<25)
#define CTRL_ER (1<<24)
#define CTRL_BUS (0xff<<CTRL_BUS_BIT)
#define CTRL_HOSTINT 0xf

#define STS_HOST_BIT	31
#define STS_MST_BIT	30
#define STS_TAR_BIT	29
#define STS_DMA_BIT	28
#define STS_DI_BIT	27
#define STS_HI_BIT	26
#define STS_IRQMODE_BIT	24
#define STS_TRACE_BIT	23
#define STS_CFGERRVALID_BIT 20
#define STS_CFGERR_BIT	19
#define STS_INTTYPE_BIT	12
#define STS_INTSTS_BIT	8
#define STS_FDEPTH_BIT	2
#define STS_FNUM_BIT	0

#define STS_HOST	(1<<STS_HOST_BIT)
#define STS_MST		(1<<STS_MST_BIT)
#define STS_TAR		(1<<STS_TAR_BIT)
#define STS_DMA		(1<<STS_DMA_BIT)
#define STS_DI		(1<<STS_DI_BIT)
#define STS_HI		(1<<STS_HI_BIT)
#define STS_IRQMODE	(0x3<<STS_IRQMODE_BIT)
#define STS_TRACE	(1<<STS_TRACE_BIT)
#define STS_CFGERRVALID	(1<<STS_CFGERRVALID_BIT)
#define STS_CFGERR	(1<<STS_CFGERR_BIT)
#define STS_INTTYPE	(0x3f<<STS_INTTYPE_BIT)
#define STS_INTSTS	(0xf<<STS_INTSTS_BIT)
#define STS_FDEPTH	(0x7<<STS_FDEPTH_BIT)
#define STS_FNUM	(0x3<<STS_FNUM_BIT)

#define STS_ISYSERR	(1<<17)
#define STS_IDMA	(1<<16)
#define STS_IDMAERR	(1<<15)
#define STS_IMSTABRT	(1<<14)
#define STS_ITGTABRT	(1<<13)
#define STS_IPARERR	(1<<12)

struct grpci2_bd_chan {
	volatile unsigned int ctrl;	/* 0x00 DMA Control */
	volatile unsigned int nchan;	/* 0x04 Next DMA Channel Address */
	volatile unsigned int nbd;	/* 0x08 Next Data Descriptor in channel */
	volatile unsigned int res;	/* 0x0C Reserved */
};

#define BD_CHAN_EN		0x80000000
#define BD_CHAN_TYPE		0x00300000
#define BD_CHAN_BDCNT		0x0000ffff
#define BD_CHAN_EN_BIT		31
#define BD_CHAN_TYPE_BIT	20
#define BD_CHAN_BDCNT_BIT	0

struct grpci2_bd_data {
	volatile unsigned int ctrl;	/* 0x00 DMA Data Control */
	volatile unsigned int pci_adr;	/* 0x04 PCI Start Address */
	volatile unsigned int ahb_adr;	/* 0x08 AHB Start address */
	volatile unsigned int next;	/* 0x0C Next Data Descriptor in channel */
};

#define BD_DATA_EN		0x80000000
#define BD_DATA_IE		0x40000000
#define BD_DATA_DR		0x20000000
#define BD_DATA_TYPE		0x00300000
#define BD_DATA_ER		0x00080000
#define BD_DATA_LEN		0x0000ffff
#define BD_DATA_EN_BIT		31
#define BD_DATA_IE_BIT		30
#define BD_DATA_DR_BIT		29
#define BD_DATA_TYPE_BIT	20
#define BD_DATA_ER_BIT		19
#define BD_DATA_LEN_BIT		0

/* GRPCI2 Capability */
struct grpci2_cap_first {
	unsigned int ctrl;
	unsigned int pci2ahb_map[6];
	unsigned int ext2ahb_map;
	unsigned int io_map;
	unsigned int pcibar_size[6];
};
#define CAP9_CTRL_OFS 0
#define CAP9_BAR_OFS 0x4
#define CAP9_IOMAP_OFS 0x20
#define CAP9_BARSIZE_OFS 0x24

struct grpci2_priv *grpci2priv = NULL;

/* PCI Interrupt assignment. Connects an PCI interrupt pin (INTA#..INTD#)
 * to a system interrupt number.
 */
unsigned char grpci2_pci_irq_table[4] =
{
	/* INTA# */	GRPCI2_INTA_SYSIRQ,
	/* INTB# */	GRPCI2_INTB_SYSIRQ,
	/* INTC# */	GRPCI2_INTC_SYSIRQ,
	/* INTD# */	GRPCI2_INTD_SYSIRQ
};

/* Driver private data struture */
struct grpci2_priv {
	struct rtems_drvmgr_dev_info	*dev;
	struct grpci2_regs		*regs;
	char				irq;
	char				irq_mode; /* IRQ Mode from CAPSTS REG */
	char				bt_enabled;
	unsigned int			irq_mask;

	unsigned int			pci_area;
	unsigned int			pci_area_end;
	unsigned int			pci_io;    
	unsigned int			pci_conf;
	unsigned int			pci_conf_end;

	unsigned int			devVend; /* PCI Device and Vendor ID of Host */

	/* PCI Bus layer configuration */
	struct pcibus_config	config;
};

int grpci2_init1(struct rtems_drvmgr_dev_info *dev);
int grpci2_init3(struct rtems_drvmgr_dev_info *dev);

/* GRPCI2 DRIVER */

struct rtems_drvmgr_drv_ops grpci2_ops = 
{
	.init = {grpci2_init1, NULL, grpci2_init3, NULL},
	.remove = NULL,
	.info = NULL
};

struct amba_dev_id grpci2_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_GRPCI2},
	{0, 0}		/* Mark end of table */
};

struct amba_drv_info grpci2_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_GRPCI2_ID,/* Driver ID */
		"GRPCI2_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&grpci2_ops,
		0,				/* No devices yet */
		sizeof(struct grpci2_priv),	/* Make drvmgr alloc private */
	},
	&grpci2_ids[0]
};

void grpci2_register_drv(void)
{
	DBG("Registering GRPCI2 driver\n");
	rtems_drvmgr_drv_register(&grpci2_info.general);
}

/*  The configuration access functions uses the DMA functionality of the
 *  AT697 pci controller to be able access all slots
 */
 
static inline unsigned int flip_dword (unsigned int l)
{
	return ((l&0xff)<<24) | (((l>>8)&0xff)<<16) | (((l>>16)&0xff)<<8)| ((l>>24)&0xff);
}

int
grpci2_read_config_dword(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char offset,
  unsigned int *val
)
{
	struct grpci2_priv *priv = grpci2priv;
	volatile unsigned int *pci_conf;
	unsigned int tmp, devfn;
	IRQ_GLOBAL_PREPARE(oldLevel);

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	if (slot > 15) {
		*val = 0xffffffff;
		return PCIBIOS_SUCCESSFUL;
	}

	/* GRPCI2 can access "non-standard" devices on bus0 (on AD11.AD16), 
	 * we skip them.
	 */
	if (bus == 0 && slot != 0)
		devfn = ((slot + 6) << 3) | (function & 0x7);
	else
		devfn = (slot << 3) | (function & 0x7);

	IRQ_GLOBAL_DISABLE(oldLevel); /* protect regs */

	/* Select bus */
	priv->regs->ctrl = (priv->regs->ctrl & ~(0xff<<16)) | (bus<<16);
	/* clear old status */
	priv->regs->sts_cap = (STS_CFGERR | STS_CFGERRVALID);

	pci_conf = (volatile unsigned int *) (priv->pci_conf |
					(devfn << 8) | (offset & 0xfc));

	tmp = *pci_conf;

	/* Wait until GRPCI2 signals that CFG access is done, it should be
	 * done instantaneously unless a DMA operation is ongoing...
	 */
	while ((priv->regs->sts_cap & STS_CFGERRVALID) == 0)
		;

	if (priv->regs->sts_cap & STS_CFGERR) {
		*val = 0xffffffff;
	} else {
		/* Bus always little endian (unaffected by byte-swapping) */
		*val = flip_dword(tmp);
	}

	IRQ_GLOBAL_ENABLE(oldLevel);

	DBG("grpci2_read  - bus: %d, dev: %02d, fn: %02d, off: %03d => addr: %p, val: %x\n",
		bus, slot, function, offset, pci_conf, *val);

	return PCIBIOS_SUCCESSFUL;
}


int 
grpci2_read_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short *val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	grpci2_read_config_dword(bus, slot, function, offset&~3, &v);
	*val = 0xffff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}


int 
grpci2_read_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char *val)
{
	unsigned int v;

	grpci2_read_config_dword(bus, slot, function, offset&~3, &v);

	*val = 0xff & (v >> (8*(offset & 3)));

	return PCIBIOS_SUCCESSFUL;
}

int
grpci2_write_config_dword(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned int val)
{
	struct grpci2_priv *priv = grpci2priv;
	volatile unsigned int *pci_conf;
	unsigned int value, devfn;
	IRQ_GLOBAL_PREPARE(oldLevel);

	if (offset & 3) return PCIBIOS_BAD_REGISTER_NUMBER;

	if (slot > 15)
		return PCIBIOS_SUCCESSFUL;

	value = flip_dword(val);

	/* GRPCI2 can access "non-standard" devices on bus0 (on AD11.AD16), 
	 * we skip them.
	 */
	if (bus == 0 && slot != 0)
		devfn = ((slot + 6) << 3) | (function & 0x7);
	else
		devfn = (slot << 3) | (function & 0x7);

	IRQ_GLOBAL_DISABLE(oldLevel); /* protect regs */

	/* Select bus */
	priv->regs->ctrl = (priv->regs->ctrl & ~(0xff<<16)) | (bus<<16);
	/* clear old status */
	priv->regs->sts_cap = (STS_CFGERR | STS_CFGERRVALID);

	pci_conf = (volatile unsigned int *) (priv->pci_conf |
				(devfn << 8) | (offset & 0xfc));

	*pci_conf = value;

	/* Wait until GRPCI2 signals that CFG access is done, it should be
	 * done instantaneously unless a DMA operation is ongoing...
	 */
	while ((priv->regs->sts_cap & STS_CFGERRVALID) == 0)
		;

#if 0
	if (priv->regs->sts_cap & STS_CFGERR) {
		/* return error ? */
	}
#endif

	IRQ_GLOBAL_ENABLE(oldLevel);

	DBG("grpci2_write - bus: %d, dev: %02d, fn: %02d, off: %03d => addr: %p, val: %x\n",
		bus, slot, function, offset, pci_conf, value);

	return PCIBIOS_SUCCESSFUL;
}

int 
grpci2_write_config_word(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned short val)
{
	unsigned int v;

	if (offset & 1) return PCIBIOS_BAD_REGISTER_NUMBER;

	grpci2_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xffff << (8*(offset&3)))) | ((0xffff&val) << (8*(offset&3)));

	return grpci2_write_config_dword(bus, slot, function, offset&~3, v);
}


int 
grpci2_write_config_byte(unsigned char bus, unsigned char slot, unsigned char function, unsigned char offset, unsigned char val) {
	unsigned int v;

	grpci2_read_config_dword(bus, slot, function, offset&~3, &v);

	v = (v & ~(0xff << (8*(offset&3)))) | ((0xff&val) << (8*(offset&3)));

	return grpci2_write_config_dword(bus, slot, function, offset&~3, v);
}

/* Return the assigned system IRQ number that corresponds to the PCI "Interrupt Pin"
 * information from configuration space.
 *
 * The IRQ information is stored in the grpci2_pci_irq_table configurable
 * by the user.
 *
 * Returns the "system IRQ" for the PCI INTA#..INTD# pin in irq_pin. Returns
 * 0xff if not assigned.
 */
unsigned char grpci2_get_assigned_irq(
	unsigned char bus,
	unsigned char slot,
	unsigned char func,
	unsigned char irq_pin
	)
{
	unsigned char sysIrqNr = 0xff; /* not assigned */
	int irq_group;

	if ( (irq_pin >= 1) && (irq_pin <= 4) ) {
		/* Use default IRQ decoding on PCI BUS0 according slot numbering */
		irq_group = slot & 0x3;
		irq_pin = ((irq_pin - 1) + irq_group) & 0x3;
		/* Valid PCI "Interrupt Pin" number */
		sysIrqNr = grpci2_pci_irq_table[irq_pin];
	}
	return sysIrqNr;
}

const pci_config_access_functions grpci2_access_functions = {
	grpci2_read_config_byte,
	grpci2_read_config_word,
	grpci2_read_config_dword,
	grpci2_write_config_byte,
	grpci2_write_config_word,
	grpci2_write_config_dword,
	grpci2_get_assigned_irq
};

/* PCI Error Interrupt handler, called when there may be a PCI Target/Master
 * Abort.
 */
void grpci2_err_isr(int irqno, struct grpci2_priv *priv)
{
	unsigned int sts = priv->regs->sts_cap;

	if (sts & (STS_IMSTABRT | STS_ITGTABRT | STS_IPARERR | STS_ISYSERR)) {
		/* A PCI error IRQ ... Error handler unimplemented
		 * add your code here...
		 */
		if (sts & STS_IMSTABRT) {
			printk("GRPCI2: unhandled Master Abort IRQ\n");
		}
		if (sts & STS_ITGTABRT) {
			printk("GRPCI2: unhandled Target Abort IRQ\n");
		}
		if (sts & STS_IPARERR) {
			printk("GRPCI2: unhandled Parity Error IRQ\n");
		}
		if (sts & STS_ISYSERR) {
			printk("GRPCI2: unhandled System Error IRQ\n");
		}
	}
}

int grpci2_hw_init(struct grpci2_priv *priv, struct grpci2_pcibar_cfg *barcfg)
{
	unsigned int ahbadr, pciadr, capptr, io_map, data, bar_sz;
	struct grpci2_regs *regs = priv->regs;
	int i;

	/* Reset any earlier setup */
	regs->ctrl = 0;
	regs->sts_cap = ~0; /* Clear Status */
	regs->dma_ctrl = 0;
	regs->dma_bdbase = 0;

	/* Translate I/O accesses 1:1, (will not work for PCI 2.3) */
	regs->io_map = priv->pci_io & 0xffff0000;

	/* set 1:1 mapping between AHB -> PCI memory space, for all Masters
	 * Each AHB master has it's own mapping registers. Max 16 AHB masters.
	 */
	for (i=0; i<16; i++)
		regs->ahbmst_map[i] = priv->pci_area;

	/* Get the GRPCI2 Host PCI ID */
	grpci2_read_config_dword(0, 0, 0, PCI_VENDOR_ID, &priv->devVend);

	/* Get address to first (always defined) capability structure */
	grpci2_read_config_byte(0, 0, 0, PCI_CAP_PTR, &capptr);
	if (capptr == 0)
		return -1;

	/* Enable/Disable Byte twisting */
	grpci2_read_config_dword(0, 0, 0, capptr+CAP9_IOMAP_OFS, &io_map);
	io_map = (io_map & ~0x1) | (priv->bt_enabled ? 1 : 0);
	grpci2_write_config_dword(0, 0, 0, capptr+CAP9_IOMAP_OFS, io_map);

	/* Setup the Host's PCI Target BARs for others to access.
	 *
	 * MAP BAR0 to Main memory for targets to access host's memory on DMA
	 * trasactions, PCI <-> AHB address is not translated (1:1). The other
	 * BARs are disabled by default.
	 *
	 * The user may override the configuration with "tgtBarCfg" resource,
	 * BARs that are defined with PCI address 0xffffffff are ignored
	 * (default used).
	 */
	for (i=0; i<6; i++) {
		if (i == 0) {
			pciadr = (unsigned int)grpci2_hw_init & 0xf0000000;
			ahbadr = pciadr;
			bar_sz = 0x10000000; /* Guess 256Mb */
		} else {
			/* Default is to disable BAR */
			pciadr = 0;
			ahbadr = 0;
			bar_sz = 0;
		}
		/* Let user Override default configuration, alignment must
		 * be correct
		 */
		if (barcfg && (barcfg[i].pciadr != 0xffffffff)) {
			pciadr = barcfg[i].pciadr;
			ahbadr = barcfg[i].ahbadr;
			bar_sz = ((pciadr - 1) & ~pciadr) + 1;
		}
		grpci2_write_config_dword(0,0,0,capptr+CAP9_BARSIZE_OFS+i*4,
									bar_sz);
		grpci2_write_config_dword(0,0,0,capptr+CAP9_BAR_OFS+i*4,ahbadr);
		grpci2_write_config_dword(0,0,0,PCI_BASE_ADDRESS_0+i*4,pciadr);
	}

	/* set as bus master and enable pci memory responses */  
	grpci2_read_config_dword(0, 0, 0, PCI_COMMAND, &data);
	data |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	grpci2_write_config_dword(0, 0, 0, PCI_COMMAND, data);

	/* Enable Error respone (CPU-TRAP) on illegal memory access */
	regs->ctrl = CTRL_ER | CTRL_PE;

	/* Successful */
	return 0;
}

/* Initializes the GRPCI2 core and driver, must be called before calling
 * init_pci()
 *
 * Return values
 *  0             Successful initalization
 *  -1            Error during initialization, for example "PCI core not found".
 *  -2            Error PCI controller not HOST (targets not supported)
 *  -3            Error due to GRPCI2 hardware initialization
 */
int grpci2_init(struct grpci2_priv *priv)
{
	struct ambapp_apb_info *apb;
	struct ambapp_ahb_info *ahb;
	int pin, erridx;
	union rtems_drvmgr_key_value *value;
	char keyname[6];
	struct amba_dev_info *ainfo = priv->dev->businfo;
	struct grpci2_pcibar_cfg *barcfg;

	/* Find PCI core from Plug&Play information */
	apb = ainfo->info.apb_slv;
	ahb = ainfo->info.ahb_slv;

	/* Found PCI core, init private structure */
	priv->irq = apb->irq;
	priv->regs = (struct grpci2_regs *)apb->start;
	priv->bt_enabled = DEFAULT_BT_ENABLED;
	priv->irq_mode = (priv->regs->sts_cap & STS_IRQMODE) >> STS_IRQMODE_BIT;

	/* Calculate the PCI windows 
	 *  AMBA->PCI Window:                       AHB SLAVE AREA0
	 *  AMBA->PCI I/O cycles Window:            AHB SLAVE AREA1 Lower half
	 *  AMBA->PCI Configuration cycles Window:  AHB SLAVE AREA1 Upper half
	 */
	priv->pci_area     = ahb->start[0];
	priv->pci_area_end = ahb->start[0] + ahb->mask[0];
	priv->pci_io       = ahb->start[1];
	priv->pci_conf     = ahb->start[1] + 0x10000;
	priv->pci_conf_end = priv->pci_conf + 0x10000;

	/* On systems where PCI I/O area and configuration area is apart of the
	 * "PCI Window" the PCI Window stops at the start of the PCI I/O area
	 */
	if ((priv->pci_io > priv->pci_area) &&
	    (priv->pci_io < (priv->pci_area_end-1))) {
		priv->pci_area_end = priv->pci_io;
	}

	/* Init PCI interrupt assignment table to all use the interrupt routed
	 * through the GRPCI2 core.
	 */
	strcpy(keyname, "INTX#");
	for (pin=1; pin<5; pin++) {
		if (grpci2_pci_irq_table[pin-1] == 0xff) {
			if (priv->irq_mode < 2) {
				/* PCI Interrupts are shared */
				grpci2_pci_irq_table[pin-1] = priv->irq;
			} else {
				/* Unique IRQ per PCI INT Pin */
				grpci2_pci_irq_table[pin-1] = priv->irq + pin-1;
			}

			/* User may override Both hardcoded IRQ setup and Plug & Play IRQ */
			keyname[3] = 'A' + (pin-1);
			value = rtems_drvmgr_dev_key_get(priv->dev, keyname, KEY_TYPE_INT);
			if (value)
				grpci2_pci_irq_table[pin-1] = value->i;
		}

		/* Remember which IRQs are enabled */
		if (grpci2_pci_irq_table[pin-1] != 0)
			priv->irq_mask |= 1 << (pin-1);
	}

	/* User may override DEFAULT_BT_ENABLED to enable/disable byte twisting */
	value = rtems_drvmgr_dev_key_get(priv->dev, "byteTwisting", KEY_TYPE_INT);
	if (value)
		priv->bt_enabled = value->i;

	/* Let user Configure the 6 target BARs */
	value = rtems_drvmgr_dev_key_get(priv->dev, "tgtBarCfg", KEY_TYPE_POINTER);
	if (value)
		barcfg = value->ptr;
	else
		barcfg = NULL;

	/* This driver only support HOST systems, we check that it can act as a 
	 * PCI Master and that it is in the Host slot. */
	if ((priv->regs->sts_cap&STS_HOST) || !(priv->regs->sts_cap&STS_MST))
		return -2; /* Target not supported */

	/* Init the PCI Core */
	if (grpci2_hw_init(priv, barcfg))
		return -3;

	/* Install PCI Error interrupt handler */
	rtems_drvmgr_interrupt_register(priv->dev, 0,
					grpci2_err_isr, (void *)priv);

	return 0;
}

/* Called when a core is found with the AMBA device and vendor ID 
 * given in grpci2_ids[]. IRQ, Console does not work here
 */
int grpci2_init1(struct rtems_drvmgr_dev_info *dev)
{
	int status;
	struct grpci2_priv *priv;
	pci_mem_config pci_mem_cfg;
	pci_config pci_drv_cfg;

	DBG("GRPCI2[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);

	if (grpci2priv) {
		DBG("Driver only supports one PCI core\n");
		return DRVMGR_FAIL;
	}

	if ((strcmp(dev->parent->dev->drv->name, "AMBAPP_GRLIB_DRV") != 0) &&
	    (strcmp(dev->parent->dev->drv->name, "AMBAPP_LEON2_DRV") != 0)) {
		/* We only support GRPCI2 driver on local bus */
		return DRVMGR_FAIL;
	}

	priv = dev->priv;
	if (!priv)
		return DRVMGR_NOMEM;

	priv->dev = dev;
	grpci2priv = priv;

	/* Initialize GRPCI2 Hardware */
	status = grpci2_init(priv);
	if (status) {
		printf("Failed to initialize grpci2 driver %d\n", status);
		return -1;
	}

	/* Prepare PCI driver description */
	memset(&pci_drv_cfg, 0, sizeof(pci_drv_cfg));
	pci_drv_cfg.pci_config_addr = 0;
	pci_drv_cfg.pci_config_data = 0;
	pci_drv_cfg.pci_functions = &grpci2_access_functions;

	/* Prepare memory MAP */
	memset(&pci_mem_cfg, 0, sizeof(pci_mem_cfg));
	pci_mem_cfg.pci_mem_start = priv->pci_area;
	pci_mem_cfg.pci_mem_size = priv->pci_area_end - priv->pci_area;
	pci_mem_cfg.pci_io_start = priv->pci_io;
	pci_mem_cfg.pci_io_size = priv->pci_conf - priv->pci_io;

	/* Register the PCI core at the PCI layer */
	if (pci_register_drv(&pci_drv_cfg, &pci_mem_cfg, priv)) {
		/* Registration failed */
		return DRVMGR_FAIL;
	}

	/* Initialize PCI configuration */
	status = init_pci();
	if (status) {
		printf("Failed to initialize GRPCI2 core (%d)\n", status);
		return DRVMGR_FAIL;
	}

	/* Initialize/Register Driver Manager PCI Bus */
	return pcibus_register(dev, &priv->config);
}

int grpci2_init3(struct rtems_drvmgr_dev_info *dev)
{
	struct grpci2_priv *priv = dev->priv;

	/* Enable PCI Error Interrupt handler */
	rtems_drvmgr_interrupt_enable(dev, 0, grpci2_err_isr, (void *)priv);

	/* Unmask Error IRQ and all PCI interrupts at PCI Core. For this to be
	 * safe every PCI board have to be resetted (no IRQ generation) before
	 * Global IRQs are enabled (Init is reached or similar)
	 */
	priv->regs->ctrl |= (CTRL_EI | priv->irq_mask);

	return DRVMGR_OK;
}
