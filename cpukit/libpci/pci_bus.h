/*  PCI bus driver Interface.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  General part of PCI Bus driver. The driver is typically
 *  initialized from the PCI host driver separating the host
 *  driver from the common parts in PCI drivers.
 *  The PCI library must be initialized before starting the
 *  PCI bus driver. The PCI library have set up BARs and
 *  assigned system IRQs for targets.
 *  This PCI bus driver rely on the PCI library (pci.c) for
 *  interrupt registeration (pci_interrupt_register) and PCI
 *  target set up.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __PCI_BUS_H__
#define __PCI_BUS_H__

#include <drvmgr/drvmgr.h>
#include <pci.h>
#include <pci/access.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PCI Driver ID generation (VENDOR: 16-bit, DEVICE: 16-bit) */
#define DRIVER_PCI_ID(vendor, device) \
	DRIVER_ID(DRVMGR_BUS_TYPE_PCI, \
		((((vendor) & 0xffff) << 16) | ((device) & 0xffff)))

/* PCI Driver ID generation (CLASS: 24-bit) */
#define DRIVER_PCI_CLASS(class) \
	DRIVER_ID(DRVMGR_BUS_TYPE_PCI, ((1 << 32) | ((class) & 0xffffff)))

/* PCI driver IDs  (DRIVER_PCI_VENDOR_DEVICE or DRIVER_PCI_CLASS_NAME) */
#define DRIVER_PCI_GAISLER_RASTAIO_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_IO)
#define DRIVER_PCI_GAISLER_RASTATMTC_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_TMTC)
#define DRIVER_PCI_GAISLER_GR701_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_701)
#define DRIVER_PCI_GAISLER_RASTAADCDAC_ID	DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_ADCDAC)
#define DRIVER_PCI_GAISLER_TMTC_1553_ID	        DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_TMTC_1553)

struct pci_dev_id {
	uint16_t		vendor;
	uint16_t		device;
	uint16_t		subvendor;
	uint16_t		subdevice;
	uint32_t		class; /* 24 lower bits */
};

struct pci_dev_id_match {
	uint16_t		vendor;
	uint16_t		device;
	uint16_t		subvendor;
	uint16_t		subdevice;
	uint32_t		class;  /* 24 lower bits */
	uint32_t		class_mask; /* 24 lower bits */
};
#define PCIID_DEVVEND(vendor, device) \
	{vendor, device, PCI_ID_ANY, PCI_ID_ANY, 0, 0}
#define PCIID_END_TABLE {0, 0, 0, 0, 0, 0}

enum {
	/* A Device has up to 6 BARs and an optional ROM BAR */
	PCIDEV_RES_BAR1 = 0,
	PCIDEV_RES_BAR2 = 1,
	PCIDEV_RES_BAR3 = 2,
	PCIDEV_RES_BAR4 = 3,
	PCIDEV_RES_BAR5 = 4,
	PCIDEV_RES_BAR6 = 5,
	PCIDEV_RES_ROM  = 6,
};
/* Maximum Number of Resources of a device */
#define PCIDEV_RES_CNT (PCIDEV_RES_ROM + 1)

/* IO, MEMIO or MEM resource. Can be BAR, ROM or Bridge Window */
struct pcibus_res {
	uint32_t		address; /* Base Address, CPU accessible */
	uint32_t		size;    /* 0=Unimplemented, 0!=Resource Size */
	struct pci_res		*res;    /* PCI-layer resource */
};

struct pci_dev_info {
	struct pci_dev_id	id;
	uint8_t			rev;
	uint8_t			irq; /* 0 = NO IRQ */
	pci_dev_t		pcidev;
	struct pcibus_res	resources[PCIDEV_RES_CNT];
	struct pci_dev		*pci_device;
};

struct pci_drv_info {
	struct drvmgr_drv	general;	/* General bus info */
	/* PCI specific bus information */
	struct pci_dev_id_match		*ids;		/* Supported hardware */
};

/* Access routines */
struct pcibus_regmem_ops {
	drvmgr_r8 r8;
	drvmgr_r16 r16;
	drvmgr_r32 r32;
	drvmgr_r64 r64;
	drvmgr_w8 w8;
	drvmgr_w16 w16;
	drvmgr_w32 w32;
	drvmgr_w64 w64;
};

/* Let driver configure PCI bus driver */
struct pcibus_config {
	struct pcibus_regmem_ops *memreg_ops;
};

/* Select Sub-Function Read/Write function by ID */
#define RW_SIZE_1   0x0001    /* Access Size */
#define RW_SIZE_2   0x0002
#define RW_SIZE_4   0x0004
#define RW_SIZE_8   0x0008
#define RW_SIZE_ANY 0x0000

#define RW_DIR_ANY  0x0000   /* Access Direction */
#define RW_READ     0x0000
#define RW_WRITE    0x0010
#define RW_SET      0x0020

#define RW_TYPE_ANY 0x0000  /* Access type */
#define RW_REG      0x0100
#define RW_MEM      0x0200
#define RW_MEMREG   0x0300
#define RW_CFG      0x0400

#define RW_ARG      0x1000 /* Optional Argument */
#define RW_ERR      0x2000 /* Optional Error Handler */

/* Build a Read/Write function ID */
#define DRVMGR_RWFUNC(minor) DRVMGR_FUNCID(FUNCID_RW, minor)

/* PCI I/O Register Access - Not implemented */
#define PCI_IO_R8    DRVMGR_RWFUNC(RW_SIZE_1|RW_READ|RW_REG)
#define PCI_IO_R16   DRVMGR_RWFUNC(RW_SIZE_2|RW_READ|RW_REG)
#define PCI_IO_R32   DRVMGR_RWFUNC(RW_SIZE_4|RW_READ|RW_REG)
#define PCI_IO_R64   DRVMGR_RWFUNC(RW_SIZE_8|RW_READ|RW_REG)
#define PCI_IO_W8    DRVMGR_RWFUNC(RW_SIZE_1|RW_WRITE|RW_REG)
#define PCI_IO_W16   DRVMGR_RWFUNC(RW_SIZE_2|RW_WRITE|RW_REG)
#define PCI_IO_W32   DRVMGR_RWFUNC(RW_SIZE_4|RW_WRITE|RW_REG)
#define PCI_IO_W64   DRVMGR_RWFUNC(RW_SIZE_8|RW_WRITE|RW_REG)

/* PCI Register Access over Memory Space */
#define PCI_MREG_R8   DRVMGR_RWFUNC(RW_SIZE_1|RW_READ|RW_MEMREG)
#define PCI_MREG_R16  DRVMGR_RWFUNC(RW_SIZE_2|RW_READ|RW_MEMREG)
#define PCI_MREG_R32  DRVMGR_RWFUNC(RW_SIZE_4|RW_READ|RW_MEMREG)
#define PCI_MREG_R64  DRVMGR_RWFUNC(RW_SIZE_8|RW_READ|RW_MEMREG)
#define PCI_MREG_W8   DRVMGR_RWFUNC(RW_SIZE_1|RW_WRITE|RW_MEMREG)
#define PCI_MREG_W16  DRVMGR_RWFUNC(RW_SIZE_2|RW_WRITE|RW_MEMREG)
#define PCI_MREG_W32  DRVMGR_RWFUNC(RW_SIZE_4|RW_WRITE|RW_MEMREG)
#define PCI_MREG_W64  DRVMGR_RWFUNC(RW_SIZE_8|RW_WRITE|RW_MEMREG)

#if 0
extern uint8_t pcibus_cfg_r8(struct drvmgr_dev *dev, int ofs);
extern uint16_t pcibus_cfg_r16(struct drvmgr_dev *dev, int ofs);
extern uint32_t pcibus_cfg_r32(struct drvmgr_dev *dev, int ofs);

extern void pcibus_cfg_w8(struct drvmgr_dev *dev, int ofs, uint8_t value);
extern void pcibus_cfg_w16(struct drvmgr_dev *dev, int ofs, uint16_t value);
extern void pcibus_cfg_w32(struct drvmgr_dev *dev, int ofs, uint32_t value);
#endif

/* Weak default PCI driver resources, override this from project configuration
 * to set PCI Bus resources used to configure PCI device drivers.
 */
extern struct drvmgr_bus_res pcibus_drv_resources;

/* Attach a PCI bus on top of a PCI Host device */
extern int pcibus_register(struct drvmgr_dev *dev);

#ifdef __cplusplus
}
#endif

#endif
