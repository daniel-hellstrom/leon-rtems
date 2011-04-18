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

struct pcibus_config {
	struct rtems_drvmgr_drv_res *resources;	/* Driver Resources */
};

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
#define PCIID_DEVVEND(vendor, device) {vendor,device,PCI_ID_ANY,PCI_ID_ANY,0,0}
#define PCIID_END_TABLE {0,0,0,0,0,0}

struct pci_dev_info {
	struct pci_dev_id	id;
	uint8_t			rev;
	uint8_t			irq; /* 0 = NO IRQ */
	pci_dev_t		pcidev;
	struct pci_dev		*pci_device;
};

struct pci_drv_info {
	struct rtems_drvmgr_drv_info	general;	/* General bus info */
	/* PCI specific bus information */
	struct pci_dev_id_match		*ids;		/* Supported hardware */
};

#if 0
extern uint8_t pcibus_cfg_r8(struct rtems_drvmgr_dev_info *dev, int ofs);
extern uint16_t pcibus_cfg_r16(struct rtems_drvmgr_dev_info *dev, int ofs);
extern uint32_t pcibus_cfg_r32(struct rtems_drvmgr_dev_info *dev, int ofs);

extern void pcibus_cfg_w8(struct rtems_drvmgr_dev_info *dev, int ofs, uint8_t value);
extern void pcibus_cfg_w16(struct rtems_drvmgr_dev_info *dev, int ofs, uint16_t value);
extern void pcibus_cfg_w32(struct rtems_drvmgr_dev_info *dev, int ofs, uint32_t value);
#endif

/* Attach a PCI bus on top of a PCI Host device */
extern int pcibus_register(
	struct rtems_drvmgr_dev_info *dev,
	struct pcibus_config *config);

#ifdef __cplusplus
}
#endif

#endif
