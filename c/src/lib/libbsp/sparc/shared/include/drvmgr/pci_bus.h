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

#ifdef __cplusplus
extern "C" {
#endif

/* PCI Driver ID generation (VENDOR: 16-bit, DEVICE: 16-bit) */
#define DRIVER_PCI_ID(vendor, device) \
	DRIVER_ID(DRVMGR_BUS_TYPE_PCI, ((((vendor) & 0xffff) << 16) | ((device) & 0xffff)))

/* PCI driver IDs */
#define DRIVER_PCI_GAISLER_RASTAIO_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_IO)
#define DRIVER_PCI_GAISLER_RASTATMTC_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_TMTC)
#define DRIVER_PCI_GAISLER_GR701_ID		DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_701)
#define DRIVER_PCI_GAISLER_RASTAADCDAC_ID	DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_RASTA_ADCDAC)
#define DRIVER_PCI_GAISLER_TMTC_1553_ID	        DRIVER_PCI_ID(PCIID_VENDOR_GAISLER, PCIID_DEVICE_GR_TMTC_1553)

struct pcibus_config {
	struct rtems_drvmgr_drv_res		*resources;	/* Driver Resources */
};

struct pci_dev_id {
	unsigned short		vendor;
	unsigned short		device;
};

struct pci_dev_info {
	struct pci_dev_id	id;
	int			bus;
	int			dev;
	int			func;
	unsigned char		irq;
};

struct pci_drv_info {
	struct rtems_drvmgr_drv_info	general;	/* General bus info */
	/* PCI specific bus information */
	struct pci_dev_id		*ids;		/* Supported hardware */
};

/* Attach a PCI bus on top of a PCI Host device */
extern int pcibus_register(
	struct rtems_drvmgr_dev_info *dev,
	struct pcibus_config *config);

#ifdef __cplusplus
}
#endif

#endif
