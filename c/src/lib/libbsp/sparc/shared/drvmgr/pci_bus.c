/*  PCI bus driver.
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
 *  2008-12-03, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pci.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/pci_bus.h>

#define INSERT_DEV_LAST_IN_BUS_DEVLIST

#define DBG(args...)
/*#define DBG(args...) printk(args)*/
int pcibus_bus_init1(struct rtems_drvmgr_bus_info *bus);
int pcibus_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev);
int pcibus_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int pcibus_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int pcibus_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int pcibus_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int pcibus_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int pcibus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz);

int pcibus_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params);

int pcibus_dev_id_compare(struct pci_dev_info *a, struct pci_dev_info *b);

struct rtems_drvmgr_bus_ops pcibus_ops =
{
	.init		= 
			{
				pcibus_bus_init1,
				NULL,
				NULL,
				NULL
			},
	.remove		= NULL,
	.unite		= pcibus_unite,
	.dev_id_compare = (int(*)(void*,void*)) pcibus_dev_id_compare,
	.int_register	= pcibus_int_register,
	.int_unregister	= pcibus_int_unregister,
	.int_enable	= pcibus_int_enable,
	.int_disable	= pcibus_int_disable,
	.int_clear	= pcibus_int_clear,
	.int_mask	= NULL,
	.int_unmask	= NULL,
	.get_params	= pcibus_get_params,
	.freq_get	= pcibus_freq_get,
};

struct pcibus_priv {
	struct rtems_drvmgr_dev_info	*dev;
	struct pcibus_config		*config;
};

int pcibus_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev)
{
	struct pci_drv_info *pdrv;
	struct pci_dev_id *id;
	struct pci_dev_info *pci;

	if ( !drv || !dev || !dev->parent )
		return 0;

	if ( (drv->bus_type!=DRVMGR_BUS_TYPE_PCI) || (dev->parent->bus_type != DRVMGR_BUS_TYPE_PCI) ) {
		return 0;
	}

	pci = (struct pci_dev_info *)dev->businfo;
	if ( !pci )
		return 0;

	pdrv = (struct pci_drv_info *)drv;
	id = pdrv->ids;
	if ( !id )
		return 0;
	while( id->vendor != 0 ) {
		if ( (id->vendor == pci->id.vendor) &&
		      (id->device == pci->id.device) ) {
			/* Unite device and driver */
			DBG("DRV 0x%x and DEV 0x%x united\n", (unsigned int)drv, (unsigned int)dev);
			return 1;
		}
		id++;
	}

	return 0;
}

static int pcibus_int_get(struct rtems_drvmgr_dev_info *dev, int index)
{
	int irq;

	/* Relative (positive) or absolute (negative) IRQ number */
	if ( index >= 0 ) {
		/* IRQ Index relative to Cores base IRQ */

		/* Get Base IRQ */
		irq = ((struct pci_dev_info *)dev->businfo)->irq;
		if ( irq <= 0 )
			return -1;
		irq += index;
	} else {
		/* Absolute IRQ number */
		irq = -index;
	}
	return irq;
}

/* Use standard PCI facility to register interrupt handler */
int pcibus_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct rtems_drvmgr_dev_info *busdev;
	struct ambapp_priv *priv;
	int irq;

	busdev = dev->parent->dev;
	priv = dev->parent->priv;

	/* Get IRQ number from index and device information */
	irq = pcibus_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Register PCI interrupt on 0x%x for dev 0x%x (IRQ: %d)\n",
		(unsigned int)busdev, (unsigned int)dev, irq);

	return pci_interrupt_register(irq, isr, arg);
}

/* Use standard PCI facility to unregister interrupt handler */
int pcibus_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	struct rtems_drvmgr_dev_info *busdev;
	struct ambapp_priv *priv;
	int irq;

	busdev = dev->parent->dev;
	priv = dev->parent->priv;

	/* Get IRQ number from index and device information */
	irq = pcibus_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Unregister PCI interrupt on 0x%x for dev 0x%x (IRQ: %d)\n",
		(unsigned int)busdev, (unsigned int)dev, irq);

	return pci_interrupt_unregister(irq, isr, arg);
}

/* Use standard PCI facility to enable interrupt */
int pcibus_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	int irq;

	/* Get IRQ number from index and device information */
	irq = pcibus_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	return pci_interrupt_enable(irq, isr, arg);
}

/* Use standard PCI facility to disable interrupt */
int pcibus_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	int irq;

	/* Get IRQ number from index and device information */
	irq = pcibus_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	return pci_interrupt_disable(irq, isr, arg);
}

/* Use standard PCI facility to clear interrupt */
int pcibus_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	int irq;

	/* Get IRQ number from index and device information */
	irq = pcibus_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	return pci_interrupt_clear(irq, isr, arg);
}

int pcibus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz)
{
	/* Standard PCI Bus frequency */
	*freq_hz = 33000000;
	return 0;
}

int pcibus_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	/* No device prefix */
	params->dev_prefix = NULL;

	return 0;
}

int pcibus_dev_id_compare(struct pci_dev_info *a, struct pci_dev_info *b)
{
	if ( (a->id.vendor == b->id.vendor) && (a->id.device == b->id.device) )
		return 0; /* Match */
	return 1;
}

int pcibus_for_each(int (*func)(int, int, int, void*), void *arg)
{
	unsigned int d;
	unsigned char bus,dev,fun,hd;

	for (bus=0; bus<BusCountPCI(); bus++) {
		dev = 0;
		if ( bus == 0 )
			dev = 1; /* Skip PCI host bridge */
		for (; dev<PCI_MAX_DEVICES; dev++) {

			pci_read_config_byte(bus, dev, 0, PCI_HEADER_TYPE, &hd);
			hd = (hd & PCI_MULTI_FUNCTION ? PCI_MAX_FUNCTIONS : 1);

			for (fun=0; fun<hd; fun++) {
				/* 
				 * The last devfn id/slot is special; must skip it
				 */
				if (PCI_MAX_DEVICES-1==dev && PCI_MAX_FUNCTIONS-1 == fun)
					break;

				pci_read_config_dword(bus, dev, fun, PCI_VENDOR_ID, &d);
				if ((PCI_INVALID_VENDORDEVICEID == d) || (0 == d))
					continue;

				DBG("pcibus_for_each: found 0x%08x at %d/%d/%d\n",d,bus,dev,fun);
				if ( func(bus, dev, fun, arg) == 1 ) {
					return 1; /* Stopped */
				}

			}
		}
	}
	/* Scanned all PCI devices */
	return 0;
}

int pcibus_dev_register(int bus, int dev, int func, void *arg)
{
	struct rtems_drvmgr_bus_info *pcibus = arg;
	struct rtems_drvmgr_dev_info *newdev;
	struct pci_dev_info *pciinfo;
#ifdef INSERT_DEV_LAST_IN_BUS_DEVLIST
	struct rtems_drvmgr_dev_info *device;
#endif

	DBG("PCI DEV REGISTER: %d:%d:%d\n", bus, dev, func);

	/* Allocate a device */
	rtems_drvmgr_alloc_dev(&newdev, sizeof(struct pci_dev_info));
	newdev->next = NULL;
	newdev->parent = pcibus; /* Ourselfs */
	newdev->minor_drv = 0;
	newdev->minor_bus = 0;
	newdev->priv = NULL;
	newdev->drv = NULL;
	newdev->name = malloc(32);
	newdev->next_in_drv = NULL;
#ifdef INSERT_DEV_LAST_IN_BUS_DEVLIST
	/* Insert Last in Bus Device Queue */
	if ( newdev->parent->children ) {
		device = newdev->parent->children;
		while ( device->next_in_bus )
			device = device->next_in_bus;
		device->next_in_bus = newdev;
	} else {
		newdev->parent->children = newdev;
	}
	newdev->next_in_bus = NULL;
#else
	/* Insert First in Bus Device Queue */
	newdev->next_in_bus = newdev->parent->children;
	newdev->parent->children = newdev;
#endif
	newdev->bus = NULL;

	/* Init PnP information, Assign Core interfaces with this device */
	pciinfo = (struct pci_dev_info *)(newdev + 1);

	/* Read Device and Vendor */
	pci_read_config_word(bus, dev, func, PCI_VENDOR_ID, &pciinfo->id.vendor);
	pci_read_config_word(bus, dev, func, PCI_DEVICE_ID, &pciinfo->id.device);

	/* Read IRQ information set by PCI layer */
	pci_read_config_byte(bus, dev, func, PCI_INTERRUPT_LINE, &pciinfo->irq);

	/* Save Location */
	pciinfo->bus = bus;
	pciinfo->dev = dev;
	pciinfo->func = func;

	/* Connect device with PCI information */
	newdev->businfo = (void *)pciinfo;

	sprintf(newdev->name, "PCI_%d:%d:%d_%04x:%04x", bus, dev, func, pciinfo->id.vendor, pciinfo->id.device);

	/* Register New Device */
	rtems_drvmgr_dev_register(newdev);

	return 0;
}

/* Register all AMBA devices available on the AMBAPP bus */
int pcibus_devs_register(struct rtems_drvmgr_bus_info *bus)
{
	pcibus_for_each(pcibus_dev_register, bus);
	return DRVMGR_OK;
}

/*** DEVICE FUNCTIONS ***/

int pcibus_register(
	struct rtems_drvmgr_dev_info *dev, struct pcibus_config *config)
{
	struct pcibus_priv *priv;

	DBG("PCI BUS: initializing\n");

	/* Create BUS */
	rtems_drvmgr_alloc_bus(&dev->bus, sizeof(struct pcibus_priv));
	dev->bus->bus_type = DRVMGR_BUS_TYPE_PCI;
	dev->bus->next = NULL;
	dev->bus->dev = dev;
	dev->bus->children = NULL;
	dev->bus->ops = &pcibus_ops;
	dev->bus->dev_cnt = 0;
	dev->bus->reslist = NULL;
	dev->bus->mmaps = NULL;

	/* Init BUS private structures */
	priv = (struct pcibus_priv *)(dev->bus + 1);
	dev->bus->priv = priv;	
	priv->config = config;

	/* Register BUS */
	rtems_drvmgr_bus_register(dev->bus);

	return DRVMGR_OK;
}

/*** BUS INITIALIZE FUNCTIONS ***/

int pcibus_bus_init1(struct rtems_drvmgr_bus_info *bus)
{
	struct pcibus_priv *priv = (struct pcibus_priv *)bus->priv;

	/* Add resource configuration */
	if ( priv->config->resources )
		rtems_drvmgr_bus_res_add(bus, priv->config->resources);

	return pcibus_devs_register(bus);
}
