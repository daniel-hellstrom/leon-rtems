/*  General part of a AMBA Plug & Play bus driver.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  This is the general part of the different AMBA Plug & Play
 *  drivers. The drivers are wrappers around this driver, making
 *  the code size smaller for systems with multiple AMBA Plug & 
 *  Play buses.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2009-11-20, Daniel Hellstrom <daniel@gaisler.com>
 *   Added support for AMBA-RMAP
 *
 *  2008-12-03, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>

#include <bsp.h>
#include <ambapp.h>

/*#define DEBUG 1*/
#define DBG(args...)
/*#define DBG(args...) printk(args)*/

char *ambapp_device_id2str(int vendor, int id);

struct grlib_gptimer_regs {
	volatile unsigned int scaler_value;   /* common timer registers */
	volatile unsigned int scaler_reload;
	volatile unsigned int status;
	volatile unsigned int notused;
};

/* AMBA IMPLEMENTATION */

int ambapp_bus_init1(struct rtems_drvmgr_bus_info *bus);
int ambapp_bus_remove(struct rtems_drvmgr_bus_info *bus);
int ambapp_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev);
int ambapp_int_register(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int ambapp_int_unregister(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int ambapp_int_enable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int ambapp_int_disable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int ambapp_int_clear(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int ambapp_int_mask(struct rtems_drvmgr_dev_info *dev, int index);
int ambapp_int_unmask(struct rtems_drvmgr_dev_info *dev, int index);
int ambapp_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params);
int ambapp_bus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz);

struct rtems_drvmgr_bus_ops ambapp_bus_ops =
{
	.init		= 
	{
		/* init1 */ ambapp_bus_init1,
		/* init2 */ NULL,
		/* init3 */ NULL,
		/* init4 */ NULL
	},
	.remove		= ambapp_bus_remove,
	.unite		= ambapp_unite,
	.int_register	= ambapp_int_register,
	.int_unregister	= ambapp_int_unregister,
	.int_enable	= ambapp_int_enable,
	.int_disable	= ambapp_int_disable,
	.int_clear	= ambapp_int_clear,
	.int_mask	= ambapp_int_mask,
	.int_unmask	= ambapp_int_unmask,
	.get_params	= ambapp_get_params,
	.freq_get	= ambapp_bus_freq_get,
};

struct ambapp_priv {
	struct ambapp_config		*config;
};

int ambapp_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev)
{
	struct amba_drv_info *adrv;
	struct amba_dev_id *id;
	struct amba_dev_info *amba;

	if ( !drv || !dev || !dev->parent )
		return 0;

	if ( ! (((drv->bus_type == DRVMGR_BUS_TYPE_AMBAPP) && (dev->parent->bus_type == DRVMGR_BUS_TYPE_AMBAPP)) ||
	       ((drv->bus_type == DRVMGR_BUS_TYPE_AMBAPP_RMAP) && (dev->parent->bus_type == DRVMGR_BUS_TYPE_AMBAPP_RMAP)) ||
	       ((drv->bus_type == DRVMGR_BUS_TYPE_AMBAPP_DIST) && (dev->parent->bus_type == DRVMGR_BUS_TYPE_AMBAPP_DIST)))
	   ) {
		return 0;
	}

	amba = (struct amba_dev_info *)dev->businfo;
	if ( !amba )
		return 0;

	adrv = (struct amba_drv_info *)drv;
	id = adrv->ids;
	if ( !id )
		return 0;
	while( id->vendor != 0 ) {
		if ( (id->vendor == amba->id.vendor) &&
		      (id->device == amba->id.device) ) {
			/* Unite device and driver */
			DBG("DRV 0x%x and DEV 0x%x united\n", (unsigned int)drv, (unsigned int)dev);
			return 1;
		}
		id++;
	}

	return 0;
}

static int ambapp_int_get(struct rtems_drvmgr_dev_info *dev, int index)
{
	int irq;

	/* Relative (positive) or absolute (negative) IRQ number */
	if ( index >= 0 ) {
		/* IRQ Index relative to Cores base IRQ */

		/* Get Base IRQ */
		irq = ((struct amba_dev_info *)dev->businfo)->info.irq;
		if ( irq < 0 )
			return -1;
		irq += index;
	} else {
		/* Absolute IRQ number */
		irq = -index;
	}
	return irq;
}

int ambapp_int_register(
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
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Register interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_register ) {
		/* Let device override driver default */
		return priv->config->ops->int_register(dev, irq, isr, arg);
	} else {
		return -1;
	}
}

int ambapp_int_unregister(
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
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Unregister interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_unregister ) {
		/* Let device override driver default */
		return priv->config->ops->int_unregister(dev, irq, isr, arg);
	} else {
		return -1;
	}
}

int ambapp_int_enable(
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
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Enable interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_enable ) {
		/* Let device override driver default */
		return priv->config->ops->int_enable(dev, irq, isr, arg);
	} else {
		return -1;
	}
}

int ambapp_int_disable(
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
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Disable interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_disable ) {
		/* Let device override driver default */
		return priv->config->ops->int_disable(dev, irq, isr, arg);
	} else {
		return -1;
	}
}

int ambapp_int_clear(
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
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("Clear interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_clear ) {
		/* Let device override driver default */
		return priv->config->ops->int_clear(dev, irq, isr, arg);
	} else {
		return -1;
	}
}

int ambapp_int_mask(
	struct rtems_drvmgr_dev_info *dev,
	int index)
{
	struct rtems_drvmgr_dev_info *busdev;
	struct ambapp_priv *priv;
	int irq;

	busdev = dev->parent->dev;
	priv = dev->parent->priv;

	/* Get IRQ number from index and device information */
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("MASK interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_mask ) {
		/* Let device override driver default */
		return priv->config->ops->int_mask(dev, irq);
	} else {
		return -1;
	}
}

int ambapp_int_unmask(
	struct rtems_drvmgr_dev_info *dev,
	int index)
{
	struct rtems_drvmgr_dev_info *busdev;
	struct ambapp_priv *priv;
	int irq;

	busdev = dev->parent->dev;
	priv = dev->parent->priv;

	/* Get IRQ number from index and device information */
	irq = ambapp_int_get(dev, index);
	if ( irq < 0 ) 
		return -1;

	DBG("UNMASK interrupt on 0x%x for dev 0x%x (IRQ: %d)\n", (unsigned int)busdev, (unsigned int)dev, irq);

	if ( priv->config->ops->int_unmask ) {
		/* Let device override driver default */
		return priv->config->ops->int_unmask(dev, irq);
	} else {
		return -1;
	}
}

/* Assign frequency to an AMBA Bus */
void ambapp_bus_freq_register(
	struct rtems_drvmgr_dev_info *dev,
	int amba_interface,
	unsigned int freq_hz
	)
{
	struct ambapp_priv *priv = (struct ambapp_priv *)dev->parent->priv;
	struct ambapp_dev *adev;
	struct amba_dev_info *pnp = dev->businfo;

	if ( freq_hz == 0 )
		return;

	if ( amba_interface == DEV_AHB_MST ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.ahb_mst -
				sizeof(struct ambapp_dev));
	} else if ( amba_interface == DEV_AHB_SLV ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.ahb_slv -
				sizeof(struct ambapp_dev));
	} else if ( amba_interface == DEV_APB_SLV ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.apb_slv -
				sizeof(struct ambapp_dev));
	} else {
		return;
	}

	/* Calculate Top bus frequency from lower part. The frequency comes
	 * from some kind of hardware able to report local bus frequency.
	 */
	ambapp_freq_init(priv->config->abus, adev, freq_hz);
}

int ambapp_bus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz)
{
	struct ambapp_priv *priv = (struct ambapp_priv *)dev->parent->priv;
	struct ambapp_dev *adev;
	struct amba_dev_info *pnp = dev->businfo;

	if ( options == DEV_AHB_MST ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.ahb_mst -
				sizeof(struct ambapp_dev));
	} else if ( options == DEV_AHB_SLV ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.ahb_slv -
				sizeof(struct ambapp_dev));
	} else if ( options == DEV_APB_SLV ) {
		adev = (struct ambapp_dev *)
			((unsigned int)pnp->info.apb_slv -
				sizeof(struct ambapp_dev));
	} else {
		*freq_hz = 0;
		return -1;
	}

	/* Calculate core/bus frequency from top most bus frequency. */
	*freq_hz = ambapp_freq_get(priv->config->abus, adev);
	if ( *freq_hz == 0 )
		return -1;
	return 0;
}

int ambapp_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	struct ambapp_priv *priv = dev->parent->priv;

	if ( priv->config->ops->get_params ) {
		/* Let device override driver default */
		return priv->config->ops->get_params(dev, params);
	} else {
		return -1;
	}
}

/* Fix device in last stage */
int ambapp_dev_fixup(struct rtems_drvmgr_dev_info *dev, struct amba_dev_info *pnp)
{
	/* OCCAN speciality:
	 *  Mulitple cores are supported through the same amba AHB interface.
	 *  The number of "sub cores" can be detected by decoding the AMBA
	 *  Plug&Play version information. verion = ncores. A maximum of 8
	 *  sub cores are supported, each separeated with 0x100 inbetween.
	 *
	 *  Now, lets detect sub cores.
	 */
	if ( (pnp->info.device == GAISLER_CANAHB) && (pnp->info.vendor == VENDOR_GAISLER) ) {
		struct rtems_drvmgr_dev_info *newdev;
		struct amba_dev_info *pnpinfo;
		int subcores;
		int core;

		subcores = (pnp->info.ahb_slv->ver & 0x7) + 1;
		for(core = 1; core < subcores; core++) {
			rtems_drvmgr_alloc_dev(&newdev, sizeof(*pnpinfo));
			memcpy(newdev, dev, sizeof(*newdev));
			pnpinfo = (struct amba_dev_info *)(newdev+1);
			memcpy(pnpinfo, pnp, sizeof(*pnp));
			pnpinfo->info.index = core;
			pnpinfo->info.irq += core;
			newdev->businfo = (void *)pnpinfo;

			/* Register device */
			rtems_drvmgr_dev_register(newdev);
		}
	} else if ( (pnp->info.device == GAISLER_GPIO) && (pnp->info.vendor == VENDOR_GAISLER) ) {
		/* PIO[N] is connected to IRQ[N]. */
		pnp->info.irq = 0;
	}
	return 0;
}

struct ambapp_dev_reg_struct {
	struct ambapp_bus		*abus;
	struct rtems_drvmgr_bus_info	*bus;
	struct ambapp_dev		*ahb_mst;
	struct ambapp_dev		*ahb_slv;
	struct ambapp_dev		*apb_slv;
};

void ambapp_core_register(
	struct ambapp_dev	*ahb_mst,
	struct ambapp_dev	*ahb_slv,
	struct ambapp_dev	*apb_slv,
	struct ambapp_dev_reg_struct *arg
	)
{
	struct rtems_drvmgr_dev_info *newdev;
	struct amba_dev_info *pnpinfo;
	unsigned short device;
	unsigned char vendor;

	if ( ahb_mst ) {
		device = ahb_mst->device;
		vendor = ahb_mst->vendor;
	}else if ( ahb_slv ) {
		device = ahb_slv->device;
		vendor = ahb_slv->vendor;
	}else if( apb_slv ) {
		device = apb_slv->device;
		vendor = apb_slv->vendor;
	} else {
		DBG("NO DEV!\n");
		return;
	}

	DBG("CORE REGISTER DEV [%x:%x] MST: 0x%x, SLV: 0x%x, APB: 0x%x\n", vendor, device, (unsigned int)ahb_mst, (unsigned int)ahb_slv, (unsigned int)apb_slv);

	/* Allocate a device */
	rtems_drvmgr_alloc_dev(&newdev, sizeof(struct amba_dev_info));
	pnpinfo = (struct amba_dev_info *)(newdev + 1);
	newdev->next = NULL;
	newdev->parent = arg->bus; /* Ourselfs */
	newdev->minor_drv = 0;
	newdev->minor_bus = 0;
	newdev->priv = NULL;
	newdev->drv = NULL;
	newdev->name = ambapp_device_id2str(vendor, device);
	newdev->next_in_drv = NULL;
	newdev->bus = NULL;

	/* Init PnP information, Assign Core interfaces with this device */
	pnpinfo->id.vendor = vendor;
	pnpinfo->id.device = device;
	pnpinfo->info.vendor = vendor;
	pnpinfo->info.device = device;
	pnpinfo->info.index = 0;
	if ( ahb_mst ) {
		pnpinfo->info.ahb_mst = (struct ambapp_ahb_info *)
						ahb_mst->devinfo;
		ambapp_alloc_dev(ahb_mst, (void *)newdev);
		if ( pnpinfo->info.ahb_mst->irq )
			pnpinfo->info.irq = pnpinfo->info.ahb_mst->irq;
	}
	if ( ahb_slv ) {
		pnpinfo->info.ahb_slv = (struct ambapp_ahb_info *)
					ahb_slv->devinfo;
		ambapp_alloc_dev(ahb_slv, (void *)newdev);
		if ( pnpinfo->info.ahb_slv->irq )
			pnpinfo->info.irq = pnpinfo->info.ahb_slv->irq;
	}
	if ( apb_slv ) {
		pnpinfo->info.apb_slv = (struct ambapp_apb_info *)
					apb_slv->devinfo;
		ambapp_alloc_dev(apb_slv, (void *)newdev);
		if ( pnpinfo->info.apb_slv->irq )
			pnpinfo->info.irq = pnpinfo->info.apb_slv->irq;
	}
	if ( pnpinfo->info.irq == 0 )
		pnpinfo->info.irq = -1; /* indicate no IRQ */

	/* Connect device with PnP information */
	newdev->businfo = (void *)pnpinfo;

	ambapp_dev_fixup(newdev, pnpinfo);

	/* Register New Device */
	rtems_drvmgr_dev_register(newdev);
}

/* Register one AMBA device */
int ambapp_dev_register(struct ambapp_dev *dev, int index, int maxpdepth, void *arg)
{
	struct ambapp_dev_reg_struct *p = arg;

#ifdef DEBUG
	char *type;

	if ( dev->dev_type == DEV_AHB_MST )
		type = "AHB MST";
	else if ( dev->dev_type == DEV_AHB_SLV )
		type = "AHB SLV";
	else if ( dev->dev_type == DEV_APB_SLV )
		type = "APB SLV";
	
	DBG("Found [%d:%x:%x], %s\n", index, dev->vendor, dev->device, type);
#endif

	if ( dev->dev_type == DEV_AHB_MST ) {
		if ( p->ahb_mst ) {
			/* This should not happen */
			printk("ambapp_dev_register: ahb_mst not NULL!\n");
			exit(1);
		}

		/* Remember AHB Master */
		p->ahb_mst = dev;

		/* Find AHB Slave and APB slave for this Core */
		ambapp_for_each(p->abus->root, (OPTIONS_AHB_SLVS|OPTIONS_APB_SLVS|OPTIONS_FREE), dev->vendor, dev->device, 10, ambapp_dev_register, p);

		ambapp_core_register(p->ahb_mst, p->ahb_slv, p->apb_slv, p);
		p->ahb_mst = p->ahb_slv = p->apb_slv = NULL;
		return 0;

	} else if ( dev->dev_type == DEV_AHB_SLV ) {
		if ( p->ahb_slv ) {
			/* Already got our AHB Slave interface */
			return 0;
		}

		/* Remember AHB Slave */
		p->ahb_slv = dev;

		if ( p->ahb_mst ) {
			/* Continue searching for APB Slave */
			return 0;
		} else {
			/* Find APB Slave interface for this Core */
			ambapp_for_each(p->abus->root, (OPTIONS_APB_SLVS|OPTIONS_FREE), dev->vendor, dev->device, 10, ambapp_dev_register, p);

			ambapp_core_register(p->ahb_mst, p->ahb_slv, p->apb_slv, p);
			p->ahb_mst = p->ahb_slv = p->apb_slv = NULL;
			return 0;
		}
	} else if ( dev->dev_type == DEV_APB_SLV ) {
		if ( p->apb_slv ) {
			/* This should not happen */
			printk("ambapp_dev_register: apb_slv not NULL!\n");
			exit(1);
		}
		/* Remember APB Slave */
		p->apb_slv = dev;

		if ( p->ahb_mst || p->ahb_slv ) {
			/* Stop scanning */
			return 1;
		} else {
			ambapp_core_register(p->ahb_mst, p->ahb_slv, p->apb_slv, p);
			p->ahb_mst = p->ahb_slv = p->apb_slv = NULL;
			return 0;
		}
	}

	return 0;
}

/* Register all AMBA devices available on the AMBAPP bus */
int ambapp_ids_register(struct rtems_drvmgr_bus_info *bus)
{
	struct ambapp_priv *priv = bus->priv;
	struct ambapp_bus *abus;
	struct ambapp_dev_reg_struct arg;

	DBG("ambapp_ids_register:\n");

	memset(&arg, 0, sizeof(arg));

	abus = priv->config->abus;
	arg.abus = abus;
	arg.bus = bus;

	/* Combine the AHB MST, AHB SLV and APB SLV interfaces of a core. A core has often more than
	 * one interface. A core can not have more than one interface of the same type.
	 */
	ambapp_for_each(abus->root, (OPTIONS_ALL_DEVS|OPTIONS_FREE), -1, -1, 10, ambapp_dev_register, &arg);

#ifdef DEBUG
	ambapp_print(abus->root, 1);
#endif

	return DRVMGR_OK;
}

/*** DEVICE FUNCTIONS ***/

int ambapp_bus_register(struct rtems_drvmgr_dev_info *dev, struct ambapp_config *config)
{
	struct ambapp_priv *priv;

	if ( !config || !config->ops )
		return DRVMGR_OK;

	DBG("AMBAPP BUS: initializing\n");

	/* Register BUS */
	rtems_drvmgr_alloc_bus(&dev->bus, sizeof(struct ambapp_priv));
	priv = (struct ambapp_priv *)(dev->bus + 1);
	priv->config = config;
	if ( priv->config->bus_type == DRVMGR_BUS_TYPE_AMBAPP_DIST )
		dev->bus->bus_type = DRVMGR_BUS_TYPE_AMBAPP_DIST;
	else if ( priv->config->bus_type == DRVMGR_BUS_TYPE_AMBAPP_RMAP )
		dev->bus->bus_type = DRVMGR_BUS_TYPE_AMBAPP_RMAP;
	else
		dev->bus->bus_type = DRVMGR_BUS_TYPE_AMBAPP;
	dev->bus->next = NULL;
	dev->bus->dev = dev;
	dev->bus->priv = priv;
	dev->bus->children = NULL;
	dev->bus->ops = &ambapp_bus_ops;
	dev->bus->dev_cnt = 0;
	dev->bus->reslist = NULL;
	dev->bus->mmaps = config->mmaps;

	/* Add resource configuration */
	if ( priv->config->resources )
		rtems_drvmgr_bus_res_add(dev->bus, priv->config->resources);

	rtems_drvmgr_bus_register(dev->bus);

	return DRVMGR_OK;
}

/*** BUS INITIALIZE FUNCTIONS ***/

/* Initialize the bus, register devices on this bus */
int ambapp_bus_init1(struct rtems_drvmgr_bus_info *bus)
{
	/* Initialize the bus, register devices on this bus */
	return ambapp_ids_register(bus);
}

int ambapp_bus_remove(struct rtems_drvmgr_bus_info *bus)
{
	return DRVMGR_OK;
}
