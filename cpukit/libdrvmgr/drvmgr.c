/* Driver Manager Interface Implementation.
 *
 *  COPYRIGHT (c) 2009-2011.
 *  Aeroflex Gaisler AB
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/drvmgr_confdefs.h>

#include "drvmgr_internal.h"

/* Enable debugging */
/*#define DEBUG 1*/

#ifdef DEBUG
#define DBG(x...) printk( x )
#else
#define DBG(x...) 
#endif

struct rtems_driver_manager drv_mgr =
{
	.level =		0,
	.initializing_objs =	0,
	.root_drv =		NULL,
	.root_dev =		NULL,

	.drivers =	LIST_INITIALIZER(struct rtems_drvmgr_drv_info, next),

	.buses = {
		LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),
	},
	.buses_inactive =	LIST_INITIALIZER(struct rtems_drvmgr_bus_info, next),

	.devices = {
		LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
		LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
	},
	.devices_inactive =	LIST_INITIALIZER(struct rtems_drvmgr_dev_info, next),
};

static int do_bus_init(
	struct rtems_driver_manager *mgr,
	struct rtems_drvmgr_bus_info *bus,
	int level);
static int do_dev_init(
	struct rtems_driver_manager *mgr,
	struct rtems_drvmgr_dev_info *dev,
	int level);

/* DRIVER MANAGER */

void _DRV_Manager_init_level(int level)
{
	struct rtems_driver_manager *mgr = &drv_mgr;

	if ( mgr->level >= level )
		return;

	/* Set new Level */
	mgr->level = level;

	/* Initialize buses and devices into this new level */
	drvmgr_init_update();
}

/* Initialize Data structures of the driver manager and call driver
 * register functions configured by the user.
 */
void _DRV_Manager_initialization(void)
{
	struct rtems_drvmgr_drv_reg_func *drvreg;

	/* drv_mgr is already initialized statically by compiler */

	/* Call driver register functions. */
	drvreg = &rtems_drvmgr_drivers[0];
	while ( drvreg->drv_reg ) {
		/* Make driver register */
		drvreg->drv_reg();
		drvreg++;
	}
}

/* Take ready devices and buses into the correct init level step by step.
 * Once a bus or a device has been registered there is no turning 
 * back - they are taken to the level of the driver manager.
 */
void drvmgr_init_update(void)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_bus_info *bus;
	struct rtems_drvmgr_dev_info *dev;
	int bus_might_been_registered;
	int level;

	/* "Lock" to make sure we don't use up the stack and that the lists
	 * remain consistent.
	 */
	if ( mgr->initializing_objs || (mgr->level == 0) )
		return;
	mgr->initializing_objs = 1;

init_registered_buses:
	/* Take all buses and devices ready into the same stage 
	 * as the driver manager global level.
	 */
	for (level=0; level<mgr->level; level++) {

		bus_might_been_registered = 0;

		/* Take buses into next level */

		while ( (bus=BUS_LIST_HEAD(&mgr->buses[level])) != NULL ) {

			/* Remove first in the list (will be inserted in
			 * appropriate list by do_bus_init())
			 */
			rtems_drvmgr_list_remove_head(&mgr->buses[level]);

			/* Initialize Bus, this will register devices on
			 * the bus. Take bus into next level.
			 */
			do_bus_init(mgr, bus, level+1);
		}

		/* Take devices into next level */
		while ( (dev=DEV_LIST_HEAD(&mgr->devices[level])) != NULL ) {

			/* Always process first in list */
			dev = DEV_LIST_HEAD(&mgr->devices[level]);

			/* Remove first in the list (will be inserted in
			 * appropriate list by do_dev_init())
			 */
			rtems_drvmgr_list_remove_head(&mgr->devices[level]);

			/* Initialize Device, this may register a new bus */
			do_dev_init(mgr, dev, level+1);

			bus_might_been_registered = 1;
		}

		/* Make sure all buses registered and ready are taken at
		 * the same time into init level N.
		 */
		if ( bus_might_been_registered )
			goto init_registered_buses;
	}

	/* Release bus/device initialization "Lock" */
	mgr->initializing_objs = 0;
}

/* Init driver manager. REMOVE THIS FUNCTION WHEN IMPLEMENTED IN RTEMS */
int rtems_drvmgr_init(void)
{
	int level;

	_DRV_Manager_initialization();

	for (level=1; level<=DRVMGR_LEVEL_MAX; level++) {
		_DRV_Manager_init_level(level);
	}

	return 0;
}

/* Take bus into next level */
static int do_bus_init(
	struct rtems_driver_manager *mgr,
	struct rtems_drvmgr_bus_info *bus,
	int level)
{
	int (*init)(struct rtems_drvmgr_bus_info *);

	/* If bridge device has failed during initialization, the bus is not
	 * initialized further.
	 */
	if (bus->dev->state & DEV_STATE_INIT_FAILED) {
		bus->state |= BUS_STATE_DEPEND_FAILED;
		goto inactivate_out;
	}

	if ( bus->ops && (init=bus->ops->init[level-1]) ) {
		/* Note: This init1 function may register new devices */
		if ( (bus->error=init(bus)) != DRVMGR_OK ) {
			/* An error of some kind during bus initialization.
			 *
			 * Child devices and their buses are not inactived
			 * directly here, instead they will all be catched by
			 * do_dev_init() and do_bus_init() by checking if
			 * parent or bridge-device failed. We know that
			 * initialization will happen later for those devices.
			 */
			goto inactivate_out;
		}
	}

	/* Bus taken into the new level */
	bus->level = level;

	/* Put bus into list of buses reached level 'level'.
	 * Put at end of bus list so that init[N+1]() calls comes 
	 * in the same order as init[N]()
	 */
	rtems_drvmgr_list_add_tail(&mgr->buses[level], bus);

	return 0;

inactivate_out:
	bus->state |= BUS_STATE_INIT_FAILED;
	bus->state |= BUS_STATE_LIST_INACTIVE;
	rtems_drvmgr_list_add_head(&mgr->buses_inactive, bus);

	DBG("do_bus_init(%d): (DEV: %s) failed\n", level, bus->dev->name);

	return 1;
}

/* Take device to initialization level 1 */
static int do_dev_init(
	struct rtems_driver_manager *mgr,
	struct rtems_drvmgr_dev_info *dev,
	int level)
{
	int (*init)(struct rtems_drvmgr_dev_info *);

	/* Try to allocate Private Device Structure for driver if driver
	 * requests for this feature.
	 */
	if ( dev->drv && dev->drv->dev_priv_size && !dev->priv ) {
		dev->priv = malloc(dev->drv->dev_priv_size);
		memset(dev->priv, 0, dev->drv->dev_priv_size);
	}

	/* If parent bus has failed during initialization,
	 * the device is not initialized further.
	 */
	if (dev->parent && (dev->parent->state & BUS_STATE_INIT_FAILED)) {
		dev->state |= DEV_STATE_DEPEND_FAILED;
		goto inactivate_out;
	}

	/* Call Driver's Init Routine */
	if ( dev->drv && (init=dev->drv->ops->init[level-1]) ) {
		/* Note: This init function may register new devices */
		if ( (dev->error=init(dev)) != DRVMGR_OK ) {
			/* An error of some kind has occured in the
			 * driver/device, the failed device is put into the
			 * inactive list, this way Init2,3 and/or 4 will not
			 * be called for this device.
			 *
			 * The device is not removed from the bus (not
			 * unregistered). The driver can be used to find
			 * device information and debugging for example even
			 * if device initialization failed.
			 *
			 * Child buses and their devices are not inactived
			 * directly here, instead they will all be catched by
			 * do_dev_init() and do_bus_init() by checking if
			 * parent or bridge-device failed. We know that
			 * initialization will happen later for those devices.
			 */
			goto inactivate_out;
		}
	}

	/* Dev taken into new level */
	dev->level = level;

	/* Put at end of device list so that init[N+1]() calls comes 
	 * in the same order as init[N]()
	 */
	rtems_drvmgr_list_add_tail(&mgr->devices[level], dev);

	return 0;

inactivate_out:
	dev->state |= DEV_STATE_INIT_FAILED;
	dev->state |= DEV_STATE_LIST_INACTIVE;
	rtems_drvmgr_list_add_head(&mgr->devices_inactive, dev);

	DBG("do_dev_init(%d): DRV: %s (DEV: %s) failed\n",
		level, dev->drv->name, dev->name);

	return 1; /* Failed to take device into requested level */
}

/* Register Root device driver */
int rtems_drvmgr_root_drv_register(struct rtems_drvmgr_drv_info *drv)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_dev_info *root;

	if ( mgr->root_drv ) {
		/* Only possible to register root device once */
		return DRVMGR_FAIL;
	}

	/* Create root device for root bus */
	rtems_drvmgr_alloc_dev(&root, 0);
	if ( root == NULL )
		return DRVMGR_FAIL;

	/* Set root device driver */
	drv->next = NULL;
	mgr->root_drv = drv;
	mgr->root_dev = root;

	/* Init root device non-NULL fields */
	root->minor_drv = -1;
	root->minor_bus = 0;
	root->businfo = mgr;
	root->name = "root bus";
	/* Custom Driver association */
	root->drv = mgr->root_drv;

	/* This registers the root device and a bus */
	rtems_drvmgr_dev_register(root);

	return DRVMGR_OK;
}

/* Register a driver */
int rtems_drvmgr_drv_register(struct rtems_drvmgr_drv_info *drv)
{
	struct rtems_driver_manager *mgr = &drv_mgr;

	drv->obj_type = DRVMGR_OBJ_DRV;

	/* Put driver into list of registered drivers */
	rtems_drvmgr_list_add_head(&mgr->drivers, drv);

	return 0;
}

/* Insert a device into a driver's device list and assign a driver minor number
 * to the device.
 *
 * The devices are ordered by their minor number (sorted linked list of devices)
 * the minor number is found by looking for a gap or at the end.
 */
static void rtems_drvmgr_insert_dev_into_drv(
	struct rtems_drvmgr_drv_info *drv,
	struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_drvmgr_dev_info *curr, **pprevnext;
	int minor;

	minor = 0;
	pprevnext = &drv->dev;
	curr = drv->dev;

	while (curr) {
		if ( minor < curr->minor_drv ) {
			/* Found a gap. Insert new device between prev 
			 * and curr. */
			break;
		}
		minor++;
		pprevnext = &curr->next_in_drv;
		curr = curr->next_in_drv;
	}
	dev->next_in_drv = curr;
	*pprevnext = dev;

	/* Set minor */
	dev->minor_drv = minor;
	drv->dev_cnt++;
}

/* Insert a device into a bus device list and assign a bus minor number to the
 * device.
 *
 * The devices are ordered by their minor number (sorted linked list of devices)
 * and by their registeration order if not using the same driver.
 *
 * The minor number is found by looking for a gap or at the end.
 */
static void rtems_drvmgr_insert_dev_into_bus(
	struct rtems_drvmgr_bus_info *bus,
	struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_drvmgr_dev_info *curr, **pprevnext;
	int minor;

	minor = 0;
	pprevnext = &bus->children;
	curr = bus->children;

	while (curr) {
		if (dev->drv && (dev->drv == curr->drv)) {
			if (minor < curr->minor_bus) {
				/* Found a gap. Insert new device between prev
				 * and curr. */
				break;
			}
			minor++;
		}
		pprevnext = &curr->next_in_bus;
		curr = curr->next_in_bus;
	}
	dev->next_in_bus = curr;
	*pprevnext = dev;

	/* Set minor. Devices without driver are given -1 */
	if (dev->drv == NULL)
		minor = -1;
	dev->minor_bus = minor;
	bus->dev_cnt++;
}

/* Try to find a driver for a device (unite a device with driver).
 * a device with a driver
 */
static struct rtems_drvmgr_drv_info *rtems_drvmgr_dev_find_drv(
		struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_drv_info *drv;

	/* Try to find a driver that can handle this device */
	for (drv = DRV_LIST_HEAD(&mgr->drivers); drv; drv = drv->next)
		if (dev->parent->ops->unite(drv, dev) == 1)
			return drv;
	return NULL;
}

/* Register a device */
int rtems_drvmgr_dev_register(struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_drv_info *drv;
	struct rtems_drvmgr_bus_info *bus = dev->parent;
	struct rtems_drvmgr_key *keys;
	struct rtems_drvmgr_list *init_list = &mgr->devices_inactive;

	DBG("DEV_REG: %s at bus \"%s\"\n", dev->name, 
		bus && bus->dev && bus->dev->name ? bus->dev->name : "UNKNOWN");

	/* Custom driver assocation? */
	if (dev->drv) {
		drv = dev->drv;
		DBG("CUSTOM ASSOCIATION (%s to %s)\n", dev->name, drv->name);
	} else {
		/* Try to find a driver that can handle this device */
		drv = rtems_drvmgr_dev_find_drv(dev);
	}

	dev->drv = drv;
	if (!drv) {
		/* No driver found that can handle this device, put into
		 * inactive list
		 */
		dev->minor_drv = -1;
		dev->state |= DEV_STATE_LIST_INACTIVE;
	} else {
		/* United device with driver.
		 * Put the device on the registered device list 
		 */
		dev->state |= DEV_STATE_UNITED;

		/* Check if user want to skip this core. This is not a
		 * normal request, however in a multi-processor system
		 * the two(or more) RTEMS instances must not use the same
		 * devices in a system, not reporting a device to
		 * it's driver will effectively accomplish this. In a
		 * non Plug & Play system one can easily avoid this
		 * problem by not report the core, but in a Plug & Play
		 * system the bus driver will report all found cores.
		 *
		 * To stop the two RTEMS instances from using the same
		 * device the user can simply define a resource entry
		 * for a certain device but set the keys field to NULL.
		 */
		if (rtems_drvmgr_keys_get(dev, &keys) == 0 && keys == NULL) {
			/* Found Driver resource entry point 
			 * for this device, it was NULL, this
			 * indicates to skip the core.
			 *
			 * We put it into the inactive list
			 * marking it as ignored.
			 */
			dev->state |= DEV_STATE_IGNORED;
		} else {
			/* Assign Driver Minor number and put into driver's
			 * device list
			 */
			rtems_drvmgr_insert_dev_into_drv(drv, dev);

			/* Just register device, it will be initialized
			 * later together with bus.
			 *
			 * At the end of the list (breadth first search)
			 */
			init_list = &mgr->devices[0];

			DBG("Registered %s (DRV: %s) on %s\n",
				dev->name, drv->name,
				bus ? bus->dev->name: "NO PARENT");
		}
	}

	rtems_drvmgr_list_add_tail(init_list, dev);

	if (bus) {
		/* Assign Bus Minor number and put into bus device list
		 * unless root device.
		 */
		rtems_drvmgr_insert_dev_into_bus(bus, dev);

		/* Trigger Device initialization if not root device and
		 * has a driver
		 */
		if (dev->drv)
			drvmgr_init_update();
	}

	return 0;
}

/* Register a bus */
int rtems_drvmgr_bus_register(struct rtems_drvmgr_bus_info *bus)
{
	struct rtems_driver_manager *mgr = &drv_mgr;

	/* Put driver into list of found buses */
	rtems_drvmgr_list_add_tail(&mgr->buses[0], bus);

	/* Take bus into level1 and so on */
	drvmgr_init_update();

	return 0;
}

/* Allocate memory for a Device structure */
int rtems_drvmgr_alloc_dev(struct rtems_drvmgr_dev_info **pdev, int extra)
{
	struct rtems_drvmgr_dev_info *dev;
	int size;

	size = ((sizeof(struct rtems_drvmgr_dev_info) + 3) & ~0x3) + extra;
	dev = (struct rtems_drvmgr_dev_info *)malloc(size);
	if ( !dev ) {
		/* Failed to allocate device structure - critical error */
		rtems_fatal_error_occurred(RTEMS_NO_MEMORY);
	}
	*pdev = dev;
	memset(dev, 0, size);
	dev->obj_type = DRVMGR_OBJ_DEV;

	return 0;
}

/* Allocate memory for a Bus structure */
int rtems_drvmgr_alloc_bus(struct rtems_drvmgr_bus_info **pbus, int extra)
{
	struct rtems_drvmgr_bus_info *bus;
	int size;

	size = ((sizeof(struct rtems_drvmgr_bus_info) + 3) & ~0x3) + extra;
	bus = (struct rtems_drvmgr_bus_info *)malloc(size);
	if ( !bus ) {
		/* Failed to allocate device structure - critical error */
		rtems_fatal_error_occurred(RTEMS_NO_MEMORY);
	}
	*pbus = bus;
	memset(bus, 0, size);
	bus->obj_type = DRVMGR_OBJ_BUS;

	return 0;
}

/* Add driver resources to a bus instance */
void rtems_drvmgr_bus_res_add(struct rtems_drvmgr_bus_info *bus,
				struct rtems_drvmgr_bus_res *bres)
{
	/* insert first in bus resource list */
	bres->next = bus->reslist;
	bus->reslist = bres;
}