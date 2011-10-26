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
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

struct rtems_driver_manager drv_mgr = {
	.level =		0,
	.initializing_objs =	0,
	.lock =                 0,
	.root_dev =		{0},
	.root_drv =		NULL,

	.drivers =	LIST_INITIALIZER(struct drvmgr_drv, next),

	.buses = {
		LIST_INITIALIZER(struct drvmgr_bus, next),
		LIST_INITIALIZER(struct drvmgr_bus, next),
		LIST_INITIALIZER(struct drvmgr_bus, next),
		LIST_INITIALIZER(struct drvmgr_bus, next),
		LIST_INITIALIZER(struct drvmgr_bus, next),
	},
	.buses_inactive =	LIST_INITIALIZER(struct drvmgr_bus, next),

	.devices = {
		LIST_INITIALIZER(struct drvmgr_dev, next),
		LIST_INITIALIZER(struct drvmgr_dev, next),
		LIST_INITIALIZER(struct drvmgr_dev, next),
		LIST_INITIALIZER(struct drvmgr_dev, next),
		LIST_INITIALIZER(struct drvmgr_dev, next),
	},
	.devices_inactive =	LIST_INITIALIZER(struct drvmgr_dev, next),
};

static int do_bus_init(
	struct rtems_driver_manager *mgr,
	struct drvmgr_bus *bus,
	int level);
static int do_dev_init(
	struct rtems_driver_manager *mgr,
	struct drvmgr_dev *dev,
	int level);

/* DRIVER MANAGER */

void _DRV_Manager_init_level(int level)
{
	struct rtems_driver_manager *mgr = &drv_mgr;

	if (mgr->level >= level)
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
	struct drvmgr_drv_reg_func *drvreg;

	/* drv_mgr is already initialized statically by compiler except
	 * the lock
	 */
	DRVMGR_LOCK_INIT();

	/* Call driver register functions. */
	drvreg = &drvmgr_drivers[0];
	while (drvreg->drv_reg) {
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
	struct drvmgr_bus *bus;
	struct drvmgr_dev *dev;
	int bus_might_been_registered;
	int level;

	/* "Lock" to make sure we don't use up the stack and that the lists
	 * remain consistent.
	 */
	DRVMGR_LOCK_WRITE();
	if (mgr->initializing_objs || (mgr->level == 0))
		goto out;
	mgr->initializing_objs = 1;

init_registered_buses:
	/* Take all buses and devices ready into the same stage
	 * as the driver manager global level.
	 */
	for (level = 0; level < mgr->level; level++) {

		bus_might_been_registered = 0;

		/* Take buses into next level */

		while ((bus = BUS_LIST_HEAD(&mgr->buses[level])) != NULL) {

			/* Remove first in the list (will be inserted in
			 * appropriate list by do_bus_init())
			 */
			drvmgr_list_remove_head(&mgr->buses[level]);

			DRVMGR_UNLOCK();

			/* Initialize Bus, this will register devices on
			 * the bus. Take bus into next level.
			 */
			do_bus_init(mgr, bus, level+1);

			DRVMGR_LOCK_WRITE();
		}

		/* Take devices into next level */
		while ((dev = DEV_LIST_HEAD(&mgr->devices[level])) != NULL) {

			/* Always process first in list */
			dev = DEV_LIST_HEAD(&mgr->devices[level]);

			/* Remove first in the list (will be inserted in
			 * appropriate list by do_dev_init())
			 */
			drvmgr_list_remove_head(&mgr->devices[level]);

			DRVMGR_UNLOCK();

			/* Initialize Device, this may register a new bus */
			do_dev_init(mgr, dev, level+1);

			DRVMGR_LOCK_WRITE();

			bus_might_been_registered = 1;
		}

		/* Make sure all buses registered and ready are taken at
		 * the same time into init level N.
		 */
		if (bus_might_been_registered)
			goto init_registered_buses;
	}

	/* Release bus/device initialization "Lock" */
	mgr->initializing_objs = 0;

out:
	DRVMGR_UNLOCK();
}

/* Take bus into next level */
static int do_bus_init(
	struct rtems_driver_manager *mgr,
	struct drvmgr_bus *bus,
	int level)
{
	int (*init)(struct drvmgr_bus *);

	/* If bridge device has failed during initialization, the bus is not
	 * initialized further.
	 */
	if (bus->dev->state & DEV_STATE_INIT_FAILED) {
		bus->state |= BUS_STATE_DEPEND_FAILED;
		goto inactivate_out;
	}

	if (bus->ops && (init = bus->ops->init[level-1])) {
		/* Note: This init1 function may register new devices */
		bus->error = init(bus);
		if (bus->error != DRVMGR_OK) {
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

	DRVMGR_LOCK_WRITE();

	/* Bus taken into the new level */
	bus->level = level;

	/* Put bus into list of buses reached level 'level'.
	 * Put at end of bus list so that init[N+1]() calls comes
	 * in the same order as init[N]()
	 */
	drvmgr_list_add_tail(&mgr->buses[level], bus);

	DRVMGR_UNLOCK();

	return 0;

inactivate_out:
	DRVMGR_LOCK_WRITE();
	bus->state |= BUS_STATE_INIT_FAILED;
	bus->state |= BUS_STATE_LIST_INACTIVE;
	drvmgr_list_add_head(&mgr->buses_inactive, bus);
	DRVMGR_UNLOCK();

	DBG("do_bus_init(%d): (DEV: %s) failed\n", level, bus->dev->name);

	return 1;
}

/* Take device to initialization level 1 */
static int do_dev_init(
	struct rtems_driver_manager *mgr,
	struct drvmgr_dev *dev,
	int level)
{
	int (*init)(struct drvmgr_dev *);

	/* Try to allocate Private Device Structure for driver if driver
	 * requests for this feature.
	 */
	if (dev->drv && dev->drv->dev_priv_size && !dev->priv) {
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
	if (dev->drv && (init = dev->drv->ops->init[level-1])) {
		/* Note: This init function may register new devices */
		dev->error = init(dev);
		if (dev->error != DRVMGR_OK) {
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

	DRVMGR_LOCK_WRITE();
	/* Dev taken into new level */
	dev->level = level;

	/* Put at end of device list so that init[N+1]() calls comes
	 * in the same order as init[N]()
	 */
	drvmgr_list_add_tail(&mgr->devices[level], dev);
	DRVMGR_UNLOCK();

	return 0;

inactivate_out:
	DRVMGR_LOCK_WRITE();
	dev->state |= DEV_STATE_INIT_FAILED;
	dev->state |= DEV_STATE_LIST_INACTIVE;
	drvmgr_list_add_head(&mgr->devices_inactive, dev);
	DRVMGR_UNLOCK();

	DBG("do_dev_init(%d): DRV: %s (DEV: %s) failed\n",
		level, dev->drv->name, dev->name);

	return 1; /* Failed to take device into requested level */
}

/* Register Root device driver */
int drvmgr_root_drv_register(struct drvmgr_drv *drv)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_dev *root = &mgr->root_dev;

	if (mgr->root_drv) {
		/* Only possible to register root device once */
		return DRVMGR_FAIL;
	}

	/* Set root device driver */
	drv->next = NULL;
	mgr->root_drv = drv;

	/* Init root device non-NULL fields */
	root->minor_drv = -1;
	root->minor_bus = 0;
	root->businfo = mgr;
	root->name = "root bus";
	/* Custom Driver association */
	root->drv = mgr->root_drv;

	/* This registers the root device and a bus */
	drvmgr_dev_register(root);

	return DRVMGR_OK;
}

/* Register a driver */
int drvmgr_drv_register(struct drvmgr_drv *drv)
{
	struct rtems_driver_manager *mgr = &drv_mgr;

	drv->obj_type = DRVMGR_OBJ_DRV;

	/* Put driver into list of registered drivers */
	drvmgr_list_add_head(&mgr->drivers, drv);

	/* TODO: we could scan for devices that this new driver has support
	 *       for. However, at this stage we assume that all drivers are
	 *       registered before devices are registered.
	 *
	 * LOCK: From the same assumsion locking the driver list is not needed
	 *       either.
	 */

	return 0;
}

/* Insert a device into a driver's device list and assign a driver minor number
 * to the device.
 *
 * The devices are ordered by their minor number (sorted linked list of devices)
 * the minor number is found by looking for a gap or at the end.
 */
static void drvmgr_insert_dev_into_drv(
	struct drvmgr_drv *drv,
	struct drvmgr_dev *dev)
{
	struct drvmgr_dev *curr, **pprevnext;
	int minor;

	DRVMGR_LOCK_WRITE();

	minor = 0;
	pprevnext = &drv->dev;
	curr = drv->dev;

	while (curr) {
		if (minor < curr->minor_drv) {
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

	DRVMGR_UNLOCK();
}

/* Insert a device into a bus device list and assign a bus minor number to the
 * device.
 *
 * The devices are ordered by their minor number (sorted linked list of devices)
 * and by their registeration order if not using the same driver.
 *
 * The minor number is found by looking for a gap or at the end.
 */
static void drvmgr_insert_dev_into_bus(
	struct drvmgr_bus *bus,
	struct drvmgr_dev *dev)
{
	struct drvmgr_dev *curr, **pprevnext;
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
static struct drvmgr_drv *drvmgr_dev_find_drv(
		struct drvmgr_dev *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_drv *drv;

	/* NOTE: No locking is needed here since Driver list is supposed to be
	 *       initialized once during startup, we treat it as a static
	 *       read-only list
	 */

	/* Try to find a driver that can handle this device */
	for (drv = DRV_LIST_HEAD(&mgr->drivers); drv; drv = drv->next)
		if (dev->parent->ops->unite(drv, dev) == 1)
			break;

	return drv;
}

/* Register a device */
int drvmgr_dev_register(struct drvmgr_dev *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_drv *drv;
	struct drvmgr_bus *bus = dev->parent;
	struct drvmgr_key *keys;
	struct drvmgr_list *init_list = &mgr->devices_inactive;

	DBG("DEV_REG: %s at bus \"%s\"\n", dev->name,
		bus && bus->dev && bus->dev->name ? bus->dev->name : "UNKNOWN");

	/* Custom driver assocation? */
	if (dev->drv) {
		drv = dev->drv;
		DBG("CUSTOM ASSOCIATION (%s to %s)\n", dev->name, drv->name);
	} else {
		/* Try to find a driver that can handle this device */
		drv = drvmgr_dev_find_drv(dev);
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
		if (drvmgr_keys_get(dev, &keys) == 0 && keys == NULL) {
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
			drvmgr_insert_dev_into_drv(drv, dev);

			/* Just register device, it will be initialized
			 * later together with bus.
			 *
			 * At the end of the list (breadth first search)
			 */
			init_list = &mgr->devices[0];

			DBG("Registered %s (DRV: %s) on %s\n",
				dev->name, drv->name,
				bus ? bus->dev->name : "NO PARENT");
		}
	}

	DRVMGR_LOCK_WRITE();

	drvmgr_list_add_tail(init_list, dev);

	if (bus) {
		/* Assign Bus Minor number and put into bus device list
		 * unless root device.
		 */
		drvmgr_insert_dev_into_bus(bus, dev);

		DRVMGR_UNLOCK();

		/* Trigger Device initialization if not root device and
		 * has a driver
		 */
		if (dev->drv)
			drvmgr_init_update();
	} else
		DRVMGR_UNLOCK();

	return 0;
}

/* Register a bus */
int drvmgr_bus_register(struct drvmgr_bus *bus)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_bus *bus_up;

	/* Get bus architecture depth - the distance from root bus */
	bus->depth = 0;
	bus_up = bus->dev->parent;
	while (bus_up) {
		bus->depth++;
		bus_up = bus_up->dev->parent;
	}

	DRVMGR_LOCK_WRITE();

	/* Put driver into list of found buses */
	drvmgr_list_add_tail(&mgr->buses[0], bus);

	DRVMGR_UNLOCK();

	/* Take bus into level1 and so on */
	drvmgr_init_update();

	return 0;
}

/* Allocate memory for a Device structure */
int drvmgr_alloc_dev(struct drvmgr_dev **pdev, int extra)
{
	struct drvmgr_dev *dev;
	int size;

	size = ((sizeof(struct drvmgr_dev) + 3) & ~0x3) + extra;
	dev = (struct drvmgr_dev *)malloc(size);
	if (!dev) {
		/* Failed to allocate device structure - critical error */
		rtems_fatal_error_occurred(RTEMS_NO_MEMORY);
	}
	*pdev = dev;
	memset(dev, 0, size);
	dev->obj_type = DRVMGR_OBJ_DEV;

	return 0;
}

/* Allocate memory for a Bus structure */
int drvmgr_alloc_bus(struct drvmgr_bus **pbus, int extra)
{
	struct drvmgr_bus *bus;
	int size;

	size = ((sizeof(struct drvmgr_bus) + 3) & ~0x3) + extra;
	bus = (struct drvmgr_bus *)malloc(size);
	if (!bus) {
		/* Failed to allocate device structure - critical error */
		rtems_fatal_error_occurred(RTEMS_NO_MEMORY);
	}
	*pbus = bus;
	memset(bus, 0, size);
	bus->obj_type = DRVMGR_OBJ_BUS;

	return 0;
}

/* Add driver resources to a bus instance */
void drvmgr_bus_res_add(struct drvmgr_bus *bus,
				struct drvmgr_bus_res *bres)
{
	/* insert first in bus resource list. Locking isn't needed since
	 * resources can only be added before resource requests are made.
	 * When bus has been registered resources are considered a read-only
	 * tree.
	 */
	bres->next = bus->reslist;
	bus->reslist = bres;
}
