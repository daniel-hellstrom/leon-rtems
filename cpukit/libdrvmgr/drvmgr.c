/* Driver Manager Interface Implementation.
 *
 *  COPYRIGHT (c) 2009.
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

	if ( bus->ops && (init=bus->ops->init[level-1]) ) {
		/* Note: This init1 function may register new devices */
		if ( (bus->error=init(bus)) != DRVMGR_OK ) {
			/* An error of some kind during bus
			 *  initialization 
			 */
			rtems_drvmgr_list_add_head(&mgr->buses_inactive, bus);

			bus->state |= BUS_STATE_INIT_FAILED;
			bus->state |= BUS_STATE_LIST_INACTIVE;
			DBG("do_bus_init(%d): DRV: %s (DEV: %s) failed\n", 
				level, dev->drv->name, dev->name);
			return 1;
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

	/* Call Driver's Init Routine */
	if ( dev->drv && (init=dev->drv->ops->init[level-1]) ) {
		/* Note: This init function may register new devices */
		if ( (dev->error=init(dev)) != DRVMGR_OK ) {
			/* An error of some kind has occured in the
			 * driver/device, the failed device it put into
			 * a separate list, this way Init2 will not be
			 * called for this device.
			 *
			 * The device is added to the failed device list
			 * instead of beeing freed, this is for system
			 * overview and debugging.
			 */
			rtems_drvmgr_list_add_head(&mgr->devices_inactive, dev);
			dev->state |= DEV_STATE_INIT_FAILED;
			dev->state |= DEV_STATE_LIST_INACTIVE;

			if ( dev->drv->dev_priv_size && dev->priv ) {
				free(dev->priv);
				dev->priv = NULL;
			}

			DBG("do_dev_init(%d): DRV: %s (DEV: %s) failed\n",
				level, dev->drv->name, dev->name);
			return 1; /* Failed to take into */
		}
	}

	/* Dev taken into new level */
	dev->level = level;

	/* Put at end of device list so that init[N+1]() calls comes 
	 * in the same order as init[N]()
	 */
	rtems_drvmgr_list_add_tail(&mgr->devices[level], dev);

	return 0;
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

	/* Put driver into list of registered drivers */
	rtems_drvmgr_list_add_head(&mgr->drivers, drv);

	return 0;
}

/* Get a unique number for a hardware device. Devices of the same type and on the same bus
 * may not have the same number, however device of different type may. This number can be
 * used later to separate devices on a particular bus.
 *
 * This function simply returns the number of devices of the same type currently 
 * registered. If no devices of the same type has been registered 0 is returned...
 * 
 */
static int find_bus_minor(
	struct rtems_drvmgr_dev_info *dev,
	struct rtems_drvmgr_dev_info *busdev,
	void *businfo)
{
	int cnt = 0;
	int (*func)(void *a, void *b);

	if ( !busdev || !busdev->parent || !busdev->parent->ops || !busdev->parent->ops->dev_id_compare )
		return 0;
	/* Get compare function from Bus driver that the device belongs to */
	func = busdev->parent->ops->dev_id_compare;

	while ( busdev ) {
		/* Compare */
		if ( (dev != busdev) && (func(businfo, busdev->businfo) == 0) ) {
			cnt++;
		}
		busdev = busdev->next_in_bus;
	}
	return cnt;
}

/* Find next minor number for a driver, when the devices are ordered by their
 * minor number (sorted linked list of devices) the minor number is found by
 * looking for a gap or at the end.
 * 
 */
void rtems_drvmgr_insert_dev_into_drv(
	struct rtems_drvmgr_drv_info *drv,
	struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_drvmgr_dev_info *curr, *prev;
	int minor;

	minor = 0;
	prev = NULL;
	curr = drv->dev;

	while (curr) {
		if ( minor < curr->minor_drv ) {
			/* Found a gap. Insert new device between prev 
			 * and curr. */
			break;
		}
		minor++;
		prev = curr;
		curr = curr->next_in_drv;
	}

	if ( prev == NULL ) {
		/* First in list */
		dev->next_in_drv = curr;
		drv->dev = dev;
	} else {
		/* End of list or in middle */
		dev->next_in_drv = prev->next_in_drv;
		prev->next_in_drv = dev;
	}

	/* Set minor */
	dev->minor_drv = minor;
}

/* Register a device */
int rtems_drvmgr_dev_register(struct rtems_drvmgr_dev_info *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_drv_info *drv;
	struct rtems_drvmgr_bus_info *busdev = dev->parent;
	struct rtems_drvmgr_key *keys;

	/* Init dev */
	if ( busdev ) {
		/* Only root device has no parent. */
		dev->minor_bus = find_bus_minor(dev, busdev->children, dev->businfo);
		busdev->dev_cnt++;
	}

	DBG("DEV_REG: %s at bus \"%s\"\n", dev->name, 
		dev->parent && dev->parent->dev && dev->parent->dev->name ?
		dev->parent->dev->name : "UNKNOWN");

	/* Custom driver assocation? */
	if ( dev->drv ) {
		drv = dev->drv;
		DBG("CUSTOM ASSOCIATION (%s to %s)\n", dev->name, drv->name);
		goto united;
	}

	/* Try to find a driver that can handle this device */
	drv = DRV_LIST_HEAD(&mgr->drivers);
	while ( drv ) {
		if ( busdev->ops->unite(drv, dev) == 1) {
united:
			/* United device with driver 
			 * Put the device on the registered device list 
			 */
			dev->state |= DEV_STATE_UNITED;
			dev->drv = drv;

			/* Check if user want to skip this core. This is not a
			 * normal request, however in a multi-processor system
			 * the two RTEMS instances must not use the same
			 * devices in a system, not reporting a device to
			 * it's driver will effectively accomplish this. In a
			 * non Plug & Play system one can easily avoid this
			 * problem by not report the core, but in a Plug & Play
			 * system the bus driver will report all found cores.
			 *
			 * To stop the two RTEMS instances from using the same
			 * device the user can simply device a resource entry
			 * for a certain device but set the keys field to NULL.
			 */
			if ( rtems_drvmgr_keys_get(dev, &keys) == 0 ) {
				if ( keys == NULL ) {
					/* Found Driver resource entry point 
					 * for this device, it was NULL, this
					 * indicates to skip the core.
					 *
					 * We put it into the inactive list
					 * marking it as ignored.
					 */
					 dev->state |= DEV_STATE_IGNORED;
					 break;
				}
			}

			/* Assign Driver Minor number and put into driver's
			 * device list 
			 */
			rtems_drvmgr_insert_dev_into_drv(drv, dev);
			drv->dev_cnt++;

			/* Just register device, it will be initialized
			 * later together with bus.
			 *
			 * At the end of the list (breadth first search)
			 */
			rtems_drvmgr_list_add_tail(&mgr->devices[0], dev);

			DBG("Registered %s (DRV: %s) on %s\n",
				dev->name, drv->name,
				dev->parent ? dev->parent->dev->name:
				"NO PARENT" );

			/* Trigger Device initialization if not root device */
			if ( dev->parent ) {
				drvmgr_init_update();
			}

			return 0;
		}
		drv = drv->next;
	}

	/* No driver found that can handle this device, put into inactive
	 * list
	 */
	dev->minor_drv = -1;
	dev->state |= DEV_STATE_LIST_INACTIVE;
	rtems_drvmgr_list_add_tail(&mgr->devices_inactive, dev);

	return 0;
}

/* Unregister a BUS and all it's devices.
 *
 * This procedure is done twice if remove() fails. If a driver
 * depends on another driver, the first removal will fail, hopefully
 * not the second time since the other driver (the one requiring 
 * service from the first) has been removed. This will not work for
 * drivers on a remote bus however.
 */
int rtems_drvmgr_bus_unregister(struct rtems_drvmgr_bus_info *bus, int remove)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	int err, removal_iterations;
	struct rtems_drvmgr_dev_info *subdev;
	struct rtems_drvmgr_list *list;

	removal_iterations = 2;
remove_all_children:
	err = 0;
	subdev = bus->children;
	while ( subdev ) {
		if ( rtems_drvmgr_dev_unregister(subdev, remove) ) {
			/* An error occured */
			err++;
		}
		subdev = subdev->next_in_bus;
	}

	if ( err > 0 && (--removal_iterations > 0) ) {
		/* Try to remove devices once more */
		goto remove_all_children;
	} else if ( err > 0 ) {
		/* Failed to remove all children on the bus. */
		return -1;
	}

	if ( bus->ops->remove ) {
		bus->error = bus->ops->remove(bus);
		if ( bus->error != DRVMGR_OK )
			return bus->error;
	}

	bus->dev->bus = NULL;

	/* Remove bus from bus-list */
	if ( bus->state & DEV_STATE_LIST_INACTIVE ) {
		list = &mgr->buses_inactive;
	} else {
		list = &mgr->buses[bus->level];
	}
	rtems_drvmgr_list_remove(list, bus);

	if ( remove )
		free(bus);

	return 0;
}

/* Unregister device, 
 *  - let assigned driver handle deletion
 *  - remove from device list 
 *  - remove from driver list
 *  - remove from bus list
 *  - add to removed list for debugging
 */
int rtems_drvmgr_dev_unregister(struct rtems_drvmgr_dev_info *dev, int remove)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_dev_info *subdev, **pprev;
	struct rtems_drvmgr_list *list;
	int err;

	if ( dev->state & DEV_STATE_REMOVED )
		return -1; /* Already removed */

	/* Remove children if this device exports a bus of devices. All 
	 * children must be removed first as they depend upon the bus
	 * services this device provide.
	 */
	if ( dev->bus ) {
		err = rtems_drvmgr_bus_unregister(dev->bus, remove);
		if ( err )
			return err;
	}

	/* Remove device by letting assigned driver take care of hardware
	 * issues
	 */
	list = NULL;
	if ( dev->state & DEV_STATE_LIST_INACTIVE ) {
		list = &mgr->devices_inactive;
	} else {
		list = &mgr->devices[dev->level];
	}

	if ( (dev->state & DEV_STATE_UNITED) && dev->drv ) {
		if ( !dev->drv->ops->remove ) {
			/* No remove function must be considered 
			 * severe 
			 */
			return -1;
		}
		dev->error = dev->drv->ops->remove(dev);
		if ( dev->error != DRVMGR_OK) {
			/* Failed to remove device */
			return -1;
		}

		/* Delete device from driver list */
		pprev = &dev->drv->dev;
		subdev = dev->drv->dev;
		while ( subdev != dev ) {
			pprev = &subdev->next_in_drv;
			subdev = subdev->next_in_drv;
		}
		*pprev = subdev->next_in_drv;
		dev->drv->dev_cnt--;
	}

	dev->state |= DEV_STATE_REMOVED;

	/* Free Device Driver Private memory if allocated previously by
	 * Driver manager.
	 */
	if ( dev->drv->dev_priv_size && dev->priv ) {
		free(dev->priv);
		dev->priv = NULL;
	}

	/* Remove from all lists or move into inactive list */
	if ( (list != &mgr->devices_inactive) || remove ) {
		rtems_drvmgr_list_remove(list, dev);

		/* Insert into inactive list if device is not going to 
		 * be removed 
		 */
		if ( !remove ) {
			rtems_drvmgr_list_add_tail(&mgr->devices_inactive, dev);
		}
	}

	/* Remove device from parent bus list */
	pprev = &dev->parent->children;
	subdev = dev->parent->children;
	while ( subdev != dev ) {
		pprev = &subdev->next_in_bus;
		subdev = subdev->next_in_bus;
	}
	/* Device must be in list */
	*pprev = subdev->next_in_bus;
	dev->parent->dev_cnt--;

	/* All references to this device has been removed at this point 
	 * if remove is non-zero.
	 */
	if ( remove ) {
		free(dev);
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

	return 0;
}

/* Add driver resources to a bus instance */
int rtems_drvmgr_bus_res_add(struct rtems_drvmgr_bus_info *bus, struct rtems_drvmgr_drv_res *res)
{
	struct rtems_drvmgr_bus_res *node;

	node = malloc(sizeof(struct rtems_drvmgr_bus_res));
	if ( !node )
		return -1;

	node->next = bus->reslist;
	node->resource = res;
	bus->reslist = node;

	return 0;
}

int rtems_drvmgr_for_each_dev(
	struct rtems_drvmgr_list *devlist,
	unsigned int state_set_mask,
	unsigned int state_clr_mask,
	int (*func)(struct rtems_drvmgr_dev_info *dev, void *arg),
	void *arg
	)
{
	struct rtems_drvmgr_dev_info *dev;
	int ret;

	/* Get First Device */
	dev = DEV_LIST_HEAD(devlist);
	while ( dev ) {
		if ( ((state_set_mask != 0) && ((dev->state & state_set_mask) == state_set_mask)) ||
		     ((state_clr_mask != 0) && ((dev->state & state_clr_mask) == 0)) ||
		     ((state_set_mask == 0) && (state_clr_mask == 0)) )
			if ( (ret=func(dev, arg)) != 0 )
				return ret;
		dev = dev->next;
	}
	return 0;
}
