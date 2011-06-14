#include <stdlib.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/drvmgr_list.h>
#include "drvmgr_internal.h"

/* Unregister a BUS and all it's devices.
 *
 * This procedure is done twice if remove() fails. If a driver
 * depends on another driver, the first removal may fail, hopefully
 * not the second time since the other driver (the one requiring 
 * service from the first) has been removed.
 */
int drvmgr_bus_unregister(struct drvmgr_bus *bus)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	int err, removal_iterations;
	struct drvmgr_dev *subdev, *subdev_next;
	struct drvmgr_list *list;

	removal_iterations = 2;
remove_all_children:
	err = 0;
	subdev = bus->children;
	while (subdev) {
		subdev_next = subdev->next_in_bus;
		if (drvmgr_dev_unregister(subdev) != DRVMGR_OK) {
			/* An error occured */
			err++;
		}
		subdev = subdev_next;
	}

	if (err > 0) {
		/* Try to remove devices once more, or fail */
		if (--removal_iterations > 0)
			goto remove_all_children;
		else
			return -1; /* Failed to remove all children */
	}

	if (bus->ops->remove) {
		bus->error = bus->ops->remove(bus);
		if (bus->error != DRVMGR_OK)
			return bus->error;
	}
	bus->dev->bus = NULL;

	/* Remove bus from bus-list */
	if (bus->state & BUS_STATE_LIST_INACTIVE) {
		list = &mgr->buses_inactive;
	} else {
		list = &mgr->buses[bus->level];
	}
	drvmgr_list_remove(list, bus);

	/* All references to this bus has been removed at this point */
	free(bus);

	return DRVMGR_OK;
}

/* Unregister device,
 *  - let assigned driver handle deletion
 *  - remove from device list
 *  - remove from driver list
 *  - remove from bus list
 */
int drvmgr_dev_unregister(struct drvmgr_dev *dev)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_dev *subdev, **pprev;
	struct drvmgr_list *list;
	int err;

	/* Remove children if this device exports a bus of devices. All 
	 * children must be removed first as they depend upon the bus
	 * services this device provide.
	 */
	if (dev->bus) {
		err = drvmgr_bus_unregister(dev->bus);
		if (err != DRVMGR_OK)
			return err;
	}

	/* Remove device by letting assigned driver take care of hardware
	 * issues
	 */
	if (dev->drv) {
		if (!dev->drv->ops->remove) {
			/* No remove function is considered severe when someone
			 * is trying to remove the device
			 */
			return -1;
		}
		dev->error = dev->drv->ops->remove(dev);
		if (dev->error != DRVMGR_OK)
			return -1;

		/* Delete device from driver's device list */
		pprev = &dev->drv->dev;
		subdev = dev->drv->dev;
		while (subdev != dev) {
			pprev = &subdev->next_in_drv;
			subdev = subdev->next_in_drv;
		}
		*pprev = subdev->next_in_drv;
		dev->drv->dev_cnt--;
	}

	/* Free Device Driver Private memory if allocated previously by
	 * Driver manager.
	 */
	if (dev->drv->dev_priv_size && dev->priv)
		free(dev->priv);

	/* Find list the device is located at and remove it */
	if (dev->state & DEV_STATE_LIST_INACTIVE) {
		list = &mgr->devices_inactive;
	} else {
		list = &mgr->devices[dev->level];
	}
	drvmgr_list_remove(list, dev);

	/* Remove device from parent bus list (no check if dev not in list) */
	pprev = &dev->parent->children;
	subdev = dev->parent->children;
	while ( subdev != dev ) {
		pprev = &subdev->next_in_bus;
		subdev = subdev->next_in_bus;
	}
	*pprev = subdev->next_in_bus;
	dev->parent->dev_cnt--;

	/* All references to this device has been removed at this point */
	free(dev);

	return DRVMGR_OK;
}
