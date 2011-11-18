/*  Driver Manager Information printing Interface Implementation.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler AB
 *
 *  These function simply print stuff from about the driver manager.
 *  What devices were found and were united with a driver, the Bus 
 *  topology, memory taken, etc.
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
#include "drvmgr_internal.h"

void rtems_drvmgr_dev_print(struct rtems_drvmgr_dev_info *dev, char *prefix, int print_bus)
{
	struct rtems_drvmgr_dev_info *busdev;
	char bus_prefix[16];

	printf(" %s|-> DEV: 0x%x %s\n", prefix, (unsigned int)dev, dev->name);
	if ( (print_bus == 0) || !dev->bus )
		return;
	/* This device provides a bus, print the bus */
	strcpy(bus_prefix, prefix);
	strcat(bus_prefix, "  ");
	busdev = dev->bus->children;
	while ( busdev ) {
		rtems_drvmgr_dev_print(busdev, bus_prefix, print_bus);
		busdev = busdev->next_in_bus;
	}
}

void rtems_drvmgr_print_drvs(int show_devs)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_drv_info *drv;
	struct rtems_drvmgr_dev_info *dev;

	/* Print Drivers */
	printf(" --- DRIVERS %s---\n", show_devs ? "AND THEIR DEVICES " : "");
	drv = DRV_LIST_HEAD(&mgr->drivers);
	while ( drv ) {
		printf(" DRV 0x%x: %s\n", (unsigned int)drv, drv->name ? drv->name : "NO_NAME");
		/* Print devices united with this driver */
		if ( show_devs ) {
			dev = drv->dev;
			while ( dev ) {
				rtems_drvmgr_dev_print(dev, "   ", 0);
				dev = dev->next_in_drv;
			}
		}

		drv = drv->next;
	}
	printf("\n\n");
}

static int print_dev_found(struct rtems_drvmgr_dev_info *dev, void *arg)
{
	char **pparg = arg;

	if ( pparg && *pparg ) {
		printf(*pparg);
		*pparg = NULL;
	}

	printf(" DEV 0x%x: %s on bus 0x%x (%d,%d)\n", (unsigned int)dev, dev->name ? dev->name : "NO_NAME", (unsigned int)dev->parent, dev->minor_drv, dev->minor_bus);

	return 0; /* Continue to next device */
}

void rtems_drvmgr_print_devs(unsigned int options)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	char *parg;

	/* Print Drivers */
	if ( options & DRV_MGR_PRINT_DEVS_ASSIGNED ) {
		parg = " --- DEVICES ASSIGNED TO DRIVER ---\n";
		rtems_drvmgr_for_each_dev(&mgr->devices[DRVMGR_LEVEL_MAX],
				DEV_STATE_UNITED, 0, print_dev_found, &parg);
		if ( parg != NULL )
			printf("\nNO DEVICES WERE ASSIGNED A DRIVER\n");
	}

	if ( options & DRV_MGR_PRINT_DEVS_UNASSIGNED ) {
		parg = "\n --- DEVICES WITHOUT DRIVER ---\n";
		rtems_drvmgr_for_each_dev(&mgr->devices_inactive, 0,
			DEV_STATE_UNITED, print_dev_found, &parg);
		if ( parg != NULL )
			printf("\nNO DEVICES WERE WITHOUT DRIVER\n");
	}

	if ( options & DRV_MGR_PRINT_DEVS_FAILED ) {
		parg = "\n --- DEVICES FAILED TO INITIALIZE ---\n";
		rtems_drvmgr_for_each_dev(&mgr->devices_inactive,
			DEV_STATE_INIT_FAILED, 0, print_dev_found, &parg);
		if ( parg != NULL )
			printf("\nNO DEVICES FAILED TO INITIALIZE\n");
	}

	if ( options & DRV_MGR_PRINT_DEVS_REMOVED ) {
		parg = "\n --- DEVICES REMOVED ---\n";
		rtems_drvmgr_for_each_dev(&mgr->devices_inactive,
			DEV_STATE_REMOVED, 0, print_dev_found, &parg);
		if ( parg != NULL )
			printf("\nNO DEVICES WERE REMOVED\n");
	}

	if ( options & DRV_MGR_PRINT_DEVS_IGNORED ) {
		parg = "\n --- DEVICES IGNORED ---\n";
		rtems_drvmgr_for_each_dev(&mgr->devices_inactive,
			DEV_STATE_IGNORED, 0, print_dev_found, &parg);
		if ( parg != NULL )
			printf("\nNO DEVICES WERE IGNORED\n");
	}

	printf("\n\n");
}

void rtems_drvmgr_print_topo()
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_dev_info *dev;

	/* Print Bus topology */
	printf(" --- BUS TOPOLOGY ---\n");
	dev = mgr->root_dev;
	rtems_drvmgr_dev_print(dev, "", 1);
	printf("\n\n");
}

void rtems_drvmgr_print_buses()
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_bus_info *bus;

	/* Print Drivers */
	printf(" --- BUSES ---\n");
	bus = BUS_LIST_HEAD(&mgr->buses[DRVMGR_LEVEL_MAX]);
	while ( bus ) {
		printf(" BUS 0x%x (0x%x): %s on bus 0x%x\n",
			(unsigned int)bus, (unsigned int)bus->dev,
			bus->dev->name ? bus->dev->name : "NO_NAME",
			(unsigned int)bus->dev->parent);
		bus = bus->next;
	}
	printf("\n\n");
}

/* Print the memory usage */
void rtems_drvmgr_print_mem(void)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct rtems_drvmgr_bus_info *bus;
	struct rtems_drvmgr_dev_info *dev;
	struct rtems_drvmgr_drv_info *drv;

	struct rtems_drvmgr_bus_res *node;
	struct rtems_drvmgr_drv_res *res;
	struct rtems_drvmgr_key *key;

	unsigned int busmem = 0;
	unsigned int devmem = 0;
	unsigned int drvmem = 0;
	unsigned int resmem = 0;

	bus = BUS_LIST_HEAD(&mgr->buses[DRVMGR_LEVEL_MAX]);
	while ( bus ) {
		busmem += sizeof(struct rtems_drvmgr_bus_info);

		/* Get size of resources on this bus */
		node = bus->reslist;
		while ( node ) {
			resmem += sizeof(struct rtems_drvmgr_bus_res);

			res = node->resource;
			while ( res->keys ) {
				resmem += sizeof(struct rtems_drvmgr_drv_res);

				key = res->keys;
				while ( key->key_type != KEY_TYPE_NONE ) {
					resmem += sizeof
						(struct rtems_drvmgr_key);
					key++;
				}
				resmem += sizeof(struct rtems_drvmgr_key);
				res++;
			}

			node = node->next;
		}

		bus = bus->next;
	}

	drv = DRV_LIST_HEAD(&mgr->drivers);
	while ( drv ) {
		drvmem += sizeof(struct rtems_drvmgr_drv_info);
		drv = drv->next;
	}

	dev = DEV_LIST_HEAD(&mgr->devices[DRVMGR_LEVEL_MAX]);
	while ( dev ) {
		devmem += sizeof(struct rtems_drvmgr_dev_info);
		dev = dev->next;
	}

	printf(" --- MEMORY USAGE ---\n");
	printf(" BUS:   %d bytes\n", busmem);
	printf(" DRV:   %d bytes\n", drvmem);
	printf(" DEV:   %d bytes\n", devmem);
	printf(" RES:   %d bytes\n", resmem);
	printf(" TOTAL: %d bytes\n", busmem+drvmem+devmem+resmem);
	printf("\n\n");
}
