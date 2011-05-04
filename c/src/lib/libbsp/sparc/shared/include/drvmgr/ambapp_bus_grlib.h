/*  LEON3 GRLIB AMBA Plug & Play bus driver interface.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  This is driver is a wrapper for the general AMBA Plug & Play bus
 *  driver. This is the root bus driver for GRLIB systems.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *
 */

#ifndef __AMBAPP_BUS_GRLIB_H__
#define __AMBAPP_BUS_GRLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

struct grlib_config {
	struct ambapp_bus		*abus;
	struct rtems_drvmgr_bus_res	*resources;
};

int drv_mgr_grlib_init(struct grlib_config *config);

#ifdef __cplusplus
}
#endif

#endif
