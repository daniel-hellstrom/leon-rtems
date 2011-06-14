/*  SpaceWire RMAP AMBA Plug & Play bus driver interface.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __AMBAPP_BUS_RMAP_H__
#define __AMBAPP_BUS_RMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

extern void ambapp_rmap_register(void);

/* Allocate memory from the SpaceWire target's memory */
extern void *ambapp_rmap_partition_memalign(
	struct drvmgr_dev *dev,
	int partition,
	size_t boundary,
	size_t size);

/* Register a partition of memory in the SpaceWire target */
extern int ambapp_rmap_partition_create(
	struct drvmgr_dev *dev,
	int partition,
	unsigned int start,
	size_t size);

#ifdef __cplusplus
}
#endif

#endif
