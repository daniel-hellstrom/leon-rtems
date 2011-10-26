/*  Driver Manager Driver Translate Interface Implementation
 *
 *  COPYRIGHT (c) 2010.
 *  Aeroflex Gaisler AB
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  Used by device drivers. The functions rely on that the parent bus driver
 *  has implemented the neccessary operations correctly.
 *
 *  The translate functions are used to translate addresses between buses
 *  for DMA cores located on a "remote" bus, or for memory-mapped obtaining
 *  an address that can be used to access an remote bus.
 *
 *  For example, PCI I/O might be memory-mapped at the PCI Host bridge,
 *  say address 0xfff10000-0xfff1ffff is mapped to the PCI I/O address
 *  of 0x00000000-0x0000ffff. The PCI Host bridge driver may then set up
 *  a map so that a driver that get PCI address 0x100 can translate that
 *  into 0xfff10100.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drvmgr/drvmgr.h>
#include "drvmgr_internal.h"

int drvmgr_translate_bus(
	struct drvmgr_bus *from,
	struct drvmgr_bus *to,
	int reverse,
	void *src_address,
	void **dst_address)
{
	struct drvmgr_bus *path[16];
	int upstream, ret, depth, i;
	void *dst, *from_adr, *to_adr;
	struct drvmgr_map_entry *map;
	struct drvmgr_bus *bus;

	dst = src_address;
	ret = 0;

	if (from == to) /* no need traslating addresses when on same bus */
		goto out;

	if (from->depth > to->depth) {
		/* up-streams */
		upstream = 1;
		depth = from->depth - to->depth;
		if (depth >= 16)
			return -1; /* Does not support such a big depth */

		/* Intensionally we skip the last bus since its bridge is
		 * not used in this translation
		 */
		path[0] = from;
		for (i=1; i < depth; i++)
			path[i] = path[i-1]->dev->parent;
	} else {
		/* down-streams */
		upstream = 0;
		depth = to->depth - from->depth;
		if (depth >= 16)
			return -1; /* Does not support such a big depth */

		/* Intensionally we skip the last bus since its bridge is
		 * not used in this translation
		 */
		path[depth-1] = to;
		for (i=depth-1; i > 0; i--)
			path[i-1] = path[i]->dev->parent;
	}

	/* Translate address */
	for (i=0; i < depth && ret == 0; i++) {
		bus = path[i];

		if ((upstream && reverse) || (!upstream && !reverse))
			map = bus->maps_down;
		else
			map = bus->maps_up;

		if (map == NULL)
			continue; /* No translation needed - 1:1 mapping */

		ret = -1;

		if (map == DRVMGR_TRANSLATE_NO_BRIDGE)
			break; /* No bridge interface in this direction */

		while (map->size != 0) {
			if (reverse) {
				/* Opposite direction */
				from_adr = map->to_adr;
				to_adr = map->from_adr;
			} else {
				from_adr = map->from_adr;
				to_adr = map->to_adr;
			}

			if ((dst >= from_adr) &&
			    (dst <= (from_adr + (map->size - 1)))) {
				dst = (dst - from_adr) + to_adr;
				ret = 0;
				break;
			}
			map++;
		}
	}

out:
	if (dst_address)
		*dst_address = dst;

	return ret;
}

/* Translate Address, used by drivers when an address need to be
 * converted in order to access a remote address or for a remote
 * hardware to access (DMA) to access CPU local RAM.
 *  - from remote address to CPU local
 *  - from CPU local to remote
 */
int drvmgr_translate(
	struct drvmgr_dev *dev,
	int cpu_addresses,
	int upstream,
	void *src_address,
	void **dst_address)
{
	struct drvmgr_bus *to, *from;
	int rev;

	if (upstream) {
		from = dev->parent;
		to = drv_mgr.root_dev.bus;
	} else {
		from = drv_mgr.root_dev.bus;
		to = dev->parent;
	}

	if (cpu_addresses)
		rev = upstream;
	else
		rev = !upstream;

	return drvmgr_translate_bus(from, to, rev, src_address, dst_address);
}
