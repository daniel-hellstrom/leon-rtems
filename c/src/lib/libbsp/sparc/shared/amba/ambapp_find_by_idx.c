/*
 *  AMBA Plug & Play routines
 *
 *  COPYRIGHT (c) 2011
 *  Aeroflex Gaisler
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id: ambapp_find_by_idx.c,v
 *
 */

#include <ambapp.h>

/* AMBAPP helper routine to find a device by index. The index input into this
 * function determines which AMBA-PnP index within a bus plug&play information,
 * that is pretty 
 */
int ambapp_find_by_idx(struct ambapp_dev *dev, int index, void *pcount)
{
	int *pi = pcount;

	if (pi) {
		if (*pi-- == 0)
			return (int)dev;
		else
			return 0;
	} else {
		/* Satisfied with first matching device, stop search */
		return (int)dev;
	}
}
