/*  Driver Manager Driver Translate Interface Implementation.
 *
 *  COPYRIGHT (c) 2010.
 *  Aeroflex Gaisler AB
 *
 *  This is the part the device drivers use, the functions rely on that
 *  the parent bus driver has implemented the neccessary operations
 *  correctly.
 *
 *  The translate functions are used to translate addresses between buses
 *  for DMA cores located on a "remote" bus, or for memory-mapped obtaining
 *  an address that can be used to access an remote bus.
 *
 *  For an example, PCI I/O might be memory-mapped at the PCI Host bridge,
 *  say address 0xfff10000-0xfff1ffff is mapped to the PCI I/O address
 *  of 0x00000000-0x0000ffff. The PCI Host bridge driver may then set up
 *  a map so that a driver that get PCI address 0x100 can translate that 
 *  into 0xfff10100.
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

/* Translate Address, used by drivers when an address need to be
 * converted in order to access a remote address or for a remote
 * hardware to access (DMA) to access CPU local RAM.
 *  - from remote address to CPU local
 *  - from CPU local to remote
 */
int rtems_drvmgr_mmap_translate(
	struct rtems_drvmgr_dev_info *dev,
	int from_remote_to_cpu,
	void *src_address,
	void **dst_address)
{
	int ret;
	void *dst;
	struct rtems_drvmgr_mmap_entry *mmap;
	char *src, *local_adr, *remote_adr;

	if ( !dev || !dev->parent ) {
		ret = -1;
		dst = (void *)0xffffffff;
	} else if ( !dev->parent->mmaps ) {
		dst = src_address;
		ret = 0;
	} else {
		ret = -1;
		dst = (void *)0xffffffff;
		src = src_address;
		mmap = dev->parent->mmaps;
		while ( mmap->map_size != 0 ) {

			if ( from_remote_to_cpu ) {
				/* Translate from remote address into 
				 * CPU address
				 */
				local_adr = (char *)mmap->remote_adr;
				remote_adr = (char *)mmap->local_adr;
			} else {
				/* Translate from CPU address into remote
				 * address
				 */
				local_adr = (char *)mmap->local_adr;
				remote_adr = (char *)mmap->remote_adr;
			}

			if ((src >= local_adr) &&
			    (src <= (local_adr + (mmap->map_size - 1)))) {
				dst = (void *)((src - local_adr) + remote_adr);
				break;
			}
			mmap++;
		}
	}
	if ( dst_address )
		*dst_address = dst;
	return ret;
}
