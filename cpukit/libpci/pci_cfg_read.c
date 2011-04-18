/*  Read current PCI configuration that bootloader or BIOS has already setup
 *  and initialize the PCI structures.
 *
 *  COPYRIGHT (c) 2010-2011.
 *  Aeroflex Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2011-03-04, Daniel Hellstrom <daniel@gaisler.com>
 *    created
 */

#include <rtems.h>
#include <rtems/bspIo.h>
#include <pci/cfg.h>

/* Number of buses */
extern int pci_bus_cnt;

/* The Host Bridge bus */
extern struct pci_bus pci_hb;

int pci_config_read(void)
{
	pci_bus_cnt = 0;
	printk("PCI READ CONFIGURATION: NOT IMPLEMENTED\n");
	return -1;
}
