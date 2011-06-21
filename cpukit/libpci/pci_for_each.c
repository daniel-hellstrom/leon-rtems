/*  PCI Help Function, Iterate over all PCI devices
 *
 *  COPYRIGHT (c) 2010-2011.
 *  Aeroflex Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2011-02-11, Daniel Hellstrom <daniel@gaisler.com>
 *    created from pci_bus.c
 */

#include <pci.h>
#include <pci/access.h>

/*#define DEBUG*/

#ifdef DEBUG
#include <rtems/bspIo.h>
#define DBG(args...) printk(args)
#else
#define DBG(args...)
#endif

int pci_for_each(int (*func)(pci_dev_t, void*), void *arg)
{
	uint32_t id;
	uint8_t hd;
	int bus, dev, fun, result, fail;
	int maxbus = pci_bus_count();
	pci_dev_t pcidev;

	for (bus = 0; bus < maxbus ; bus++) {
		dev = 0;
		if (bus == 0)
			dev = 1; /* Skip PCI host bridge */
		for (; dev < PCI_MAX_DEVICES; dev++) {
			pcidev = PCI_DEV(bus, dev, 0);

			for (fun = 0; fun < PCI_MAX_FUNCTIONS; fun++, pcidev++) {
				fail = pci_cfg_r32(pcidev, PCI_VENDOR_ID, &id);
				if (fail || (0xffffffff == id) || (0 == id)) {
					if (fun == 0)
						break;
					else
						continue;
				}

				DBG("pcibus_for_each: found 0x%08lx at"
				    " %d/%d/%d\n", id, bus, dev, fun);
				result = func(pcidev, arg);
				if (result != 0)
					return result; /* Stopped */

				/* Stop if not a multi-function device */
				if (fun == 0) {
					pci_cfg_r8(pcidev, PCI_HEADER_TYPE,
							&hd);
					if ((hd & PCI_MULTI_FUNCTION) == 0)
						break;
				}
			}
		}
	}

	return 0; /* scanned all */
}
