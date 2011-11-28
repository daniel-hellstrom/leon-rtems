/*  PCI IRQ Library
 *
 *  COPYRIGHT (c) 2010-2011.
 *  Aeroflex Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2011-02-11, Daniel Hellstrom <daniel@gaisler.com>
 *    created
 */

#include <pci.h>
#include <pci/access.h>

int pci_dev_irq(pci_dev_t dev)
{
	uint8_t irq_line;
	pci_cfg_r8(dev, PCI_INTERRUPT_LINE, &irq_line);
	return irq_line;
}
