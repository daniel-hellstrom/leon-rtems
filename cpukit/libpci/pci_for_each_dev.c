#include <pci/cfg.h>

/* Iterate over all PCI devices on a bus (not child buses) and call func(),
 * iteration is stopped if a non-zero value is returned by func().
 */
int pci_for_each_dev(
	struct pci_bus *bus,
	int (*func)(struct pci_dev *, void *arg),
	void *arg)
{
	struct pci_dev *dev = bus->devs;
	int ret = 0;

	while (dev) {
		ret = func(dev, arg);
		if (ret)
			break;
		dev = dev->next;
	}

	return ret;
}
