#include <pci/cfg.h>

int pci_for_each_dev(
	int (*func)(struct pci_dev *, void *arg),
	void *arg)
{
	return pci_for_each_child(&pci_hb, func, arg, SEARCH_DEPTH);
}
