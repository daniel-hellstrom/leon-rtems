#include <pci.h>

uint8_t pci_mem_ld8(uint8_t *adr)
{
	return *adr;
}

void pci_mem_st8(uint8_t *adr, uint8_t data)
{
	*adr = data;
}
