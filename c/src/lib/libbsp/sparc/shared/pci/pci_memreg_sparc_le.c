/* Registers-over-Memory Space - SPARC Little endian PCI bus definitions */

#include <pci.h>
#include <libcpu/access.h>

struct pci_memreg_ops pci_memreg_sparc_le_ops = {
	.ld8    = sparc_ld8,
	.st8    = sparc_st8,

	.ld_le16 = sparc_ld_le16,
	.st_le16 = sparc_st_le16,
	.ld_be16 = sparc_ld_be16,
	.st_be16 = sparc_st_be16,

	.ld_le32 = sparc_ld_le32,
	.st_le32 = sparc_st_le32,
	.ld_be32 = sparc_ld_be32,
	.st_be32 = sparc_st_be16,
};
