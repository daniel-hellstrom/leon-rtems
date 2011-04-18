#include <libcpu/byteorder.h>

uint16_t sparc_ld_le16(uint16_t *addr)
{
	return ld_le16(addr);
}

void sparc_st_le16(uint16_t *addr, uint16_t val)
{
	st_le16(addr, val);
}

uint32_t sparc_ld_le32(uint32_t *addr)
{
	return ld_le32(addr);
}

void sparc_st_le32(uint32_t *addr, uint32_t val)
{
	st_le32(addr, val);
}
