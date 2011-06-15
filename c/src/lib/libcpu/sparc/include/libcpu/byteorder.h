/*
 * byteorder.h  - Endian conversion for SPARC. SPARC is big endian only.
 *
 * Copyright (C) 2011  Daniel Hellstrom (daniel@gaisler.com)
 *
 */

#ifndef _LIBCPU_BYTEORDER_H
#define _LIBCPU_BYTEORDER_H

#include <rtems/system.h>
#include <rtems/score/cpu.h>

#ifdef __cplusplus
extern "C" {
#endif

RTEMS_INLINE_ROUTINE uint16_t ld_le16(volatile uint16_t *addr)
{
	return CPU_swap_u16(*addr);
}

RTEMS_INLINE_ROUTINE void st_le16(volatile uint16_t *addr, uint16_t val)
{
	*addr = CPU_swap_u16(val);
}

RTEMS_INLINE_ROUTINE uint32_t ld_le32(volatile uint32_t *addr)
{
	return CPU_swap_u32(*addr);
}

RTEMS_INLINE_ROUTINE void st_le32(volatile uint32_t *addr, uint32_t val)
{
	*addr = CPU_swap_u32(val);
}

RTEMS_INLINE_ROUTINE uint16_t ld_be16(volatile uint16_t *addr)
{
	return *addr;
}

RTEMS_INLINE_ROUTINE void st_be16(volatile uint16_t *addr, uint16_t val)
{
	*addr = val;
}

RTEMS_INLINE_ROUTINE uint32_t ld_be32(volatile uint32_t *addr)
{
	return *addr;
}

RTEMS_INLINE_ROUTINE void st_be32(volatile uint32_t *addr, uint32_t val)
{
	*addr = val;
}

#ifdef __cplusplus
}
#endif

#endif
