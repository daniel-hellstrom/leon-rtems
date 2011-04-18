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

static inline uint16_t ld_le16(volatile uint16_t *addr)
{
	return CPU_swap_u16(*addr);
}

static inline void st_le16(volatile uint16_t *addr, uint16_t val)
{
	*addr = CPU_swap_u16(val);
}

static inline uint32_t ld_le32(volatile uint32_t *addr)
{
	return CPU_swap_u32(*addr);
}

static inline void st_le32(volatile uint32_t *addr, uint32_t val)
{
	*addr = CPU_swap_u32(val);
}

static inline uint16_t ld_be16(volatile uint16_t *addr)
{
	return *addr;
}

static inline void st_be16(volatile uint16_t *addr, uint16_t val)
{
	*addr = val;
}

static inline uint32_t ld_be32(volatile uint32_t *addr)
{
	return *addr;
}

static inline void st_be32(volatile uint32_t *addr, uint32_t val)
{
	*addr = val;
}

#endif
