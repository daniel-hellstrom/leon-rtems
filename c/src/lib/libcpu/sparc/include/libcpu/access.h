/*
 * access.h  - access routines for SPARC. SPARC is big endian only.
 *
 * Copyright (C) 2011  Daniel Hellstrom (daniel@gaisler.com)
 *
 */

#ifndef _LIBCPU_ACCESS_H
#define _LIBCPU_ACCESS_H

#include <rtems/system.h>
#include <rtems/score/cpu.h>

#ifdef __cplusplus
extern "C" {
#endif

/* "Raw" access */
extern uint8_t sparc_ld8(uint8_t *addr);
extern void sparc_st8(uint16_t *addr, uint8_t val);
extern uint16_t sparc_ld16(uint16_t *addr);
extern void sparc_st16(uint16_t *addr, uint16_t val);
extern uint32_t sparc_ld32(uint32_t *addr);
extern void sparc_st32(uint32_t *addr, uint32_t val);
extern uint64_t sparc_ld64(uint16_t *addr);
extern void sparc_st64(uint64_t *addr, uint64_t val);

/* Aliases for Big Endian */
extern uint16_t sparc_ld_be16(uint16_t *addr);
extern void sparc_st_be16(uint16_t *addr, uint16_t val);
extern uint32_t sparc_ld_be32(uint32_t *addr);
extern void sparc_st_be32(uint32_t *addr, uint32_t val);

/* Little endian */
extern uint16_t sparc_ld_le16(uint16_t *addr);
extern void sparc_st_le16(uint16_t *addr, uint16_t val);
extern uint32_t sparc_ld_le32(uint32_t *addr);
extern void sparc_st_le32(uint32_t *addr, uint32_t val);

#ifdef __cplusplus
}
#endif

#endif
