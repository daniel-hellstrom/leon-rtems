/*  SpaceWire RMAP AMBA Plug & Play bus driver interface.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __AMBAPP_BUS_RMAP_H__
#define __AMBAPP_BUS_RMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

extern void ambapp_rmap_register(void);

/*** Bus operations with READ/WRITE access operations ***
 *
 * The functions are implemented using the standard drvmgr RW interface
 */
#define AMBAPP_RMAP_R8        DRVMGR_RWFUNC(RW_SIZE_1|RW_READ|RW_REG)
#define AMBAPP_RMAP_R16       DRVMGR_RWFUNC(RW_SIZE_2|RW_READ|RW_REG)
#define AMBAPP_RMAP_R32       DRVMGR_RWFUNC(RW_SIZE_4|RW_READ|RW_REG)
#define AMBAPP_RMAP_R64       DRVMGR_RWFUNC(RW_SIZE_8|RW_READ|RW_REG)
#define AMBAPP_RMAP_W8        DRVMGR_RWFUNC(RW_SIZE_1|RW_WRITE|RW_REG)
#define AMBAPP_RMAP_W16       DRVMGR_RWFUNC(RW_SIZE_2|RW_WRITE|RW_REG)
#define AMBAPP_RMAP_W32       DRVMGR_RWFUNC(RW_SIZE_4|RW_WRITE|RW_REG)
#define AMBAPP_RMAP_W64       DRVMGR_RWFUNC(RW_SIZE_8|RW_WRITE|RW_REG)
#define AMBAPP_RMAP_RMEM      DRVMGR_RWFUNC(RW_SIZE_ANY|RW_READ|RW_MEM)
#define AMBAPP_RMAP_WMEM      DRVMGR_RWFUNC(RW_SIZE_ANY|RW_WRITE|RW_MEM)
#define AMBAPP_RMAP_MEMSET    DRVMGR_RWFUNC(RW_SIZE_ANY|RW_SET|RW_MEM)
#define AMBAPP_RMAP_RW_ARG    DRVMGR_RWFUNC(RW_ARG)
#define AMBAPP_RMAP_RW_ERR    DRVMGR_RWFUNC(RW_ERR)

/* Read/Write function types with additional argument */
typedef uint8_t (*ambapp_rmap_r8)(uint8_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint16_t (*ambapp_rmap_r16)(uint16_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint32_t (*ambapp_rmap_r32)(uint32_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint64_t (*ambapp_rmap_r64)(uint64_t *srcadr, struct drvmgr_rw_arg *a);
typedef void (*ambapp_rmap_w8)(uint8_t *dstadr, uint8_t data, struct drvmgr_rw_arg *a);
typedef void (*ambapp_rmap_w16)(uint16_t *dstadr, uint16_t data, struct drvmgr_rw_arg *a);
typedef void (*ambapp_rmap_w32)(uint32_t *dstadr, uint32_t data, struct drvmgr_rw_arg *a);
typedef void (*ambapp_rmap_w64)(uint64_t *dstadr, uint64_t data, struct drvmgr_rw_arg *a);
/* READ/COPY a memory area located on bus into CPU memory.
 * From 'src' (remote) to the destination 'dest' (local), n=number of bytes
 */
typedef int (*ambapp_rmap_rmem)(void *dest, const void *src, int n, struct drvmgr_rw_arg *a);
/* WRITE/COPY a user buffer located in CPU memory to a location on the bus.
 * From 'src' (local) to the destination 'dest' (remote), n=number of bytes
 */
typedef int (*ambapp_rmap_wmem)(void *dest, const void *src, int n, struct drvmgr_rw_arg *a);
/* Set a memory area to the byte value given in c, see LIBC memset(). Memset is
 * implemented by calling wmem() multiple times with a "large" buffer.
 */
typedef int (*ambapp_rmap_memset)(void *dstadr, int c, size_t n, struct drvmgr_rw_arg *a);

/* Allocate memory from the SpaceWire target's memory */
extern void *ambapp_rmap_partition_memalign(
	struct drvmgr_dev *dev,
	int partition,
	size_t boundary,
	size_t size);

/* Register a partition of memory in the SpaceWire target */
extern int ambapp_rmap_partition_create(
	struct drvmgr_dev *dev,
	int partition,
	unsigned int start,
	size_t size);

#ifdef __cplusplus
}
#endif

#endif
