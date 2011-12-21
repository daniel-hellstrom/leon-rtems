/*  SpaceWire Network bus driver interface.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __SPW_BUS_H__
#define __SPW_BUS_H__

#include <drvmgr/drvmgr.h>
#include <drvmgr/spw_bus_ids.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SpW Bus Driver ID generation */
#define DRIVER_SPWBUS_ID(vendor, device) \
	DRIVER_ID(DRVMGR_BUS_TYPE_SPW_RMAP, ((((vendor) & 0xffff) << 16) | ((device) & 0xffff)))

/* SpaceWire Driver IDs */
#define DRIVER_SPW_RMAP_AMBAPP_ID	1

/**** SpaceWire  Driver IDs *****/
#define SPW_NODE_ID_GRLIB		DRIVER_SPWBUS_ID(SPWBUS_VENDOR_GAISLER,SPWBUS_DEVICE_GAISLER_GRLIB)


/* SpaceWire Target ID */
struct spw_id {
	unsigned int	spwid;	/* SpaceWire Target ID */
};
#define SPW_NODE_ID_NONE 0

/* SpaceWire Target Setup */
struct spw_node {
	struct spw_id			id;	/* Node ID */
	char				*name;	/* Name of Target */
	struct drvmgr_key		*keys;	/* Target setup (Destination address, Destination Key) */
};

/* spw_node.keys that must be defined for SpaceWire targets 
 * INTEGER	DST_ADR		SpaceWire Destination Address
 * INTEGER	DST_KEY		RMAP Destination Key
 */
	
#define EMPTY_SPW_NODE {{SPW_NODE_ID_NONE}, NULL, NULL}

/* Virtual IRQ to GPIO Pin translation */
struct spwbus_virq_config {
	char	*gpio_fsname;	/* GPIO Filesystem Name */
	void	*handle;	/* If already opened, set "File descriptor" handle here */
};

/* SpaceWire Bus driver configuration */
struct spw_bus_config {
	void				*rmap;		/* RMAP Stack handle, note that it should be thread-safe */
	int				maxlen;		/* Maximum length */
	struct spw_node			*nodes;		/* Bus configuration (SpaceWire nodes available) */
	char				devName[32];	/* GRSPW Dev name bus is attached to */
	struct drvmgr_bus_res	*resources;	/* Driver resouces present on the bus */
	struct spwbus_virq_config	virq_table[4];	/* Virtual IRQ number to GPIO translation table */
};

/* Register SpW Bus */
int drv_mgr_spw_init(struct spw_bus_config *config);

struct spw_bus_dev_info {
	unsigned int	spwid;

	unsigned char	dstadr;
	unsigned char	dstkey;
	char		virqs[4]; /* 4 virtual IRQs(VIRQ[1..4]) */
};

#define SPWBUS_VIRQ1 1
#define SPWBUS_VIRQ2 2
#define SPWBUS_VIRQ3 3
#define SPWBUS_VIRQ4 4

struct spw_bus_drv_info {
	struct drvmgr_drv	general;	/* General bus info */
	/* SpW RMAP specific bus information */
	struct spw_id		*ids;		/* Supported target hardware */
};

/*** Bus operations with READ/WRITE access operations ***/
#define SPWBUS_RW_ARG    DRVMGR_RWFUNC(RW_ARG)
#define SPWBUS_RW_ERR    DRVMGR_RWFUNC(RW_ERR)
#define SPWBUS_R8        DRVMGR_RWFUNC(RW_SIZE_1|RW_READ|RW_REG)
#define SPWBUS_R16       DRVMGR_RWFUNC(RW_SIZE_2|RW_READ|RW_REG)
#define SPWBUS_R32       DRVMGR_RWFUNC(RW_SIZE_4|RW_READ|RW_REG)
#define SPWBUS_R64       DRVMGR_RWFUNC(RW_SIZE_8|RW_READ|RW_REG)
#define SPWBUS_W8        DRVMGR_RWFUNC(RW_SIZE_1|RW_WRITE|RW_REG)
#define SPWBUS_W16       DRVMGR_RWFUNC(RW_SIZE_2|RW_WRITE|RW_REG)
#define SPWBUS_W32       DRVMGR_RWFUNC(RW_SIZE_4|RW_WRITE|RW_REG)
#define SPWBUS_W64       DRVMGR_RWFUNC(RW_SIZE_8|RW_WRITE|RW_REG)
#define SPWBUS_RMEM      DRVMGR_RWFUNC(RW_SIZE_ANY|RW_READ|RW_MEM)
#define SPWBUS_WMEM      DRVMGR_RWFUNC(RW_SIZE_ANY|RW_WRITE|RW_MEM)
#define SPWBUS_MEMSET    DRVMGR_RWFUNC(RW_SIZE_ANY|RW_SET|RW_MEM)

/* Read/Write function types with additional argument */
typedef uint8_t (*spwbus_r8)(uint8_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint16_t (*spwbus_r16)(uint16_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint32_t (*spwbus_r32)(uint32_t *srcadr, struct drvmgr_rw_arg *a);
typedef uint64_t (*spwbus_r64)(uint64_t *srcadr, struct drvmgr_rw_arg *a);
typedef void (*spwbus_w8)(uint8_t *dstadr, uint8_t data, struct drvmgr_rw_arg *a);
typedef void (*spwbus_w16)(uint16_t *dstadr, uint16_t data, struct drvmgr_rw_arg *a);
typedef void (*spwbus_w32)(uint32_t *dstadr, uint32_t data, struct drvmgr_rw_arg *a);
typedef void (*spwbus_w64)(uint64_t *dstadr, uint64_t data, struct drvmgr_rw_arg *a);

/* READ/COPY a memory area located in 'src' to the destination 'dest', n=number of bytes */
typedef int (*spwbus_rmem)(void *dest, const void *src, int n, struct drvmgr_rw_arg *a);

/* WRITE/COPY a user buffer located in 'src' to the destination 'dest', n=number of bytes */
typedef int (*spwbus_wmem)(void *dest, const void *src, int n, struct drvmgr_rw_arg *a);

/* Set a memory area to the byte value given in c, see LIBC memset(). Memset is implemented by 
 * calling busops->write_mem multiple times with a zero buffer.
 */
typedef int (*spwbus_memset)(void *dstadr, int c, size_t n, struct drvmgr_rw_arg *a);

#ifdef __cplusplus
}
#endif

#endif
