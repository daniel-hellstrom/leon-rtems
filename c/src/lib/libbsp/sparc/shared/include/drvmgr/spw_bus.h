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
	struct rtems_drvmgr_key		*keys;	/* Target setup (Destination address, Destination Key) */
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
	struct rtems_drvmgr_drv_res	*resources;	/* Driver resouces present on the bus */
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
	struct rtems_drvmgr_drv_info	general;	/* General bus info */
	/* SpW RMAP specific bus information */
	struct spw_id			*ids;		/* Supported target hardware */
};

extern int rtems_drvmgr_memcpy(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n);
extern unsigned char rtems_drvmgr_read_byte(struct rtems_drvmgr_dev_info *dev, unsigned char *address);
extern unsigned short rtems_drvmgr_read_word(struct rtems_drvmgr_dev_info *dev, unsigned short *address);
extern unsigned int rtems_drvmgr_read_dword(struct rtems_drvmgr_dev_info *dev, unsigned int *address);
extern unsigned long long rtems_drvmgr_read_qword(struct rtems_drvmgr_dev_info *dev, unsigned long long *address);
extern int rtems_drvmgr_write_byte(struct rtems_drvmgr_dev_info *dev, unsigned char *address, unsigned char data);
extern int rtems_drvmgr_write_word(struct rtems_drvmgr_dev_info *dev, unsigned short *address, unsigned short data);
extern int rtems_drvmgr_write_dword(struct rtems_drvmgr_dev_info *dev, unsigned int *address, unsigned int data);
extern int rtems_drvmgr_write_qword(struct rtems_drvmgr_dev_info *dev, unsigned long long *address, unsigned long long data);

#ifdef __cplusplus
}
#endif

#endif
