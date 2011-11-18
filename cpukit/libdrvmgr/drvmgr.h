/*  Driver Manager Interface.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler AB
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *
 *
 * Define this to make the driver manager do Breath First "searching" when 
 * initializing devices on buses. If not defined depth first will be used.
 */

#ifndef _DRIVER_MANAGER_H_
#define _DRIVER_MANAGER_H_

#include <rtems.h>
#include <drvmgr/drvmgr_list.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the Driver Manager */
extern void _DRV_Manager_initialization(void);

/*** Configure Driver manager ***/

/* Define the number of initialization levels of device drivers */
#define DRVMGR_LEVEL_MAX 4

/* Add support for I/O register and memory READ/WRITE functions. */
#define DRV_MGR_BUS_RW

struct rtems_drvmgr_dev_info;	/* Driver */
struct rtems_drvmgr_bus_info;	/* Bus */
struct rtems_drvmgr_drv_info;	/* Device */

/*** List Interface shortcuts ***/
#define BUS_LIST_HEAD(list) LIST_HEAD(list, struct rtems_drvmgr_bus_info)
#define BUS_LIST_TAIL(list) LIST_TAIL(list, struct rtems_drvmgr_bus_info)
#define DEV_LIST_HEAD(list) LIST_HEAD(list, struct rtems_drvmgr_dev_info)
#define DEV_LIST_TAIL(list) LIST_TAIL(list, struct rtems_drvmgr_dev_info)
#define DRV_LIST_HEAD(list) LIST_HEAD(list, struct rtems_drvmgr_drv_info)
#define DRV_LIST_TAIL(list) LIST_TAIL(list, struct rtems_drvmgr_drv_info)

/*** Bus indentification ***/
#define DRVMGR_BUS_TYPE_NONE 0		/* Not a valid bus */
#define DRVMGR_BUS_TYPE_ROOT 1		/* Hard coded bus */
#define DRVMGR_BUS_TYPE_PCI 2		/* PCI bus */
#define DRVMGR_BUS_TYPE_AMBAPP 3	/* AMBA Plug & Play bus */
#define DRVMGR_BUS_TYPE_LEON2_AMBA 4	/* LEON2 hardcoded bus */
#define DRVMGR_BUS_TYPE_AMBAPP_DIST 5	/* Distibuted AMBA Plug & Play bus accessed using a communication interface */
#define DRVMGR_BUS_TYPE_SPW_RMAP 6	/* SpaceWire Network bus */
#define DRVMGR_BUS_TYPE_AMBAPP_RMAP 7	/* SpaceWire RMAP accessed AMBA Plug & Play bus */

/*** Driver indentification ***/ 

/* 64-bit identification integer definition
 *  ¤ Bus ID 8-bit [7..0]
 *  ¤ Reserved 8-bit field [63..56]
 *  ¤ Device ID specific for bus type 48-bit [55..8]  (Different buses have different unique identifications for hardware/driver.)
 *
 * ID Rules
 *  ¤ A root bus driver must always have device ID set to 0. There can only by one root bus driver
 *    for a certain bus type.
 *  ¤ A Driver ID must identify a unique hardware core
 *
 */

/* Bus ID Mask */
#define DRIVER_ID_BUS_MASK 0x00000000000000FFULL

/* Reserved Mask for future use */
#define DRIVER_ID_RSV_MASK 0xFF00000000000000ULL

/* Reserved Mask for future use */
#define DRIVER_ID_DEV_MASK 0x00FFFFFFFFFFFF00ULL

/* Set Bus ID Mask. */
#define DRIVER_ID(busid, devid) \
	(unsigned long long)( (((unsigned long long)(devid) << 8) & DRIVER_ID_DEV_MASK) | ((unsigned long long)(busid) & DRIVER_ID_BUS_MASK) )

/* Get IDs */
#define DRIVER_BUSID_GET(id)		((unsigned long long)(id) & DRIVER_ID_BUS_MASK)
#define DRIVER_DEVID_GET(id)		(((unsigned long long)(id) & DRIVER_ID_DEV_MASK) >> 8)

#define DRIVER_ROOTBUS_ID(bus_type)	DRIVER_ID(bus_type, 0)

/*** Root Bus drivers ***/

/* Generic Hard coded Root bus: Driver ID */
#define DRIVER_ROOT_ID			DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_ROOT)

/* PCI Plug & Play bus: Driver ID */
#define DRIVER_PCIBUS_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_PCI)

/* AMBA Plug & Play bus: Driver ID */
#define DRIVER_GRLIB_AMBAPP_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_AMBAPP)

/* AMBA Hard coded bus: Driver ID */
#define DRIVER_LEON2_AMBA_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_LEON2_AMBA)

/* Distributed AMBA Plug & Play bus: Driver ID */
#define DRIVER_AMBAPP_DIST_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_AMBAPP_DIST)

/*! Bus parameters used by driver interface functions to aquire information about 
 * bus. All Bus drivers should implement the operation 'get_params' so that the
 * driver interface routines can access bus dependent information in an non-dependent 
 * way.
 */
struct rtems_drvmgr_bus_params {
#if 0
	unsigned int	freq_hz;		/*!< Frequency of bus in Hz */
#endif
	char		*dev_prefix;		/*!<  */
};

/* Interrupt Service Routine (ISR) */
typedef void (*rtems_drvmgr_isr)(int irq, void *arg);

/*! Bus operations */
struct rtems_drvmgr_bus_ops {
	/* Functions used internally within driver manager */
	int	(*init[DRVMGR_LEVEL_MAX])(struct rtems_drvmgr_bus_info *);
	int	(*remove)(struct rtems_drvmgr_bus_info *);
	int	(*unite)(struct rtems_drvmgr_drv_info *, struct rtems_drvmgr_dev_info *);	/*!< Unite Hardware Device with Driver */
	int	(*dev_id_compare)(void *a, void *b);						/*!< Compare hardware devices from bus information */

	/* Functions called indirectly from drivers */
	int	(*int_register)(struct rtems_drvmgr_dev_info *, int index, rtems_drvmgr_isr isr, void *arg);
	int	(*int_unregister)(struct rtems_drvmgr_dev_info *, int index, rtems_drvmgr_isr isr, void *arg);
	int	(*int_enable)(struct rtems_drvmgr_dev_info *, int index, rtems_drvmgr_isr isr, void *arg);
	int	(*int_disable)(struct rtems_drvmgr_dev_info *, int index, rtems_drvmgr_isr isr, void *arg);
	int	(*int_clear)(struct rtems_drvmgr_dev_info *, int index, rtems_drvmgr_isr isr, void *arg);
	int	(*int_mask)(struct rtems_drvmgr_dev_info *, int index);
	int	(*int_unmask)(struct rtems_drvmgr_dev_info *, int index);

	/* Get Paramters */
	int	(*get_params)(struct rtems_drvmgr_dev_info *, struct rtems_drvmgr_bus_params *);
	/* Get Frequency of Bus */
	int	(*freq_get)(struct rtems_drvmgr_dev_info*, int, unsigned int*);

#ifdef DRV_MGR_BUS_RW
	/* Read I/O functions */
	int	(*read_io8)(struct rtems_drvmgr_dev_info *dev, uint8_t *srcadr, uint8_t *data);
	int	(*read_io16)(struct rtems_drvmgr_dev_info *dev, uint16_t *srcadr, uint16_t *data);
	int	(*read_io32)(struct rtems_drvmgr_dev_info *dev, uint32_t *srcadr, uint32_t *data);
	int	(*read_io64)(struct rtems_drvmgr_dev_info *dev, uint64_t *srcadr, uint64_t *data);

	/* Write I/O functions */
	int	(*write_io8)(struct rtems_drvmgr_dev_info *dev, uint8_t *dstadr, uint8_t data);
	int	(*write_io16)(struct rtems_drvmgr_dev_info *dev, uint16_t *dstadr, uint16_t data);
	int	(*write_io32)(struct rtems_drvmgr_dev_info *dev, uint32_t *dstadr, uint32_t data);
	int	(*write_io64)(struct rtems_drvmgr_dev_info *dev, uint64_t *dstadr, uint64_t data);

	/* Read/Write Memory functions */
	int	(*read_mem)(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n);
	int	(*write_mem)(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n);
#endif
};

/*** Resource definitions ***
 * 
 * Overview of structures:
 *  All bus resources entries (_bus_res) are linked together per bus (bus_info->reslist).
 *  One bus resource entry has a pointer to an array of driver resources (_drv_res). One driver 
 *  resouces is made out of an array of keys (rtems_drvmgr_key). All keys belongs to the 
 *  same driver and harwdare device. Each key has a Name, Type ID and Data interpreted 
 *  differently depending on the Type ID (union rtems_drvmgr_key_value).
 *
 */

/* Key Data Types */
#define KEY_TYPE_NONE		0
#define KEY_TYPE_INT 		1
#define KEY_TYPE_STRING		2
#define KEY_TYPE_POINTER	3

#define KEY_EMPTY	{NULL, KEY_TYPE_NONE, {0}}
#define RES_EMPTY	{0, 0, NULL}
#define MMAP_EMPTY	{0, 0, 0}

/*! Union of different values */
union rtems_drvmgr_key_value {
	unsigned int		i;		/*!< Key data type UNSIGNED INTEGER */
	char			*str;		/*!< Key data type STRING */
	void			*ptr;		/*!< Key data type ADDRESS/POINTER */
};

/* One key. One Value. Holding information relevant to the driver. */
struct rtems_drvmgr_key {
	char				*key_name;	/* Name of key */
	int				key_type;	/* How to interpret key_value */
	union rtems_drvmgr_key_value	key_value;	/* The value or pointer to the value */
};

/*! Driver resource entry, Driver resources for a certain device instance, containing a number of keys 
 * Where each key hold the data of interest.
 */
struct rtems_drvmgr_drv_res {
	uint64_t		drv_id;		/*!< Identifies the driver this resource is aiming */
	int			minor_bus;	/*!< Indentifies a specfic device */
	struct rtems_drvmgr_key	*keys;		/*!< First key in key array, ended with KEY_EMPTY */
};

/*! Bus resource list node */
struct rtems_drvmgr_bus_res {
	struct rtems_drvmgr_bus_res	*next;		/*!< Next resource node in list */
	struct rtems_drvmgr_drv_res	*resource;	/*!< Array of resources, one per device instance */
};

/*! MAP entry. Describes an linear address space translation. Untranslated
 *  Start, Translated Start and length.
 *
 * Used by bus drivers to describe the address translation needed for 
 * the translation driver interface.
 */
struct rtems_drvmgr_mmap_entry {
	unsigned int	map_size;	/*!< Size of memory Window */
	char		*local_adr;	/*!< Start address of Window from CPU's 
					 * point of view */
	char		*remote_adr;	/*!< Start address of Remote system
					 * point of view */
};

/*! Bus information. Describes a bus. */
struct rtems_drvmgr_bus_info {
	int				bus_type;	/*!< Type of bus */
	struct rtems_drvmgr_bus_info	*next;		/*!< Next Bus */
	struct rtems_drvmgr_dev_info	*dev;		/*!< Bus device, the hardware... */
	void				*priv;		/*!< Private data structure used by BUS driver */
	struct rtems_drvmgr_dev_info	*children;	/*!< Hardware devices on this bus */
	struct rtems_drvmgr_bus_ops	*ops;		/*!< Bus operations supported by this bus driver */
	int				dev_cnt;	/*!< Number of devices this bus has */
	struct rtems_drvmgr_bus_res	*reslist;	/*!< Bus resources, head of a linked list of resources. */
	struct rtems_drvmgr_mmap_entry	*mmaps;		/*!< Memory Map Translation, array of address spaces */

	/* Bus status */
	int				level;
	int				state;
	int				error;
};

/* States of a bus */
#define BUS_STATE_INIT_FAILED	0x00000001	/* Initialization Failed */
#define BUS_STATE_LIST_INACTIVE	0x00001000	/* In inactive bus list */

/* States of a device */
#define DEV_STATE_INIT_FAILED	0x00000001	/* Initialization Failed */
#define DEV_STATE_INIT_DONE	0x00000002	/* All init levels completed */
#define DEV_STATE_UNITED	0x00000100	/* Device United with Device Driver */
#define DEV_STATE_REMOVED	0x00000200	/* Device has been removed (unregistered) */
#define DEV_STATE_IGNORED	0x00000400	/* Device was ignored according to user's request, the device
						 * was never reported to it's driver (as expected).
						 */
#define DEV_STATE_LIST_INACTIVE	0x00001000	/* In inactive device list */

/*! Device information */
struct rtems_drvmgr_dev_info {
	struct rtems_drvmgr_dev_info	*next;		/*!< Next device */
	struct rtems_drvmgr_dev_info	*next_in_bus;	/*!< Next device on the same bus */
	struct rtems_drvmgr_dev_info	*next_in_drv;	/*!< Next device using the same driver */

	struct rtems_drvmgr_drv_info	*drv;		/*!< The driver owning this device */
	struct rtems_drvmgr_bus_info	*parent;	/*!< Bus that this device resides on */
	short				minor_drv;	/*!< Device number on driver (often minor in filesystem) */
	short				minor_bus;	/*!< Device number on bus (for device separation) */
	char				*name;		/*!< Name of Device Hardware */
	void				*priv;		/*!< Pointer to driver private device structure */
	void				*businfo;	/*!< Host bus specific information */
	struct rtems_drvmgr_bus_info	*bus;		/*!< Pointer to bus, set only if this is a bus */

	/* Device Status */
	unsigned int			state;		/*!< State of this device, see DEV_STATE_* */
	int				level;		/*!< Init Level */
	int				error;		/*!< Error state returned by driver */
#ifdef DRV_MGR_BUS_RW
	struct rtems_drvmgr_dev_info	*rw_dev;	/*!< Device used to make READ/WRITE bus operations, 
							 *   NULL=use calling device */
#endif
};

/*! Driver operations, function pointers. */
struct rtems_drvmgr_drv_ops {
	int	(*init[DRVMGR_LEVEL_MAX])(struct rtems_drvmgr_dev_info *);	/*! Function doing Init Stage 1 of a hardware device */
	int	(*remove)(struct rtems_drvmgr_dev_info *);	/*! Function called when device instance is to be removed */
	int	(*info)(struct rtems_drvmgr_dev_info *, int, int);/*! Function called to request information about a device or driver */
};

/*! Information about a driver used during registration */
struct rtems_drvmgr_drv_info {
	struct rtems_drvmgr_drv_info	*next;		/*!< Next Driver */
	struct rtems_drvmgr_dev_info	*dev;		/*!< Devices using this driver */

	uint64_t			drv_id;		/*!< Unique Driver ID */
	char				*name;		/*!< Name of Driver */
	int				bus_type;	/*!< Type of Bus this driver supports */
	struct rtems_drvmgr_drv_ops	*ops;		/*!< Driver operations */
	unsigned int			dev_cnt;	/*!< Number of devices in dev */
	unsigned int			dev_priv_size;	/*!< If non-zero DRVMGR will allocate memory for dev->priv */
};

/*! Structure defines a function pointer called when driver manager is ready for drivers to register
 *  themselfs. Used to select drivers available to the driver manager.
 */
struct rtems_drvmgr_drv_reg_func {
	void (*drv_reg)(void);
};

/*** DRIVER | DEVICE | BUS FUNCTIONS ***/

/* Return Codes */
enum {
	DRVMGR_OK = 0,
	DRVMGR_NOMEM = 1,
	DRVMGR_FAIL = -1
};

/*! Initialize data structures of the driver management system. 
 *  Calls predefined register driver functions so that drivers can
 *  register themselves.
 */
extern void _DRV_Manager_init(void);

/*! Take all devices into init level 'level', all devices registered later
 *  will directly be taken into this level as well, ensuring that all
 *  registerd devices has been taken into the level.
 *
 */
extern void _DRV_Manager_init_level(int level);

/* Take registered buses and devices into the correct init level,
 * this function is called from _init_level() so normally
 * we don't need to call it directly.
 */
extern void drvmgr_init_update(void);

/*! Register Root Bus device driver */
extern int rtems_drvmgr_root_drv_register(struct rtems_drvmgr_drv_info *drv);

/*! Register a driver */
extern int rtems_drvmgr_drv_register(struct rtems_drvmgr_drv_info *drv);

/*! Register a device */
extern int rtems_drvmgr_dev_register(struct rtems_drvmgr_dev_info *dev);

/*! Remove a device, and all its children devices if device is a bus device. The device 
 *  driver will be requested to remove the device and once gone from bus, device and 
 *  driver list the device is put into a inactive list for debugging (this is optional
 *  by using remove argument).
 *
 * Removing the Root Bus Device is not supported.
 *
 * \param remove If non-zero the device will be deallocated, and not put into the 
 *               inacitve list.
 */
extern int rtems_drvmgr_dev_unregister(struct rtems_drvmgr_dev_info *dev, int remove);

/*! Register a bus */
extern int rtems_drvmgr_bus_register(struct rtems_drvmgr_bus_info *bus);

/*! Allocate a device structure, if no memory available rtems_error_fatal_occurred 
 * is called.
 * The 'extra' argment tells how many bytes extra space is to be allocated after
 * the device structure, this is typically used for "businfo" structures. The extra
 * space is always aligned to a 4-byte boundary.
 */
extern int rtems_drvmgr_alloc_dev(struct rtems_drvmgr_dev_info **pdev, int extra);

/*! Allocate a bus structure, if no memory available rtems_error_fatal_occurred 
 * is called.
 * The 'extra' argment tells how many bytes extra space is to be allocated after
 * the device structure, this is typically used for "businfo" structures. The extra
 * space is always aligned to a 4-byte boundary.
 */
extern int rtems_drvmgr_alloc_bus(struct rtems_drvmgr_bus_info **pbus, int extra);

/*** DRIVER RESOURCE FUNCTIONS ***/

/*! Add resources to a bus, typically used by a bus driver.
 *
 * \param bus   The Bus to add the resources to.
 * \param res   An array with Driver resources, all together are called bus resources.
 */
extern int rtems_drvmgr_bus_res_add(struct rtems_drvmgr_bus_info *bus, struct rtems_drvmgr_drv_res *res);

/*! Find all the resource keys for a device among all driver resources on a bus. Typically
 *  used by a device driver to get configuration options.
 *
 * \param dev   Device to find resources for
 * \param key   Location where the pointer to the driver resource array (rtems_drvmgr_drv_res->keys) is stored.
 */
extern int rtems_drvmgr_keys_get(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_key **keys);

/*! Return the one key that matches key name from a driver keys array. The keys can be obtained
 *  using rtems_drvmgr_keys_get.
 *  
 * \param keys       An array of keys ended with KEY_EMPTY to search among.
 * \param key_name   Name of key to search for among the keys.
 */
extern struct rtems_drvmgr_key *rtems_drvmgr_key_get(struct rtems_drvmgr_key *keys, char *key_name);

/*! Extract key value from the key in the keys array matching name and type.
 *
 *  This function calls rtems_drvmgr_keys_get to get the key requested (from key name), then determines
 *  if the type is correct. A pointer to the key value is returned.
 *  
 *  \param keys       An array of keys ended with KEY_EMPTY to search among.
 *  \param key_name   Name of key to search for among the keys.
 *  \param key_type   Data Type of value. INTEGER, ADDRESS, STRING.
 *  \return           Returns NULL if no value found matching Key Name and Key Type.
 */
extern union rtems_drvmgr_key_value *rtems_drvmgr_key_val_get(
	struct rtems_drvmgr_key *keys,
	char *key_name,
	int key_type);

/*! Get key value from the bus resources matching [device[drv_id, minor_bus], key name, key type] 
 * if no matching key NULL is returned.
 *
 * This is typically used by device drivers to find a particular device resource.
 *
 * \param dev         The device to search resource for.
 * \param key_name    The key name to search for
 * \param key_type    The key type expected.
 * \return            Returns NULL if no value found matching Key Name and Key Type was found for device.
 */
extern union rtems_drvmgr_key_value *rtems_drvmgr_dev_key_get(
	struct rtems_drvmgr_dev_info *dev,
	char *key_name,
	int key_type);

/*** DRIVER INTERACE USED TO REQUEST INFORMATION/SERVICES FROM BUS DRIVER ***/

/*! Get Device pointer from Driver and Driver minor number 
 *
 * \param drv         Driver the device is united with.
 * \param minor       Driver minor number assigned to device.
 * \param pdev        Location where the Device point will be stored.
 * \return            Zero on success. -1 on failure, when device was not found in driver 
 *                    device list.
 */
extern int rtems_drvmgr_get_dev(
	struct rtems_drvmgr_drv_info *drv,
	int minor,
	struct rtems_drvmgr_dev_info **pdev);

/*! Get Bus frequency in Hertz. Frequency is stored into address of freq_hz.
 *
 * \param dev        The Device to get Bus frequency for.
 * \param options    Bus-type specific options
 * \param freq_hz    Location where Bus Frequency will be stored.
 */
extern int rtems_drvmgr_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz);

/*! Return 0 if dev is not located on the root bus, 1 if on root bus */
extern int rtems_drvmgr_on_rootbus(struct rtems_drvmgr_dev_info *dev);

/*! Get device name prefix, this name can be used to register a unique name in the 
 *  filesystem or to get an idea where the device is located.
 *
 * \param dev         The Device to get the device Prefix for.
 * \param dev_prefix  Location where the prefix will be stored.
 */
extern int rtems_drvmgr_get_dev_prefix(struct rtems_drvmgr_dev_info *dev, char *dev_prefix);

/*! Register a shared interrupt handler. Since this service is shared among interrupt 
 *  drivers/handlers the handler[arg] must be installed before the interrupt can be
 *  cleared or disabled. The handler is by default disabled after registration.
 *
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine.
 *  \param arg        Optional ISR argument.
 */
extern int rtems_drvmgr_interrupt_register(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);

/*! Unregister an interrupt handler. This also disables the interrupt before unregistering
 *  the interrupt handler.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine, previously registered.
 *  \param arg        Optional ISR argument, previously registered.
 */
extern int rtems_drvmgr_interrupt_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);

/*! Enable (unmask) an interrupt source 
 * 
 *  \param dev        Device to enable interrupt for.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine, previously registered.
 *  \param arg        Optional ISR argument, previously registered.
 */
extern int rtems_drvmgr_interrupt_enable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);

/*! Disable (mask) an interrupt source 
 *
 *  \param dev        Device to disable interrupt for.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine, previously registered.
 *  \param arg        Optional ISR argument, previously registered.
 */
extern int rtems_drvmgr_interrupt_disable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);

/*! Clear (ACK) pending interrupt
 *
 *  \param dev        Device to clear interrupt for.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine, previously registered.
 *  \param arg        Optional ISR argument, previously registered.
 */
extern int rtems_drvmgr_interrupt_clear(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);

/*! Force unmasking/enableing an interrupt on the interrupt controller, this is not normally used,
 *  if used the caller has masked/disabled the interrupt just before.
 *
 *  \param dev        Device to clear interrupt for.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 *  \param isr        Interrupt Service Routine, previously registered.
 *  \param arg        Optional ISR argument, previously registered.
 */
extern int rtems_drvmgr_interrupt_unmask(
	struct rtems_drvmgr_dev_info *dev,
	int index);

/*! Force masking/disable an interrupt on the interrupt controller, this is not normally performed
 *  since this will stop all other (shared) ISRs to be disabled until _unmask() is called.
 *
 *  \param dev        Device to mask interrupt for.
 *  \param index      Index is used to identify the IRQ number if hardware has multiple IRQ sources. 
 *                    Normally Index is set to 0 to indicated the first and only IRQ source.
 *                    A negative index is interpreted as a absolute bus IRQ number.
 */
extern int rtems_drvmgr_interrupt_mask(
	struct rtems_drvmgr_dev_info *dev,
	int index);

/*! Translate address 
 * 1. From CPU local bus to a remote bus for example a PCI target (from_remote_to_cpu = 0)
 * 2. From remote bus to CPU local bus (from_remote_to_cpu = 1)
 *
 * src_address the address to translate, dst_address is where the translated address is stored.
 *
 * \param dev                  Device to translate addresses for.
 * \param from_remote_to_cpu   Selection tranlation direction.
 * \param src_address          Address to translate
 * \param dst_address          Location where translated address is stored.
 *
 * Returns -1 if unable to translate. If no map is present src_address is translated 1:1 (just copied).
 */
extern int rtems_drvmgr_mmap_translate(
	struct rtems_drvmgr_dev_info *dev,
	int from_remote_to_cpu,
	void *src_address,
	void **dst_address);

#ifdef DRV_MGR_BUS_RW
/*** Extended bus operations with READ/WRITE access operations ***
 *
 * Note that these functions are optional, and seldom used.
 */

/* READ a 8-bit I/O register */
extern int rtems_drvmgr_read_io8(
	struct rtems_drvmgr_dev_info *dev,
	uint8_t *srcadr,
	uint8_t *result
	);

/* READ a 16-bit I/O register */
extern int rtems_drvmgr_read_io16(
	struct rtems_drvmgr_dev_info *dev,
	uint16_t *srcadr,
	uint16_t *result
	);

/* READ a 32-bit I/O register */
extern int rtems_drvmgr_read_io32(
	struct rtems_drvmgr_dev_info *dev,
	uint32_t *srcadr,
	uint32_t *result
	);

/* READ a 64-bit I/O register */
extern int rtems_drvmgr_read_io64(
	struct rtems_drvmgr_dev_info *dev,
	uint64_t *srcadr,
	uint64_t *result
	);

/* WRITE a 8-bit I/O register */
extern int rtems_drvmgr_write_io8(
	struct rtems_drvmgr_dev_info *dev,
	uint8_t *dstadr,
	uint8_t data
	);

/* WRITE a 16-bit I/O register */
extern int rtems_drvmgr_write_io16(
	struct rtems_drvmgr_dev_info *dev,
	uint16_t *dstadr,
	uint16_t data
	);

/* WRITE a 32-bit I/O register */
extern int rtems_drvmgr_write_io32(
	struct rtems_drvmgr_dev_info *dev,
	uint32_t *dstadr,
	uint32_t data
	);

/* WRITE a 64-bit I/O register */
extern int rtems_drvmgr_write_io64(
	struct rtems_drvmgr_dev_info *dev,
	uint64_t *dstadr,
	uint64_t data
	);

/* READ/COPY a memory area located in 'src' to the destination 'dest', n=number of bytes */
extern int rtems_drvmgr_read_mem(
	struct rtems_drvmgr_dev_info *dev,
	void *dest,
	const void *src,
	int n
	);

/* WRITE/COPY a user buffer located in 'src' to the destination 'dest', n=number of bytes */
extern int rtems_drvmgr_write_mem(
	struct rtems_drvmgr_dev_info *dev,
	void *dest,
	const void *src,
	int n
	);

/* Set a memory area to the byte value given in c, see LIBC memset(). Memset is implemented by 
 * calling busops->write_mem multiple times with a zero buffer.
 */
extern int rtems_drvmgr_write_memset(
	struct rtems_drvmgr_dev_info *dev,
	void *dstadr,
	int c,
	size_t n
	);

#endif

/*** PRINT INFORMATION ABOUT DRIVER MANAGER ***/

/*! Calls func() for every device found matching the search requirements of 
 * set_mask and clr_mask. Each bit set in set_mask must be set in the 
 * device state bit mask (dev->state), and Each bit in the clr_mask must
 * be cleared in the device state bit mask (dev->state). There are three
 * special cases:
 *
 * 1. If state_set_mask and state_clr_mask are zero the state bits are
 *    ignored and all cores are treated as a match.
 *
 * 2. If state_set_mask is zero the function func will not be called due to 
 *    a bit being set in the state mask.
 *
 * 3. If state_clr_mask is zero the function func will not be called due to 
 *    a bit being cleared in the state mask.
 *
 * If the function func() returns a non-zero value then for_each_dev will
 * return imediatly with the same return value as func() returned.
 *
 * \param devlist            The list to iterate though searching for devices.
 * \param state_set_mask     Defines the bits that must be set in dev->state
 * \param state_clr_mask     Defines the bits that must be cleared in dev->state
 * \param func               Function called on each 
 *
 */
extern int rtems_drvmgr_for_each_dev(
	struct rtems_drvmgr_list *devlist,
	unsigned int state_set_mask,
	unsigned int state_clr_mask,
	int (*func)(struct rtems_drvmgr_dev_info *dev, void *arg),
	void *arg);

/*! Print all drivers */
extern void rtems_drvmgr_print_drvs(int show_devs);

/* Print all devices */
#define DRV_MGR_PRINT_DEVS_FAILED	0x01	/* Failed during initialization */
#define DRV_MGR_PRINT_DEVS_ASSIGNED	0x02	/* Driver assigned */
#define DRV_MGR_PRINT_DEVS_UNASSIGNED	0x04	/* Driver not assigned */
#define DRV_MGR_PRINT_DEVS_REMOVED	0x08	/* Device removed */
#define DRV_MGR_PRINT_DEVS_IGNORED	0x10	/* Device ignored on user's request */

#define DRV_MGR_PRINT_DEVS_ALL		(DRV_MGR_PRINT_DEVS_FAILED | \
					DRV_MGR_PRINT_DEVS_ASSIGNED | \
					DRV_MGR_PRINT_DEVS_UNASSIGNED |\
					DRV_MGR_PRINT_DEVS_REMOVED |\
					DRV_MGR_PRINT_DEVS_IGNORED)

/*! Print devices with certain condictions met according to 'options' */
extern void rtems_drvmgr_print_devs(unsigned int options);

/*! Print device/bus topology */
extern void rtems_drvmgr_print_topo(void);

/*! Print all buses */
extern void rtems_drvmgr_print_buses(void);

/*! Print the memory usage
 * Only accounts for data structures. Not for the text size.
 */
extern void rtems_drvmgr_print_mem(void);

#ifdef __cplusplus
}
#endif

#endif
