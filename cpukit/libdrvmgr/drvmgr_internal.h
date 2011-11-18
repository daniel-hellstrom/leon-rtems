
#if 0
/* An object is here a Device or a Bus */
struct drvmgr_obj_init {
	/* Linked list of objects that have been successfully initialized */
	struct rtems_drvmgr_list	objs;
	/* Linked list of registered objects, that are waiting for
	 * initialization to level1
	 */
	struct rtems_drvmgr_list	objs_registered;
	/* Linked list of objects that have reached a certain initialization
	 * level. objs_init[N] correspond to level N+1.
	 */
	struct rtems_drvmgr_list	objs_init[DRVMGR_LEVEL_MAX-1];
	struct rtems_drvmgr_list	objs_inactive;
};

struct rtems_drvmgr {
	int	level;
	int	initializing_devs;

	/* The first device - The root device */
	struct drvmgr_drv	*root_drv;	/*!< Registered Root Driver */
	struct drvmgr_dev	*root_dev;	/*!< Registered Root Device */
	struct drvmgr_obj_init	buses;
	struct drvmgr_obj_init	devs;
};
#endif

/*! Structure hold all information the driver manager needs to know of. Used internally
 *  by Driver Manager routines.
 */
struct rtems_driver_manager {
	int	level;
	int	initializing_objs;

	/* The first device - The root device and it's driver */
	struct rtems_drvmgr_dev_info	*root_dev;
	struct rtems_drvmgr_drv_info	*root_drv;

	/*!< Linked list of all registered drivers */
	struct rtems_drvmgr_list	drivers;

	/* Buses that reached a certain initialization level.
	 * Lists by Level:
	 *  N=0         - Not intialized, just registered
	 *  N=1..MAX-1  - Reached init level N
	 *  N=MAX       - Successfully initialized bus
	 */
	struct rtems_drvmgr_list	buses[DRVMGR_LEVEL_MAX+1];
	/* Buses failed to initialize or has been removed by not freed */
	struct rtems_drvmgr_list	buses_inactive;		

	/* Devices that reached a certain initialization level.
	 * Lists by Level:
	 *  N=0         - Not intialized, just registered
	 *  N=1..MAX-1  - Reached init level N
	 *  N=MAX       - Successfully initialized bus
	 */
	struct rtems_drvmgr_list	devices[DRVMGR_LEVEL_MAX+1];	
	/*!< Devices failed to initialize, removed, ignored, no driver */
	struct rtems_drvmgr_list	devices_inactive;
};

extern struct rtems_driver_manager drv_mgr;
