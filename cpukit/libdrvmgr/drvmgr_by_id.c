#include <drvmgr/drvmgr.h>
#include "drvmgr_internal.h"

/* Get driver from driver name */
struct drvmgr_drv *drvmgr_drv_by_id(uint64_t id)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_drv *drv;

	drv = DRV_LIST_HEAD(&mgr->drivers);
	while (drv) {
		if (drv->drv_id == id)
			return drv;
		drv = drv->next;
	}

	return 0;
}
