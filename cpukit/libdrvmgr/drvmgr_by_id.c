#include <drvmgr/drvmgr.h>
#include "drvmgr_internal.h"

/* Get driver from driver name */
struct drvmgr_drv *drvmgr_drv_by_id(uint64_t id)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_drv *drv = NULL;

	/* NOTE: No locking is needed here since Driver list is supposed to be
	 *       initialized once during startup, we treat it as a static
	 *       read-only list
	 */

	drv = DRV_LIST_HEAD(&mgr->drivers);
	while (drv) {
		if (drv->drv_id == id)
			break;
		drv = drv->next;
	}

	return drv;
}
