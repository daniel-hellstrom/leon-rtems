#include <string.h>
#include <drvmgr/drvmgr.h>
#include "drvmgr_internal.h"

/* Get driver from driver name */
struct drvmgr_drv *drvmgr_drv_by_name(const char *name)
{
	struct rtems_driver_manager *mgr = &drv_mgr;
	struct drvmgr_drv *drv;

	if (!name)
		return NULL;

	drv = DRV_LIST_HEAD(&mgr->drivers);
	while (drv) {
		if (drv->name && (strcmp(drv->name, name) == 0))
			return drv;
		drv = drv->next;
	}

	return 0;
}
