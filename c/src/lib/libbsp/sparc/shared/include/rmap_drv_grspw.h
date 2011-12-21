/* GRSPW RMAP driver, used by RMAP stack to talk to the GRSPW driver.
 *
 *  COPYRIGHT (c) 2009
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __RMAP_DRV_GRSPW_H__
#define __RMAP_DRV_GRSPW_H__

#ifdef __cplusplus
extern "C" {
#endif

extern struct rmap_drv_ops rmap_grspw_ops;

struct rmap_drv_grspw_config {
	int fd;
};

void *rmap_drv_grspw_init(struct rmap_drv_grspw_config *config);

#ifdef __cplusplus
}
#endif

#endif
