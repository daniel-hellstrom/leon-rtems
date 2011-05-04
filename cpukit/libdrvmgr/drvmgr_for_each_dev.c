#include <drvmgr/drvmgr.h>
#include <drvmgr/drvmgr_list.h>

int rtems_drvmgr_for_each_dev(
	struct rtems_drvmgr_list *devlist,
	unsigned int state_set_mask,
	unsigned int state_clr_mask,
	int (*func)(struct rtems_drvmgr_dev_info *dev, void *arg),
	void *arg
	)
{
	struct rtems_drvmgr_dev_info *dev;
	int ret;

	/* Get First Device */
	dev = DEV_LIST_HEAD(devlist);
	while ( dev ) {
		if ( ((state_set_mask != 0) && ((dev->state & state_set_mask) == state_set_mask)) ||
		     ((state_clr_mask != 0) && ((dev->state & state_clr_mask) == 0)) ||
		     ((state_set_mask == 0) && (state_clr_mask == 0)) )
			if ( (ret=func(dev, arg)) != 0 )
				return ret;
		dev = dev->next;
	}
	return 0;
}
