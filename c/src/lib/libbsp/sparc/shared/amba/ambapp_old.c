/* Old AMBA scanning Interface provided for backwards compability */
#include <ambapp.h>

struct ambapp_dev_find_match_arg {
	int			index;
	int			count;
	int			type;
	void			*dev;
};

/* AMBA PP find routines */
int ambapp_dev_find_match(struct ambapp_dev *dev, int index, int maxdepth, void *arg)
{
	struct ambapp_dev_find_match_arg *p = arg;

	if ( p->index == 0 ) {
		/* Found controller, stop */
		if ( p->type == DEV_APB_SLV ) {
			*(struct ambapp_apb_info *)p->dev = 
				*(struct ambapp_apb_info *)&dev->devinfo[0];
			p->dev = ((struct ambapp_apb_info *)p->dev)+1;
		} else {
			*(struct ambapp_ahb_info *)p->dev = 
				*(struct ambapp_ahb_info *)&dev->devinfo[0];
			p->dev = ((struct ambapp_ahb_info *)p->dev)+1;
		}
		p->count--;
		if ( p->count < 1 )
			return 1;
	} else {
		p->index--;
	}
	return 0;
}

int ambapp_find_apbslvs_next(struct ambapp_bus *abus, int vendor, int device, struct ambapp_apb_info *dev, int index, int maxno)
{
	struct ambapp_dev_find_match_arg arg;

	arg.index = index;
	arg.count = maxno;
	arg.type = DEV_APB_SLV; /* APB */
	arg.dev = dev;

	ambapp_for_each(abus->root, (OPTIONS_ALL|OPTIONS_APB_SLVS), vendor, device, 10, ambapp_dev_find_match, &arg);

	return maxno - arg.count;
}

int ambapp_find_apbslv(struct ambapp_bus *abus, int vendor, int device, struct ambapp_apb_info *dev)
{
	return ambapp_find_apbslvs_next(abus, vendor, device, dev, 0, 1);
}

int ambapp_find_apbslv_next(struct ambapp_bus *abus, int vendor, int device, struct ambapp_apb_info *dev, int index)
{
	return ambapp_find_apbslvs_next(abus, vendor, device, dev, index, 1);
}

int ambapp_find_apbslvs(struct ambapp_bus *abus, int vendor, int device, struct ambapp_apb_info *dev, int maxno)
{
	return ambapp_find_apbslvs_next(abus, vendor, device, dev, 0, maxno);
}

int ambapp_find_ahbslvs_next(struct ambapp_bus *abus, int vendor, int device, struct ambapp_ahb_info *dev, int index, int maxno)
{
	struct ambapp_dev_find_match_arg arg;

	arg.index = index;
	arg.count = maxno;
	arg.type = DEV_AHB_SLV; /* AHB SLV */
	arg.dev = dev;

	ambapp_for_each(abus->root, (OPTIONS_ALL|OPTIONS_AHB_SLVS), vendor,
			device, 10, ambapp_dev_find_match, &arg);

	return maxno - arg.count;
}

int ambapp_find_ahbslv_next(struct ambapp_bus *abus, int vendor, int device, struct ambapp_ahb_info *dev, int index)
{
	return ambapp_find_ahbslvs_next(abus, vendor, device, dev, index, 1);
}

int ambapp_find_ahbslv(struct ambapp_bus *abus, int vendor, int device, struct ambapp_ahb_info *dev)
{
	return ambapp_find_ahbslvs_next(abus, vendor, device, dev, 0, 1);
}

int ambapp_find_ahbslvs(struct ambapp_bus *abus, int vendor, int device, struct ambapp_ahb_info *dev, int maxno)
{
	return ambapp_find_ahbslvs_next(abus, vendor, device, dev, 0, maxno);
}
