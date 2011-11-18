/*  Driver Manager Driver Resource Interface Implementation.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler AB
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#include <drvmgr/drvmgr.h>

/* Find all the resource keys for a device among all bus resources */
int rtems_drvmgr_keys_get(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_key **keys)
{
	struct rtems_drvmgr_bus_info *bus;
	struct rtems_drvmgr_bus_res *node;
	struct rtems_drvmgr_drv_res *res;
	uint64_t drv_id;

	bus = dev->parent;
	if ( !bus || !dev->drv )
		return -1;

	drv_id = dev->drv->drv_id;

	/* Loop all resource arrays */
	node = bus->reslist;
	while ( node ) {
		/* Find driver ID in resource array */
		res = node->resource;
		while ( res->drv_id ) {
			if ( res->drv_id == drv_id ) {
				/* Found resource matching driver, now check
				 * that this resource is for this device.
				 */
				if ( dev->minor_bus == res->minor_bus ) {
					/* Matching driver and core number */
					if ( keys ) {
						*keys = res->keys;
					}
					return 0;
				}
			}
			res++;
		}
		node = node->next;
	}
	if ( keys ) {
		*keys = NULL;
	}
	return 1;
}

/* Return key that matches key name */
struct rtems_drvmgr_key *rtems_drvmgr_key_get(
	struct rtems_drvmgr_key *keys,
	char *key_name)
{
	struct rtems_drvmgr_key *key;

	if ( !keys )
		return NULL;

	key = keys;
	while ( key->key_type != KEY_TYPE_NONE) {
		if ( strcmp(key_name, key->key_name) == 0 ) {
			return key;
		}
		key++;
	}
	return NULL;
}

union rtems_drvmgr_key_value *rtems_drvmgr_key_val_get(
	struct rtems_drvmgr_key *keys,
	char *key_name,
	int key_type)
{
	struct rtems_drvmgr_key *key_match;
	
	key_match = rtems_drvmgr_key_get(keys, key_name);
	if ( key_match ) {
		/* Found key, put pointer to value into */
		if ( (key_type == -1) || (key_match->key_type == key_type) ) {
			return &key_match->key_value;
		}
	}
	return NULL;
}

union rtems_drvmgr_key_value *rtems_drvmgr_dev_key_get(
	struct rtems_drvmgr_dev_info *dev,
	char *key_name,
	int key_type)
{
	struct rtems_drvmgr_key *keys = NULL;

	/* Find first entry in key array for the device */
	if ( rtems_drvmgr_keys_get(dev, &keys) ) {
		return NULL;
	}

	/* Find a specific key among the device keys */
	return rtems_drvmgr_key_val_get(keys, key_name, key_type);
}
