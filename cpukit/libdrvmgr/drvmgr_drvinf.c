/*  Driver Manager Driver Interface Implementation.
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler AB
 *
 *  This is the part the device drivers use, the functions rely on that
 *  the parent bus driver has implemented the neccessary operations
 *  correctly.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drvmgr/drvmgr.h>

/* Get device pointer from knowing the Driver and the Driver minor 
 * that was assigned to it
 */
int rtems_drvmgr_get_dev(
	struct rtems_drvmgr_drv_info *drv,
	int minor,
	struct rtems_drvmgr_dev_info **pdev)
{
	struct rtems_drvmgr_dev_info *dev;
	if ( !drv )
		return -1;
	dev = drv->dev;
	while( dev ){
		if ( dev->minor_drv == minor)
			break;
		dev = dev->next_in_drv;
	}
	if ( !dev )
		return -1;
	if ( pdev )
		*pdev = dev;
	return 0;
}

/* Get Bus frequency in HZ from bus driver */
int rtems_drvmgr_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options, 
	unsigned int *freq_hz)
{
	if ( !dev || !dev->parent || !dev->parent->ops ||
	     !dev->parent->ops->freq_get)
		return -1;

	return dev->parent->ops->freq_get(dev, options, freq_hz);
}

/* Get driver prefix */
int rtems_drvmgr_get_dev_prefix(struct rtems_drvmgr_dev_info *dev, char *dev_prefix)
{
	struct rtems_drvmgr_bus_params params;
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->get_params)
		return -1;

	dev->parent->ops->get_params(dev, &params);
	if ( !params.dev_prefix )
		return -1;
	if ( dev_prefix )
		strcpy(dev_prefix, params.dev_prefix);
	return 0;
}

/* Register an interrupt */
int rtems_drvmgr_interrupt_register(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_register)
		return -1;

	if ( !isr )
		return -1;

	return dev->parent->ops->int_register(dev, index, isr, arg);
}

/* Unregister an interrupt */
int rtems_drvmgr_interrupt_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	/* Since it is a shared interrupt service, the handler being unregistered must be disabled. */
	rtems_drvmgr_interrupt_disable(dev, index, isr, arg);

	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_unregister)
		return -1;

	if ( !isr )
		return -1;

	return dev->parent->ops->int_unregister(dev, index, isr, arg);
}

int rtems_drvmgr_interrupt_enable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_enable)
		return -1;

	if ( !isr )
		return -1;

	return dev->parent->ops->int_enable(dev, index, isr, arg);
}

int rtems_drvmgr_interrupt_disable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_disable)
		return -1;

	if ( !isr )
		return -1;

	return dev->parent->ops->int_disable(dev, index, isr, arg);
}

int rtems_drvmgr_interrupt_clear(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_clear)
		return -1;

	if ( !isr )
		return -1;

	return dev->parent->ops->int_clear(dev, index, isr, arg);
}

int rtems_drvmgr_interrupt_unmask(
	struct rtems_drvmgr_dev_info *dev,
	int index)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_unmask)
		return -1;

	return dev->parent->ops->int_unmask(dev, index);
}

int rtems_drvmgr_interrupt_mask(
	struct rtems_drvmgr_dev_info *dev,
	int index)
{
	if ( !dev || !dev->parent || !dev->parent->ops || !dev->parent->ops->int_mask)
		return -1;

	return dev->parent->ops->int_mask(dev, index);
}

int rtems_drvmgr_on_rootbus(struct rtems_drvmgr_dev_info *dev)
{
	if ( dev->parent && dev->parent->dev && dev->parent->dev->parent )
		return 0;
	else
		return 1;
}
