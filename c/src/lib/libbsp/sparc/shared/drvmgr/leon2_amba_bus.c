/*  LEON2 Hardcoded bus driver.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  Bus driver for a hardcoded setup. LEON2 systems have some
 *  cores always present, here called "Standard Cores". In 
 *  addtion to the standard cores there are often extra cores
 *  that can be defined using the "Custom Cores" mechanism.
 *
 *  A Core is described by assigning a base register and 
 *  IRQ0..IRQ15 using the leon2_core structure.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2008-12-03, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/leon2_amba_bus.h>

#include <bsp.h>

#define DBG(args...)
/*#define DBG(args...) printk(args)*/

struct rtems_drvmgr_drv_info leon2_bus_drv;

int leon2_amba_bus_init1(struct rtems_drvmgr_bus_info *bus);
int leon2_amba_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev);
int leon2_amba_int_register(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int leon2_amba_int_unregister(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int leon2_amba_int_enable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int leon2_amba_int_disable(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int leon2_amba_int_clear(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg);
int leon2_amba_int_mask(
	struct rtems_drvmgr_dev_info *dev,
	int index);
int leon2_amba_int_unmask(
	struct rtems_drvmgr_dev_info *dev,
	int index);
int leon2_amba_dev_id_compare(struct leon2_amba_dev_info *a, struct leon2_amba_dev_info *b);

/* LEON2 bus operations */
struct rtems_drvmgr_bus_ops leon2_amba_bus_ops =
{
	.init = {
		leon2_amba_bus_init1,
		NULL,
		NULL,
		NULL
	},
	.remove = NULL,
	.unite		= leon2_amba_unite,
	.dev_id_compare = (int(*)(void*,void*)) leon2_amba_dev_id_compare,
	.int_register	= leon2_amba_int_register,
	.int_unregister	= leon2_amba_int_unregister,
	.int_enable	= leon2_amba_int_enable,
	.int_disable	= leon2_amba_int_disable,
	.int_clear	= leon2_amba_int_clear,
	.int_mask	= leon2_amba_int_mask,
	.int_unmask	= leon2_amba_int_unmask,
	.get_params	= NULL,
};

struct leon2_isr_handler {
	void (*handler)(int irq, void *arg);
	void *arg;
};

/* Interrupt handlers */
struct leon2_isr_handler leon2_isrs[16];

/* Standard LEON2 configuration */

struct rtems_drvmgr_key leon2_timers[] =
{
	{"REG0", KEY_TYPE_INT, {0x80000040}},
	{"IRQ0", KEY_TYPE_INT, {8}},
	{"IRQ1", KEY_TYPE_INT, {9}},
	KEY_EMPTY
};

struct rtems_drvmgr_key leon2_uart1[] =
{
	{"REG0", KEY_TYPE_INT, {0x80000070}},
	{"IRQ0", KEY_TYPE_INT, {3}},
	KEY_EMPTY
};

struct rtems_drvmgr_key leon2_uart2[] =
{
	{"REG0", KEY_TYPE_INT, {0x80000080}},
	{"IRQ0", KEY_TYPE_INT, {2}},
	KEY_EMPTY
};

struct rtems_drvmgr_key leon2_irqctrl[] =
{
	{"REG0", KEY_TYPE_INT, {0x80000090}},
	KEY_EMPTY
};

struct rtems_drvmgr_key leon2_gpio0[] =
{
	{"REG0", KEY_TYPE_INT, {0x800000A0}},
	{"IRQ0", KEY_TYPE_INT, {4}},
	{"IRQ1", KEY_TYPE_INT, {5}},
	{"IRQ2", KEY_TYPE_INT, {6}},
	{"IRQ3", KEY_TYPE_INT, {7}},
	KEY_EMPTY
};

struct leon2_core drv_mgr_leon2_std_cores[] = 
{
	{{LEON2_AMBA_TIMER_ID}, "Timers", &leon2_timers[0]},
	{{LEON2_AMBA_UART_ID}, "Uart1", &leon2_uart1[0]},
	{{LEON2_AMBA_UART_ID}, "Uart2", &leon2_uart2[0]},
	{{LEON2_AMBA_IRQCTRL_ID}, "IRQCtrl", &leon2_irqctrl[0]},
	{{LEON2_AMBA_GPIO_ID}, "GPIO", &leon2_gpio0[0]},
	EMPTY_LEON2_CORE
};

static struct leon2_bus *drv_mgr_leon2_bus_config = NULL;
static struct rtems_drvmgr_bus_res *drv_mgr_leon2_bus_res = NULL;

int drv_mgr_leon2_init(struct leon2_bus *bus_config, struct rtems_drvmgr_bus_res *resources)
{
	/* Save the configuration for later */
	drv_mgr_leon2_bus_config = bus_config;
	drv_mgr_leon2_bus_res = resources;

	/* Register root device driver */
	rtems_drvmgr_root_drv_register(&leon2_bus_drv);

	return 0;
}

int leon2_amba_dev_register(struct rtems_drvmgr_bus_info *bus, struct leon2_core *core, int index)
{
	struct rtems_drvmgr_dev_info *newdev;
	struct leon2_amba_dev_info *info;
	union rtems_drvmgr_key_value *value;
	char irq_name[8];
	int i;

	/* Allocate new device and businfo */
	rtems_drvmgr_alloc_dev(&newdev, sizeof(struct leon2_amba_dev_info));
	info = (struct leon2_amba_dev_info *)(newdev + 1);

	/* Set Core ID */
	info->core_id = core->id.core_id;

	/* Get information from bus configuration */
	value = rtems_drvmgr_key_val_get(core->keys, "REG0", KEY_TYPE_INT);
	if ( !value ) {
		printk("leon2_amba_dev_register: Failed getting resource REG0\n");
		info->reg_base = 0x00000000;
	} else {
		DBG("leon2_amba_dev_register: REG0: 0x%08x\n", value->i);
		info->reg_base = value->i;
	}

	strcpy(irq_name, "IRQ");
	for(i=0; i<16; i++){
		if ( i < 10 ){
			irq_name[3] = '0' + i;
			irq_name[4] = '\0';
		} else {
			irq_name[3] = '1';
			irq_name[4] = '0' + (i-10);
			irq_name[5] = '\0';
		}

		value = rtems_drvmgr_key_val_get(core->keys, irq_name, KEY_TYPE_INT);
		if ( !value ) {
			DBG("leon2_amba_dev_register: Failed getting resource IRQ%d for REG 0x%x\n", i, info->reg_base);
			info->irqs[i] = 0;
		} else {
			DBG("leon2_amba_dev_register: IRQ%d: %d\n", i, value->i);
			info->irqs[i] = value->i;
		}
	}

	/* Init new device */
	newdev->next = NULL;
	newdev->parent = bus; /* Ourselfs */
	newdev->minor_drv = 0;
	newdev->minor_bus = 0;
	newdev->businfo = (void *)info;
	newdev->priv = NULL;
	newdev->drv = NULL;
	newdev->name = core->name;
	newdev->next_in_drv = NULL;
	newdev->bus = NULL;

	/* Register new device */
	rtems_drvmgr_dev_register(newdev);

	return 0;
}

int leon2_amba_init1(struct rtems_drvmgr_dev_info *dev)
{
	/* Init our own device */
	dev->priv = NULL;
	dev->name = "LEON2 AMBA";

	memset(leon2_isrs, 0, sizeof(leon2_isrs));

	/* Init the bus */
	rtems_drvmgr_alloc_bus(&dev->bus, 0);
	dev->bus->bus_type = DRVMGR_BUS_TYPE_LEON2_AMBA;
	dev->bus->next = NULL;
	dev->bus->dev = dev;
	dev->bus->priv = NULL;
	dev->bus->children = NULL;
	dev->bus->ops = &leon2_amba_bus_ops;
	dev->bus->dev_cnt = 0;
	dev->bus->reslist = NULL;
	dev->bus->mmaps = drv_mgr_leon2_bus_config->mmaps;
	rtems_drvmgr_bus_register(dev->bus);

	return DRVMGR_OK;
}

int leon2_amba_init2(struct rtems_drvmgr_dev_info *dev)
{
	return DRVMGR_OK;
}

int leon2_amba_remove(struct rtems_drvmgr_dev_info *dev)
{
	return DRVMGR_OK;
}

int leon2_amba_bus_init1(struct rtems_drvmgr_bus_info *bus)
{
	struct leon2_core *core;
	int i;

	if ( drv_mgr_leon2_bus_res )
		rtems_drvmgr_bus_res_add(bus, drv_mgr_leon2_bus_res);

	/**** REGISTER NEW DEVICES ****/
	i=0;
	core = drv_mgr_leon2_bus_config->std_cores;
	if ( core ) {
		while ( core->id.core_id ) {
			if ( leon2_amba_dev_register(bus, core, i) ) {
				return RTEMS_UNSATISFIED;
			}
			i++;
			core++;
		}
	}
	core = drv_mgr_leon2_bus_config->custom_cores;
	if ( core ) {
		while ( core->id.core_id ) {
			if ( leon2_amba_dev_register(bus, core, i) ) {
				return RTEMS_UNSATISFIED;
			}
			i++;
			core++;
		}
	}

	return 0;
}

int leon2_amba_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev)
{
	struct leon2_amba_dev_info *info;
	struct leon2_amba_drv_info *adrv;
	struct leon2_amba_dev_id *id;

	if ( !drv || !dev || !dev->parent )
		return 0;

	if ( (drv->bus_type!=DRVMGR_BUS_TYPE_LEON2_AMBA) || (dev->parent->bus_type != DRVMGR_BUS_TYPE_LEON2_AMBA) ) {
		return 0;
	}

	info = (struct leon2_amba_dev_info *)dev->businfo;
	if ( !info ) 
		return 0;

	/* Get LEON2 AMBA driver info */
	adrv = (struct leon2_amba_drv_info *)drv;
	id = adrv->ids;
	if ( !id )
		return 0;

	while ( id->core_id ) {
		if ( id->core_id == info->core_id ) {
			/* Driver is suitable for device, Unite them */
			return 1;
		}
		id++;
	}

	return 0;
}

int leon2_amba_dev_id_compare(struct leon2_amba_dev_info *a, struct leon2_amba_dev_info *b)
{
	return (a->core_id - b->core_id);
}

rtems_isr leon2_amba_isr(rtems_vector_number v)
{
	int irq = v - 0x10; /* Convert Vector number to Interrupt number */
	struct leon2_isr_handler *isr;
	
	isr = &leon2_isrs[irq];
	if ( isr->handler ) {
		isr->handler(irq, isr->arg);
	}
}

int leon2_amba_get_irq(struct rtems_drvmgr_dev_info *dev, int index)
{
	int irq;
	struct leon2_amba_dev_info *info;

	if ( !dev || (index > 15) )
		return -1;

	/* Relative (positive) or absolute (negative) IRQ number */
	if ( index >= 0 ) {
		/* IRQ Index relative to Cores base IRQ */

		/* Get IRQ array configured by user */
		info = (struct leon2_amba_dev_info *)dev->businfo;
		irq = info->irqs[index];
		if ( irq == 0 )
			return -1;
	} else {
		/* Absolute IRQ number */
		irq = -index;
	}
	return irq;
}

int leon2_amba_int_register
	(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	DBG("Registering IRQ %d to func 0x%x arg 0x%x\n", irq, (unsigned int)isr, (unsigned int)arg);

	return BSP_shared_interrupt_register(irq, isr, arg);
}

int leon2_amba_int_unregister
	(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	DBG("Unregistering IRQ %d to func 0x%x arg 0x%x\n", irq, (unsigned int)handler, (unsigned int)arg);

	return BSP_shared_interrupt_unregister(irq, isr, arg);
}

/* Enable interrupt */
int leon2_amba_int_enable
	(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	return BSP_shared_interrupt_enable(irq, isr, arg);
}

/* Disable interrupt */
int leon2_amba_int_disable
	(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	return BSP_shared_interrupt_disable(irq, isr, arg);
}

int leon2_amba_int_clear
	(
	struct rtems_drvmgr_dev_info *dev,
	int index,
	rtems_drvmgr_isr isr,
	void *arg
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	return BSP_shared_interrupt_clear(irq, isr, arg);
}

int leon2_amba_int_mask
	(
	struct rtems_drvmgr_dev_info *dev,
	int index
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	BSP_shared_interrupt_mask(irq);

	return 0;
}

int leon2_amba_int_unmask
	(
	struct rtems_drvmgr_dev_info *dev,
	int index
	)
{
	int irq;

	irq = leon2_amba_get_irq(dev, index);
	if ( irq < 0 )
		return -1;

	BSP_shared_interrupt_unmask(irq);

	return 0;
}

struct rtems_drvmgr_drv_ops leon2_amba_ops = 
{
	.init = {leon2_amba_init1, leon2_amba_init2, NULL, NULL},
	.remove = leon2_amba_remove,
	.info = NULL
};

struct rtems_drvmgr_drv_info leon2_bus_drv =
{
	DRVMGR_OBJ_DRV,			/* Driver */
	NULL,				/* Next driver */
	NULL,				/* Device list */
	DRIVER_LEON2_AMBA_ID,		/* Driver ID */
	"LEON2_AMBA_DRV",		/* Must be placed at top bus */
	DRVMGR_BUS_TYPE_ROOT,		/* Bus Type */
	&leon2_amba_ops,		/* Bus Operations */
	0,				/* Device Count */
	0,				/* Private structure sizee */
};
