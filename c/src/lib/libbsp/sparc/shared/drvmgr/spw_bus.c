/*  SpaceWire bus driver interface.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE. 
 *
 *  2009-11-20, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/spw_bus.h>
#include <genirq.h>
#include <gpiolib.h>
#include <rmap.h>

#include <bsp.h>
#include <stdint.h>

#undef DEBUG

#ifdef DEBUG
 #define DBG(args...) printk(args)
#else
 #define DBG(args...)
#endif

struct virq_entry {
	char *gpio_fsname;
	int fd;
};

struct spw_bus_priv {
	/* SpW-Bus driver handle */
	struct rtems_drvmgr_bus_info *bus;

	/* User configuration */
	struct spw_bus_config	*config;		/* User configuration */

	/* Device prefix */
	int			spwbus_id;
	char			prefix[16];		/* Device name prefix */

	/* IRQ Handling */
	volatile unsigned int	irq_mask;		/* Bit mask of which IRQs has been received, and to handle */
	genirq_t		genirq;			/* Shared IRQ, ISR handling */
	rtems_id		irqlock;		/* Protect IRQ handling (register,enable,disable etc.) */
	rtems_id		isr_task;		/* TASK used to execute registered ISRs on SpaceWire Bus */
	rtems_id		isr_execute_sem;	/* Used to signal to ISR TASK that one or more IRQs has been received */
	volatile int		task_terminate;		/* Used to signal to ISR TASK to stop it's execution, and delete itself */
	char			virqs[4];		/* Virtual IRQ table, used to separate IRQ from different GPIO pins */
};

struct rtems_drvmgr_drv_info spw_bus_drv;
static int spw_bus_cnt = 0;

int spw_bus_init1(struct rtems_drvmgr_bus_info *bus);
int spw_bus_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev);
int spw_bus_int_register(struct rtems_drvmgr_dev_info *dev, int index, void (*handler)(int,void*), void *arg);
int spw_bus_int_unregister(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int spw_bus_int_enable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int spw_bus_int_disable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int spw_bus_int_clear(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg);
int spw_bus_dev_id_compare(struct spw_bus_dev_info *a, struct spw_bus_dev_info *b);

int spw_bus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz);
/* READ/WRITE access to SpaceWire target over RMAP */
int spw_bus_memcpy(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n);
int spw_bus_write_mem(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n);
int spw_bus_read_io8(struct rtems_drvmgr_dev_info *dev, uint8_t *srcadr, uint8_t *result);
int spw_bus_read_io16(struct rtems_drvmgr_dev_info *dev, uint16_t *srcadr, uint16_t *result);
int spw_bus_read_io32(struct rtems_drvmgr_dev_info *dev, uint32_t *srcadr, uint32_t *result);
int spw_bus_read_io64(struct rtems_drvmgr_dev_info *dev, uint64_t *srcadr, uint64_t *result);
int spw_bus_write_io8(struct rtems_drvmgr_dev_info *dev, uint8_t *dstadr, uint8_t data);
int spw_bus_write_io16(struct rtems_drvmgr_dev_info *dev, uint16_t *dstadr, uint16_t data);
int spw_bus_write_io32(struct rtems_drvmgr_dev_info *dev, uint32_t *dstadr, uint32_t data);
int spw_bus_write_io64(struct rtems_drvmgr_dev_info *dev, uint64_t *dstadr, uint64_t data);
int spw_bus_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params);

/* SPW RMAP bus operations */
struct rtems_drvmgr_bus_ops spw_bus_ops =
{
	.init		= 
			{
				spw_bus_init1,
				NULL,
				NULL,
				NULL
			},
	.remove		= NULL,
	.unite		= spw_bus_unite,
	.dev_id_compare = (int(*)(void*,void*)) spw_bus_dev_id_compare,
	.int_register	= spw_bus_int_register,
	.int_unregister	= spw_bus_int_unregister,
	.int_enable	= spw_bus_int_enable,
	.int_disable	= spw_bus_int_disable,
	.int_clear	= spw_bus_int_clear,
	.get_params	= spw_bus_get_params,
	.freq_get	= spw_bus_freq_get,

	.read_io8	= spw_bus_read_io8,
	.read_io16	= spw_bus_read_io16,
	.read_io32	= spw_bus_read_io32,
	.read_io64	= spw_bus_read_io64,
	.write_io8	= spw_bus_write_io8,
	.write_io16	= spw_bus_write_io16,
	.write_io32	= spw_bus_write_io32,
	.write_io64	= spw_bus_write_io64,
	.read_mem	= spw_bus_memcpy,
	.write_mem	= spw_bus_write_mem,
};

int spw_bus_dev_register(struct rtems_drvmgr_bus_info *bus, struct spw_node *node, int index)
{
	struct rtems_drvmgr_dev_info *newdev;
	struct spw_bus_dev_info *info;
	union rtems_drvmgr_key_value *value;
	
	int virq;
	char virq_name[6];

	/* Allocate new device and bus information */
	rtems_drvmgr_alloc_dev(&newdev, sizeof(struct spw_bus_dev_info));
	info = (struct spw_bus_dev_info *)(newdev + 1);

	/* Set Node ID */
	info->spwid = node->id.spwid;

	/* Get information from bus configuration */
	value = rtems_drvmgr_key_val_get(node->keys, "DST_ADR", KEY_TYPE_INT);
	if ( !value ) {
		printk("spw_bus_dev_register: Failed getting resource DST_ADR\n");
		info->dstadr = 0xfe;
	} else {
		DBG("spw_bus_dev_register: DST_ADR: 0x%02x\n", value->i);
		info->dstadr = value->i;
	}
	value = rtems_drvmgr_key_val_get(node->keys, "DST_KEY", KEY_TYPE_INT);
	if ( !value ) {
		printk("spw_bus_dev_register: Failed getting resource DST_KEY\n");
		info->dstkey = 0;
	} else {
		DBG("spw_bus_dev_register: DST_KEY: 0x%02x\n", value->i);
		info->dstkey = value->i;
	}
	/* Get the Virtual IRQ numbers, that will be looked up in VIRQ->GPIO table */
	strcpy(virq_name, "VIRQX");
	for (virq=1; virq<5; virq++) {
		virq_name[4] = '0' + virq;
		value = rtems_drvmgr_key_val_get(node->keys, virq_name, KEY_TYPE_INT);
		if ( !value ) {
			/* IRQ is optional, this device does not support VIRQ[X] */
			info->virqs[virq-1] = -1;
		} else {
			DBG("spw_bus_dev_register: %s: %d\n", virq_name, value->i);
			info->virqs[virq-1] = value->i;
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
	newdev->name = node->name;
	newdev->next_in_drv = NULL;
	newdev->next_in_bus = newdev->parent->children;
	newdev->parent->children = newdev;
	newdev->bus = NULL;

	/* Register new device */
	rtems_drvmgr_dev_register(newdev);

	return 0;
}

/* Interrupt Service Routine, executes in interrupt context. This ISR:
 *  1. Disable/Mask IRQ on IRQ controller, this disables further interrupts on this IRQ number
 *  2. Mark in the private struct that the IRQ has happened
 *  3. Wake ISR TASK that will handle each marked IRQ
 *
 * The TASK will for every IRQ that is marked:
 *  1. Call the ISRs that the SpW Node drivers have registered for this specific IRQ
 *  2. unmask IRQ again.
 *
 *  wakes */
void spw_bus_isr(int irq, void *arg)
{
	struct spw_bus_priv *priv;
	unsigned int old_irq_mask;
	char *pvirq = (char *)arg;
	int virq = *pvirq;

	priv = (struct spw_bus_priv *)(pvirq - offsetof(struct spw_bus_priv, virqs) - (virq-1));

	/*rtems_drvmgr_interrupt_mask(priv->bus->dev, -irq);*/
	gpiolib_irq_disable(priv->config->virq_table[virq-1].handle);

	/* Mark IRQ was received */
	old_irq_mask = priv->irq_mask;
	priv->irq_mask = old_irq_mask | (1 << virq);

	/* Wake ISR execution TASK only if not woken before */
	if ( old_irq_mask == 0 ) {
		rtems_semaphore_release(priv->isr_execute_sem);
	}
}

void spwbus_task(rtems_task_argument argument)
{
	int virq;
	rtems_interrupt_level level;
	unsigned int mask;
	struct spw_bus_priv *priv = (struct spw_bus_priv *)argument;

	DBG("SpW-Bus: ISR Task is started\n");

	while ( priv->task_terminate == 0 ) {
		while ( (mask = priv->irq_mask) != 0 ) {

			/* Mark the IRQs handled */
			rtems_interrupt_disable(level);
			priv->irq_mask &= ~mask;
			rtems_interrupt_enable(level);

			mask = mask >> 1;
			virq = 1;
			while ( mask ) {
				if ( mask & 1 )  {
					/* execute all ISRs on this IRQ */
					DBG("SpW-Bus: ISR Task is executing VIRQ %d\n", virq);
					genirq_doirq(priv->genirq, virq);
				}

				/* Reenable the handled IRQ */
				/*rtems_drvmgr_interrupt_unmask(priv->bus->dev, -irq);*/
				gpiolib_irq_enable(priv->config->virq_table[virq-1].handle);

				virq++;
				mask = mask >> 1;
			}
		}

		DBG("SpW-Bus: ISR Task going to sleep\n");

		rtems_task_wake_after(1);

		/* Wait for new IRQs to handle */
		rtems_semaphore_obtain(priv->isr_execute_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

		DBG("SpW-Bus: ISR Task woke up\n");
	}

	DBG("SpW-Bus: ISR Task is deleted\n");

	rtems_task_delete(RTEMS_SELF);
}

int spw_bus_init1(struct rtems_drvmgr_bus_info *bus)
{
	struct spw_node *node;
	int i;
	struct spw_bus_priv *priv = (struct spw_bus_priv *)bus->priv;
	int status;
	struct spwbus_virq_config *vcfg;
	int virq;
	struct gpiolib_config gpiocfg;

	DBG("SpW-BUS: init\n");

	priv->spwbus_id = spw_bus_cnt++;
	priv->irq_mask = 0;
	priv->task_terminate = 0;

	priv->genirq = genirq_init(32);
	if ( priv->genirq == NULL ) {
		return RTEMS_UNSATISFIED;
	}

	if ( priv->config->resources )
		rtems_drvmgr_bus_res_add(bus, priv->config->resources);

	/* Create Semaphore used when doing IRQ/ISR registering/enabling etc. */
	status = rtems_semaphore_create(
		rtems_build_name('S', 'P', 'C', '0' + bus->dev->minor_drv),
		1,
		RTEMS_FIFO | RTEMS_COUNTING_SEMAPHORE | RTEMS_NO_INHERIT_PRIORITY | \
		RTEMS_LOCAL | RTEMS_NO_PRIORITY_CEILING,
		0,
		&priv->irqlock);
	if ( status != RTEMS_SUCCESSFUL ) {
		DBG("SpW-BUS: Failed to create irqlock semaphore: %d\n", status);
		return -1;
	}

	/* Create Semaphore used to Signal to ISR TASK that a Interrupt has happened */
	status = rtems_semaphore_create(
		rtems_build_name('S', 'P', 'D', '0' + bus->dev->minor_drv),
		0,
		RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_NO_INHERIT_PRIORITY | \
		RTEMS_LOCAL | RTEMS_NO_PRIORITY_CEILING,
		0,
		&priv->isr_execute_sem);
	if ( status != RTEMS_SUCCESSFUL ) {
		DBG("SpW-BUS: Failed to create irqlock semaphore: %d\n", status);
		return -1;
	}

	/* Create ISR task */
	status = rtems_task_create(
		rtems_build_name( 'I', 'S', 'T', '0' + bus->dev->minor_drv),
		1,
		RTEMS_MINIMUM_STACK_SIZE,
		RTEMS_NO_PREEMPT,
		RTEMS_LOCAL | RTEMS_NO_FLOATING_POINT,
		&priv->isr_task);
	if (status != RTEMS_SUCCESSFUL) {
		DBG ("SpW-BUS: Can't create task: %d\n", status);
		return -1;
	}

	/* Initialize VIRQs and open the GPIO drivers if not already done */
	vcfg = &priv->config->virq_table[0];
	for (virq=1; virq<5; virq++) {
		priv->virqs[virq-1] = -1;
		if ( vcfg->handle == NULL ) {
			/* Open GPIO PIN and diable IRQ for now */
			if ( vcfg->gpio_fsname ) {
				vcfg->handle = gpiolib_open_by_name(vcfg->gpio_fsname);
				if ( vcfg->handle != NULL ) {
					priv->virqs[virq-1] = virq;
					gpiocfg.mask = 0;
					gpiocfg.irq_level = GPIOLIB_IRQ_LEVEL;
					gpiocfg.irq_polarity = GPIOLIB_IRQ_POL_HIGH;
					gpiolib_set_config(vcfg->handle, &gpiocfg);
					if ( gpiolib_set(vcfg->handle, 0, 0) ) {
						DBG("SpW-BUS: Failed to configure GPIO as input for VIRQ%d\n", virq);
					}
					if ( gpiolib_irq_register(vcfg->handle, spw_bus_isr, &priv->virqs[virq-1]) ) {
						DBG("SpW-BUS: Failed to register GPIO ISR for VIRQ%d\n", virq);
					}
				} else {
					DBG("SpW-BUS: Failed to open GPIO (%s) for VIRQ%d\n",vcfg->gpio_fsname, virq);
				}
			}
		} else {
			/* Already opened for us */
			priv->virqs[virq-1] = virq;
		}
		vcfg++;
	}

	/* Start ISR Task, there is no work to do, so the task will wait for isr_execute_sem semaphore released by real ISR */
	status = rtems_task_start(priv->isr_task, spwbus_task, (int)priv);
	if ( status != RTEMS_SUCCESSFUL ) {
		DBG("SpW-BUS: Failed to start ISR task: %d\n", status);
		return -1;
	}

	/* Create Device name */
	strcpy(priv->prefix, "/dev/spwbus0");
	priv->prefix[11] = '0' + priv->spwbus_id;
	mkdir(priv->prefix, S_IRWXU | S_IRWXG | S_IRWXO);
	priv->prefix[12] = '/';
	priv->prefix[13] = '\0';

	/**** REGISTER NEW DEVICES ****/
	i=0;
	node = priv->config->nodes;
	if ( node ) {
		while ( node->id.spwid ) {
			DBG("SpW-BUS: register node %d (%p)\n", i, node);
			if ( spw_bus_dev_register(bus, node, i) ) {
				return RTEMS_UNSATISFIED;
			}
			i++;
			node++;
		}
	}

	return DRVMGR_OK;
}

int spw_bus_unite(struct rtems_drvmgr_drv_info *drv, struct rtems_drvmgr_dev_info *dev)
{
	struct spw_bus_dev_info *info;
	struct spw_bus_drv_info *spwdrv;
	struct spw_id *id;

	if ( !drv || !dev || !dev->parent )
		return 0;

	if ( (drv->bus_type!=DRVMGR_BUS_TYPE_SPW_RMAP) || (dev->parent->bus_type != DRVMGR_BUS_TYPE_SPW_RMAP) ) {
		return 0;
	}

	info = (struct spw_bus_dev_info *)dev->businfo;
	if ( !info ) 
		return 0;

	/* Get SPW RMAP driver info */
	spwdrv = (struct spw_bus_drv_info *)drv;
	id = spwdrv->ids;
	if ( !id )
		return 0;

	while ( id->spwid ) {
		if ( id->spwid == info->spwid ) {
			/* Driver is suitable for device, Unite them */
			return 1;
		}
		id++;
	}

	return 0;
}

int spw_bus_dev_id_compare(struct spw_bus_dev_info *a, struct spw_bus_dev_info *b)
{
	return (a->spwid - b->spwid);
}

int spw_bus_int_get(struct rtems_drvmgr_dev_info *dev, int index)
{
	int virq;

	/* Relative (positive) or absolute (negative) IRQ number */
	if ( index >= 0 ) {
		/* IRQ Index relative to Cores base IRQ */

		/* Get Base IRQ */
		virq = ((struct spw_bus_dev_info *)dev->businfo)->virqs[index];
		if ( virq <= 0 )
			return -1;
		virq += index;
	} else {
		/* Absolute IRQ number */
		virq = -index;
	}
	return virq;
}

int spw_bus_int_register(struct rtems_drvmgr_dev_info *dev, int index, void (*handler)(int,void*), void *arg)
{
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;
	int status, virq;
	void *handle;

	/* Get IRQ number from index and device information */
	virq = spw_bus_int_get(dev, index);
	if ( virq <= 0 )
		return -1;

	bus = dev->parent;
	priv = bus->priv;
	
	DBG("SpW-BUS: Register ISR for VIRQ%d\n", virq);

	handle = priv->config->virq_table[virq-1].handle;
	if ( handle == NULL )
		return -1;

	rtems_semaphore_obtain(priv->irqlock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

	status = genirq_register(priv->genirq, virq, handler, arg);
	if ( status == 0 ) {
		/* Register a ISR for the first registered handler */

		/* Unmask the GPIO IRQ at the source (at the GPIO core), it is still masked by the IRQ
		 * controller, it will be enabled later.
		 */
		struct gpiolib_config gpiocfg;
		gpiocfg.mask = 1;
		gpiocfg.irq_level = GPIOLIB_IRQ_LEVEL;
		gpiocfg.irq_polarity = GPIOLIB_IRQ_POL_HIGH;
		gpiolib_set_config(handle, &gpiocfg);

		/* Already done 
		gpioLib_ (priv->virq_table[virq].fd, 
		rtems_drvmgr_interrupt_register(bus->dev, -irq, spw_bus_isr, priv);
		*/
	}

	rtems_semaphore_release(priv->irqlock);

	return 0;
}

int spw_bus_int_unregister(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg)
{
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;
	int virq, status;
	void *handle;

	/* Get IRQ number from index and device information */
	virq = spw_bus_int_get(dev, index);
	if ( virq <= 0 )
		return -1;
		
	DBG("SpW-BUS: unregister ISR for VIRQ%d\n", virq);

	bus = dev->parent;
	priv = bus->priv;
	
	handle = priv->config->virq_table[virq-1].handle;
	if ( handle == NULL )
		return -1;

	rtems_semaphore_obtain(priv->irqlock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

	status = genirq_unregister(priv->genirq, virq, isr, arg);
	if ( status == 0 ) {
		/* Register a ISR for the first registered handler */
		/*rtems_drvmgr_interrupt_unregister(bus->dev, -irq, spw_bus_isr, priv);*/
	}

	rtems_semaphore_release(priv->irqlock);

	return 0;
}

/* Enable interrupt */
int spw_bus_int_enable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg)
{
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;
	int virq, status;
	void *handle;

	/* Get IRQ number from index and device information */
	virq = spw_bus_int_get(dev, index);
	if ( virq <= 0 )
		return -1;

	bus = dev->parent;
	priv = bus->priv;
	
	DBG("SpW-BUS: Enable IRQ for VIRQ%d\n", virq);

	handle = priv->config->virq_table[virq-1].handle;
	if ( handle == NULL )
		return -1;

	rtems_semaphore_obtain(priv->irqlock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

	status = genirq_enable(priv->genirq, virq, isr, arg);
	if ( status == 0 ) {
		/* Register a ISR for the first registered handler */
		if ( gpiolib_irq_enable(handle) ) {
			DBG("SpW-BUS: Failed to Enable IRQ for VIRQ%d\n", virq);
		}
		/*
		rtems_drvmgr_interrupt_enable(bus->dev, -irq, spw_bus_isr, priv);
		*/
	}

	rtems_semaphore_release(priv->irqlock);

	return 0;
}

/* Disable interrupt */
int spw_bus_int_disable(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg)
{
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;
	int virq, status;
	void *handle;

	/* Get IRQ number from index and device information */
	virq = spw_bus_int_get(dev, index);
	if ( virq <= 0 )
		return -1;

	bus = dev->parent;
	priv = bus->priv;

	handle = priv->config->virq_table[virq-1].handle;
	if ( handle == NULL )
		return -1;

	rtems_semaphore_obtain(priv->irqlock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

	status = genirq_disable(priv->genirq, virq, isr, arg);
	if ( status == 0 ) {
		/* Register a ISR for the first registered handler */
		if ( gpiolib_irq_disable(handle) ) {
			DBG("SpW-BUS: Failed to Disable IRQ for VIRQ%d\n", virq);
		}
		/*rtems_drvmgr_interrupt_disable(bus->dev, -irq, spw_bus_isr, priv);*/
	}

	rtems_semaphore_release(priv->irqlock);

	return 0;
}

int spw_bus_int_clear(struct rtems_drvmgr_dev_info *dev, int index, rtems_drvmgr_isr isr, void *arg)
{
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;
	int virq;
	void *handle;

	/* Get IRQ number from index and device information */
	virq = spw_bus_int_get(dev, index);
	if ( virq < 0 )
		return -1;

	bus = dev->parent;
	priv = bus->priv;

	handle = priv->config->virq_table[virq-1].handle;
	if ( handle == NULL )
		return -1;

	/* Register a ISR for the first registered handler */
	/*rtems_drvmgr_interrupt_clear(bus->dev, -irq, spw_bus_isr, priv);*/
	if ( gpiolib_irq_clear(handle)) {
		DBG("SpW-BUS: Failed to Clear IRQ for VIRQ%d\n", virq);
	}

	return 0;
}

/* Copy */
int spw_bus_memcpy(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n)
{
	struct rmap_command_read readcmd;
	int status;
	struct spw_bus_dev_info *info = (struct spw_bus_dev_info *)dev->businfo;
	struct spw_bus_priv *priv = (struct spw_bus_priv *)dev->parent->priv;
	int max_pkt_size = 128; /* We assume that RMAP can do at least 128 bytes data per packet */

	unsigned int source, destination, left;

	if (  n > max_pkt_size ) {
		struct rmap_config stack_cfg;
		rmap_ioctl(priv->config->rmap, RMAP_IOCTL_GET_CONFIG, &stack_cfg);
		max_pkt_size = stack_cfg.max_rx_len;
	}

	source = (unsigned int)src;
	destination = (unsigned int )dest;
	left = n;
	while ( left > 0 ) {
		readcmd.type = RMAP_CMD_RI;
		readcmd.dstadr = info->dstadr;
		readcmd.dstkey = info->dstkey;
		readcmd.address = source;
		readcmd.length = (left > max_pkt_size) ? max_pkt_size : left;
		readcmd.datalength = 0;
		readcmd.data = (void *)destination;

		DBG("RMAP READ1: 0x%08x - 0x%08x\n", source, (source + (readcmd.length - 1)));

		/* Send Command */
		status = rmap_send(priv->config->rmap, (struct rmap_command *)&readcmd);

		if ( status ) {
			printf("RMAP_MEMCPY READ: Failed to send/receive command %d\n", status);
			memset(dest, 0, n);
			/* Should we remove device? */
			return -1;
		}

		/* Read Data */
		if ( readcmd.status != 0 ) {
			printf("RMAP_MEMCPY READ: Status non-zero 0x%x, dlen: 0x%x\n", readcmd.status, readcmd.datalength);
			memset(dest, 0, n);
			/* Should we remove device? */
			return -1;
		}

		source += readcmd.length;
		destination += readcmd.length;
		left -= readcmd.length;
	}
	
#ifdef DEBUG
	if ( n == 4 ) {
		printf("RMAP READ2: 0x%08x(4): 0x%08x\n", src, *(unsigned int *)dest);
	} else {
		printf("RMAP READ2: 0x%08x - 0x%08x\n", src, ((unsigned int)src)+(n-1));
	}
#endif

	/* Return sucessful */
	return 0;
}

/* Note that ((unsigned char *)src)[n] will be overwitten with the RMAP DATA CRC */
int spw_bus_write_mem(struct rtems_drvmgr_dev_info *dev, void *dest, const void *src, int n)
{
	struct rmap_command_write writecmd;
	int status;
	struct spw_bus_dev_info *info = (struct spw_bus_dev_info *)dev->businfo;
	struct spw_bus_priv *priv = (struct spw_bus_priv *)dev->parent->priv;

	/* Use Verify Write when accessing registers (length 1,2,4). */
	if ( n <= 4 ) {
		writecmd.type = RMAP_CMD_WIV;
	} else {
		writecmd.type = RMAP_CMD_WI;
	}
	writecmd.dstadr = info->dstadr;
	writecmd.dstkey = info->dstkey;
	writecmd.address = (unsigned int)dest;
	writecmd.length = n;
	writecmd.data = (unsigned char *)src;
#ifdef DEBUG
	if ( n == 4 ) {
		printf("RMAP WRITE: 0x%08x(4): 0x%08x\n",
			dest, *(unsigned int *)src);
	} else {
		printf("RMAP WRITE: 0x%08x - 0x%08x\n",
			dest, ((unsigned int)dest)+(n-1));
	}
#endif
	/* Send Command */
	status = rmap_send(priv->config->rmap,
				(struct rmap_command *)&writecmd);

	if ( status ) {
		printf("RMAP_MEMCPY WRITE: Failed to send/receive command %d\n",
			status);
		return -1;
	}

	/* Read Data */
	if ( writecmd.status != 0 ) {
		printf("RMAP_MEMCPY WRITE: Status non-zero 0x%x, adr: 0x%x\n",
			(unsigned char)writecmd.status,
			(unsigned char)writecmd.address);
		return -1;
	}

	/* Return sucessful */
	return 0;
}

int spw_bus_freq_get(
	struct rtems_drvmgr_dev_info *dev,
	int options,
	unsigned int *freq_hz)
{
	/* Link Frequency does not translate so good here, since it is
	 * a SpaceWire network, different parts may have different
	 * transfer rates.
	 */
	*freq_hz = 0;

	return -1;
}

int spw_bus_read_io8(struct rtems_drvmgr_dev_info *dev, uint8_t *srcadr, uint8_t *result)
{
	if ( spw_bus_memcpy(dev, result, srcadr, 1) ) {
		return -1;
	}
	return 0;
}

int spw_bus_read_io16(struct rtems_drvmgr_dev_info *dev, uint16_t *srcadr, uint16_t *result)
{
	if ( spw_bus_memcpy(dev, result, srcadr, 2) ) {
		return -1;
	}
	return 0;
}

int spw_bus_read_io32(struct rtems_drvmgr_dev_info *dev, uint32_t *srcadr, uint32_t *result)
{
	if ( spw_bus_memcpy(dev, result, srcadr, 4) ) {
		return -1;
	}
	return 0;
}

int spw_bus_read_io64(struct rtems_drvmgr_dev_info *dev, uint64_t *srcadr, uint64_t *result)
{
	if ( spw_bus_memcpy(dev, result, srcadr, 8) ) {
		return -1;
	}
	return 0;
}

int spw_bus_write_io8(struct rtems_drvmgr_dev_info *dev, uint8_t *dstadr, uint8_t data)
{
	uint8_t buf[2]; /* One byte extra room for RMAP DATA CRC */

	buf[0] = data;
	if ( spw_bus_write_mem(dev, dstadr, &buf[0], 1) ) {
		return -1;
	}
	return 0;
}

int spw_bus_write_io16(struct rtems_drvmgr_dev_info *dev, uint16_t *dstadr, uint16_t data)
{
	uint16_t buf[2]; /* One byte extra room for RMAP DATA CRC */

	buf[0] = data;
	if ( spw_bus_write_mem(dev, dstadr, &buf[0], 2) ) {
		return -1;
	}
	return 0;
}

int spw_bus_write_io32(struct rtems_drvmgr_dev_info *dev, uint32_t *dstadr, uint32_t data)
{
	uint32_t buf[2]; /* One byte extra room for RMAP DATA CRC */

	buf[0] = data;
	if ( spw_bus_write_mem(dev, dstadr, &buf[0], 4) ) {
		return -1;
	}
	return 0;
}

int spw_bus_write_io64(struct rtems_drvmgr_dev_info *dev, uint64_t *dstadr, uint64_t data)
{
	uint64_t buf[2]; /* One byte extra room for RMAP DATA CRC */

	buf[0] = data;
	if ( spw_bus_write_mem(dev, dstadr, &buf[0], 8) ) {
		return -1;
	}
	return 0;
}

int spw_bus_get_params(struct rtems_drvmgr_dev_info *dev, struct rtems_drvmgr_bus_params *params)
{
	struct spw_bus_priv *priv = dev->parent->priv;

	params->dev_prefix = &priv->prefix[5];

	return 0;
}

/************* USER INTERFACE *************/

/*** START: THIS SHOULD BE MOVED TO GRSPW DRIVER ***/
struct grspw_priv {
   /* configuration parameters */ 
   struct rtems_drvmgr_dev_info *dev; /* Driver manager device */
   char devName[32]; /* Device Name */
   void *regs;
};
extern struct rtems_drvmgr_drv_info grspw_drv_info;

/* Find GRSPW device from device name */
static struct rtems_drvmgr_dev_info *grspw_find_dev(char *devName)
{
	struct rtems_drvmgr_drv_info *drv = &grspw_drv_info;
	struct rtems_drvmgr_dev_info *dev;
	struct grspw_priv *grspw_priv;

	dev = drv->dev;
	while(dev) {
		grspw_priv = dev->priv;
		if ( strcmp(devName, grspw_priv->devName) == 0 )
			return dev;
		dev = dev->next_in_drv;
	}
	return NULL;
}
/*** STOP: THIS SHOULD BE MOVED TO GRSPW DRIVER ***/


/* Called from USER to attach bus */
int spw_bus_register(struct spw_bus_config *config)
{
	struct rtems_drvmgr_dev_info *grspw_dev;
	struct rtems_drvmgr_bus_info *bus;
	struct spw_bus_priv *priv;

	DBG("SpW-BUS: finding GRSPW device\n");

	/* Find GRSPW Driver to attach bus to */
	grspw_dev = grspw_find_dev(config->devName);
	if ( !grspw_dev ) {
		DBG("SpW-BUS: Failed to find GRSPW device\n");
		return -1;
	}
	if ( grspw_dev->bus ) {
		DBG("SpW-BUS: GRSPW already has a bus attached to it, aborting\n");
		return -1;
	}

	/* Allocate Bus and private structures */
	rtems_drvmgr_alloc_bus(&bus, sizeof(*priv));
	priv = (struct spw_bus_priv *)(bus + 1);

	/* Save the configuration for later */
	priv->config = config;

	/* Init the bus */
	grspw_dev->bus = bus;
	bus->bus_type = DRVMGR_BUS_TYPE_SPW_RMAP;
	bus->next = NULL;
	bus->dev = grspw_dev;
	bus->priv = priv;
	bus->children = NULL;
	bus->ops = (struct rtems_drvmgr_bus_ops *)&spw_bus_ops;
	bus->dev_cnt = 0;
	bus->reslist = NULL;
	bus->mmaps = NULL;
	priv->bus = bus;

	DBG("SpW-BUS: registering bus\n");
	rtems_drvmgr_bus_register(bus); /* this will call spw_bus_init to register the devices */

	return 0;
}
