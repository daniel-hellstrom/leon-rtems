#include <rtems.h>
#include <stdlib.h>
#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include "tlib.h"

#ifdef RTEMS_DRVMGR_STARTUP
#include <leon.h>
volatile LEON3_Timer_Regs_Map *LEON3_Timer_Regs = 0;
#endif

struct gptimer_timer_regs {
  volatile unsigned int value;
  volatile unsigned int reload;
  volatile unsigned int ctrl;
  volatile unsigned int notused;
};

struct gptimer_regs {
  volatile unsigned int scaler_value;   /* common timer registers */
  volatile unsigned int scaler_reload;
  volatile unsigned int cfg;
  volatile unsigned int notused;
  struct gptimer_timer_regs timer[7];
};

/* GPTIMER Core Configuration Register (READ-ONLY) */
#define GPTIMER_CFG_TIMERS_BIT	0
#define GPTIMER_CFG_IRQ_BIT	3
#define GPTIMER_CFG_SI_BIT	8
#define GPTIMER_CFG_DF_BIT	9

#define GPTIMER_CFG_TIMERS	(0x7<<GPTIMER_CFG_TIMERS_BIT)
#define GPTIMER_CFG_IRQ		(0x1f<<GPTIMER_CFG_IRQ_BIT)
#define GPTIMER_CFG_SI		(1<<GPTIMER_CFG_SI_BIT)
#define GPTIMER_CFG_DF		(1<<GPTIMER_CFG_DF_BIT)

/* GPTIMER Timer Control Register */
#define GPTIMER_CTRL_EN_BIT	0
#define GPTIMER_CTRL_RS_BIT	1
#define GPTIMER_CTRL_LD_BIT	2
#define GPTIMER_CTRL_IE_BIT	3
#define GPTIMER_CTRL_IP_BIT	4
#define GPTIMER_CTRL_CH_BIT	5
#define GPTIMER_CTRL_DH_BIT	6

#define GPTIMER_CTRL_EN	(1<<GPTIMER_CTRL_EN_BIT)
#define GPTIMER_CTRL_RS	(1<<GPTIMER_CTRL_RS_BIT)
#define GPTIMER_CTRL_LD	(1<<GPTIMER_CTRL_LD_BIT)
#define GPTIMER_CTRL_IE	(1<<GPTIMER_CTRL_IE_BIT)
#define GPTIMER_CTRL_IP	(1<<GPTIMER_CTRL_IP_BIT)
#define GPTIMER_CTRL_CH	(1<<GPTIMER_CTRL_CH_BIT)
#define GPTIMER_CTRL_DH	(1<<GPTIMER_CTRL_DH_BIT)

#define DBG(x...)

/* GPTIMER timer private */
struct gptimer_timer {
	struct tlib_dev tdev;	/* Must be first in struct */
	struct gptimer_timer_regs *tregs;
	char index; /* Timer Index in this driver */
	char tindex; /* Timer Index In Hardware */
};

/* GPTIMER Core private */
struct gptimer_priv {
	struct rtems_drvmgr_dev_info *dev;
	struct gptimer_regs *regs;
	unsigned int base_clk;
	unsigned int base_freq;
	int separate_interrupt;

	/* Structure per Timer unit, the core supports up to 8 timers */
	int timer_cnt;
	struct gptimer_timer timers[0];
};

void gptimer_isr(int irqno, void *data);

#if 0
void gptimer_tlib_irq_register(struct tlib_drv *tdrv, tlib_isr_t func, void *data)
{
	struct gptimer_priv *priv = (struct gptimer_priv *)tdrv;

	if ( SHARED ...)
	
	
	drvmgr_interrupt_register();
}
#endif

/******************* Driver manager interface ***********************/

/* Driver prototypes */
struct tlib_drv gptimer_tlib_drv;
int gptimer_device_init(struct gptimer_priv *priv);

int gptimer_init1(struct rtems_drvmgr_dev_info *dev);

struct rtems_drvmgr_drv_ops gptimer_ops =
{
	.init = {gptimer_init1, NULL, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

struct amba_dev_id gptimer_ids[] =
{
	{VENDOR_GAISLER, GAISLER_GPTIMER},
	{0, 0}		/* Mark end of table */
};

struct amba_drv_info gptimer_drv_info =
{
	{
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_GPTIMER_ID,/* Driver ID */
		"GPTIMER_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&gptimer_ops,
		0,				/* No devices yet */
	},
	&gptimer_ids[0]
};

void gptimer_register_drv (void)
{
	DBG("Registering GPTIMER driver\n");
	rtems_drvmgr_drv_register(&gptimer_drv_info.general);
}

int gptimer_init1(struct rtems_drvmgr_dev_info *dev)
{
	struct gptimer_priv *priv;
	struct gptimer_regs *regs;
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;
	int timer_hw_cnt, timer_cnt, timer_start;
	int i, size;
	struct gptimer_timer *timer;
	union rtems_drvmgr_key_value *value;
#ifdef RTEMS_DRVMGR_STARTUP
	char timer_index[7];
#endif

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)dev->businfo;
	if ( ambadev == NULL ) {
		return -1;
	}
	pnpinfo = &ambadev->info;
	regs = (struct gptimer_regs *)pnpinfo->apb_slv->start;

	DBG("GPTIMER[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);

	/* Get number of Timers */
	timer_hw_cnt = regs->cfg & GPTIMER_CFG_TIMERS;

	/* Let user spelect a range of timers to be used. In AMP systems
	 * it is sometimes neccessary to leave timers for other CPU instances.
	 */
	timer_cnt = timer_hw_cnt;
	timer_start = 0;
	value = rtems_drvmgr_dev_key_get(dev, "timerStart", KEY_TYPE_INT);
	if ( value) {
		timer_start = value->i;
		timer_cnt = timer_hw_cnt - timer_start;
	}
	value = rtems_drvmgr_dev_key_get(dev, "timerCnt", KEY_TYPE_INT);
	if ( value && (value->i < timer_cnt) ) {
		timer_cnt = value->i;
	}

	/* Allocate Common Timer Description, size depends on how many timers
	 * are present.
	 */
	size = sizeof(struct gptimer_priv) +
		timer_cnt*sizeof(struct gptimer_timer);
	priv = dev->priv = (struct gptimer_priv *)malloc(size);
	if ( !priv )
		return DRVMGR_NOMEM;
	memset(priv, 0, size);
	priv->dev = dev;
	priv->regs = regs;

#ifdef RTEMS_DRVMGR_STARTUP
	if ( rtems_drvmgr_on_rootbus(priv->dev) ) {
		/* Bootloader has initialized the Timer prescaler to 1MHz,
		 * this means that the AMBA Frequency is 1MHz * PRESCALER.
		 */
		priv->base_clk = (regs->scaler_reload + 1) * 1000000;
		ambapp_bus_freq_register(priv->dev,DEV_APB_SLV,priv->base_clk);
		if ( !LEON3_Timer_Regs )
			LEON3_Timer_Regs = (void *)regs;
	} else
#endif
	{
		/* The Base Frequency of the GPTIMER core is the same as the
		 * frequency of the AMBA bus it is situated on.
		 */
		rtems_drvmgr_freq_get(dev, DEV_APB_SLV, &priv->base_clk);
	}

	/* This core will may provide important Timer functionality
	 * to other drivers and the RTEMS kernel, the Clock driver
	 * may for example use this device. So the Timer driver must be
	 * initialized in the first iiitialization stage.
	 */

	/*** Initialize Hardware ***/

	/* If user request to set prescaler, we will do that. However, note
	 * that doing so for the Root-Bus GPTIMER may affect the RTEMS Clock
	 * so that Clock frequency is wrong.
	 */
	value = rtems_drvmgr_dev_key_get(priv->dev, "prescaler", KEY_TYPE_INT);
	if ( value )
		regs->scaler_reload = value->i;

	/* Get Frequency that the timers are operating in (after prescaler) */
	priv->base_freq = priv->base_clk / (priv->regs->scaler_reload + 1);

	priv->timer_cnt = timer_cnt;
	for (i=0; i<timer_cnt; i++) {
		timer = &priv->timers[i];
		timer->index = i;
		timer->tindex = i + timer_start;
		timer->tregs = &regs->timer[timer->tindex];
		timer->tdev.drv = &gptimer_tlib_drv;

		/* Stop Timer */
		timer->tregs->ctrl = 0;

		/* Register Timer at Timer Library */
#ifdef RTEMS_DRVMGR_STARTUP
		timer_index[i] =
#endif
			tlib_dev_reg(&timer->tdev);
	}

	/* Check Interrupt support implementation, two cases:
	 *  A. All Timers share one IRQ
	 *  B. Each Timer have an individual IRQ. The number is:
	 *        BASE_IRQ + timer_index
	 */
	priv->separate_interrupt = regs->cfg & GPTIMER_CFG_SI;

	if ( priv->separate_interrupt == 0 ) {
		/* Shared IRQ handler */
		rtems_drvmgr_interrupt_register(priv->dev, 0, gptimer_isr,priv);
		rtems_drvmgr_interrupt_enable(priv->dev, 0, gptimer_isr, priv);
	}

	/* If the user request a certain Timer to be the RTEMS Clock Timer,
	 * the timer must be registered at the Clock Driver.
	 */
#ifdef RTEMS_DRVMGR_STARTUP
	value = rtems_drvmgr_dev_key_get(priv->dev, "clockTimer", KEY_TYPE_INT);
	if ( value && (value->i < timer_cnt) ) {
		LEON3_Timer_Regs = (void *)regs;
		Clock_timer_register(timer_index[value->i]);
	}
#endif

	return DRVMGR_OK;
}

static inline struct gptimer_priv *priv_from_timer(struct gptimer_timer *t)
{
	return (struct gptimer_priv *)
		((unsigned int)t -
		sizeof(struct gptimer_priv) -
		t->index * sizeof(struct gptimer_timer));
}

void gptimer_isr(int irqno, void *data)
{
	struct gptimer_priv *priv = data;
	struct gptimer_timer_regs *tregs;
	int i;
	unsigned int ctrl;

	/* Check all timers for IRQ */
	for (i=0;i<priv->timer_cnt; i++) {
		tregs = priv->timers[i].tregs;
		ctrl = tregs->ctrl;
		if ( ctrl & GPTIMER_CTRL_IP ) {
			/* IRQ Was generated by Timer, Clear Pending flag
			 * call ISR registered
			 */
			tregs->ctrl = ctrl | GPTIMER_CTRL_IP;
			if ( priv->timers[i].tdev.isr_func ) {
				priv->timers[i].tdev.isr_func(
					irqno, priv->timers[i].tdev.isr_data);
			}
		}
	}
}

void gptimer_tlib_reset(struct tlib_dev *hand)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;

	timer->tregs->ctrl = 0;
	timer->tregs->reload = 0xffffffff;
	timer->tregs->ctrl = GPTIMER_CTRL_LD;
}

void gptimer_tlib_get_freq(
	struct tlib_dev *hand,
	unsigned int *basefreq,
	unsigned int *tickrate)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;
	struct gptimer_priv *priv = priv_from_timer(timer);

	/* Calculate base frequency from Timer Clock and Prescaler */
	if ( basefreq )
		*basefreq = priv->base_freq;
	if ( tickrate )
		*tickrate = timer->tregs->reload + 1;
}

int gptimer_tlib_set_freq(struct tlib_dev *hand, unsigned int tickrate)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;

	timer->tregs->reload = tickrate - 1;

	/*Check that value was allowed (Timer may not be as wide as expected)*/
	if ( timer->tregs->reload != (tickrate - 1) )
		return -1;
	else
		return 0;
}

void gptimer_tlib_irq_reg(struct tlib_dev *hand, tlib_isr_t func, void *data)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;
	struct gptimer_priv *priv = priv_from_timer(timer);

	if ( priv->separate_interrupt ) {
		rtems_drvmgr_interrupt_register(priv->dev, timer->tindex,
						func, data);
		rtems_drvmgr_interrupt_enable(priv->dev, timer->tindex,
						func, data);
	}

	timer->tregs->ctrl |= GPTIMER_CTRL_IE;
}

void gptimer_tlib_irq_unreg(struct tlib_dev *hand, tlib_isr_t func, void *data)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;
	struct gptimer_priv *priv = priv_from_timer(timer);

	/* Turn off IRQ at source, unregister IRQ handler */
	timer->tregs->ctrl &= ~GPTIMER_CTRL_IE;

	if ( priv->separate_interrupt ) {
		rtems_drvmgr_interrupt_disable(priv->dev, timer->tindex, 
						func, data);
		rtems_drvmgr_interrupt_unregister(priv->dev, timer->tindex,
						func, data);
	} else {
		timer->tdev.isr_func = NULL;
	}
}

void gptimer_tlib_start(struct tlib_dev *hand, int once)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;
	unsigned int ctrl;

	/* Load the selected frequency before starting Frequency */
	ctrl = GPTIMER_CTRL_LD | GPTIMER_CTRL_EN;
	if ( once == 0 )
		ctrl |= GPTIMER_CTRL_RS; /* Restart Timer */
	timer->tregs->ctrl |= ctrl;
}

void gptimer_tlib_stop(struct tlib_dev *hand)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;

	/* Load the selected Frequency */
	timer->tregs->ctrl &= ~(GPTIMER_CTRL_EN|GPTIMER_CTRL_IP);
}

void gptimer_tlib_restart(struct tlib_dev *hand)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;

	timer->tregs->ctrl |= GPTIMER_CTRL_LD | GPTIMER_CTRL_EN;
}

void gptimer_tlib_get_counter(struct tlib_dev *hand, int *counter)
{
	struct gptimer_timer *timer = (struct gptimer_timer *)hand;

	*counter = timer->tregs->value;
}

struct tlib_drv gptimer_tlib_drv =
{
	.reset = gptimer_tlib_reset,
	.get_freq = gptimer_tlib_get_freq,
	.set_freq = gptimer_tlib_set_freq,
	.irq_reg = gptimer_tlib_irq_reg,
	.irq_unreg = gptimer_tlib_irq_unreg,
	.start = gptimer_tlib_start,
	.stop = gptimer_tlib_stop,
	.restart = gptimer_tlib_restart,
	.get_counter = gptimer_tlib_get_counter,
	.custom = NULL,
};
