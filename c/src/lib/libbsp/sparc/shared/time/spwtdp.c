/*  SPWTDP - SpaceWire Time Distribution Protocol. The driver provides
 *  device discovery and interrupt management.
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
#include <drvmgr/ambapp_bus.h>
#include <stdlib.h>
 
#include <spwtdp.h>

#define DRIVER_AMBAPP_GAISLER_SPWTDP_ID		DRIVER_AMBAPP_ID(VENDOR_GAISLER, GAISLER_SPWTDP)

/* Private structure of SPWTDP driver. */
struct spwtdp_priv {
	struct drvmgr_dev *dev;
	struct spwtdp_regs *regs;
	int open;
	
	spwtdp_isr_t user_isr;
	void *user_isr_arg;

	struct spwtdp_stats stats;
};

void spwtdp_isr(void *data);

struct amba_drv_info spwtdp_drv_info;

/* Hardware Reset of SPWTDP */
int spwtdp_hw_reset(struct spwtdp_priv *priv)
{
	struct spwtdp_regs *r = priv->regs;
	int i = 1000;

	r->conf[0] = 1;

	while ((r->conf[0] & 1) && i > 0) {
		i--;
	}

	spwtdp_clear_irqs(priv, -1);

	return i ? 0 : -1;
}

int spwtdp_reset(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	return spwtdp_hw_reset(priv);
}

void *spwtdp_open(int minor)
{
	struct spwtdp_priv *priv;
	struct drvmgr_dev *dev;

	/* Get Device from Minor */
	if ( drvmgr_get_dev(&spwtdp_drv_info.general, minor, &dev) ) {
		return NULL;
	}

	priv = dev->priv;
	if ( (priv == NULL) || priv->open )
		return NULL;

	/* Set initial state of software */
	priv->open = 1;

	/* Clear Statistics */
	spwtdp_clr_stats(priv);
	priv->user_isr = NULL;
	priv->user_isr_arg = NULL;

	return priv;
}

void spwtdp_close(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	if ( priv->open == 0 )
		return;

	/* Reset Hardware */
	spwtdp_hw_reset(priv);

	priv->open = 0;	
}

void spwtdp_int_enable(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	/* Register and Enable Interrupt at Interrupt controller */
	drvmgr_interrupt_register(priv->dev, 0, "spwtdp", spwtdp_isr, priv);
}

void spwtdp_int_disable(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	/* Enable Interrupt at Interrupt controller */
	drvmgr_interrupt_unregister(priv->dev, 0, spwtdp_isr, priv);
}

void spwtdp_clr_stats(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	memset(&priv->stats, 0, sizeof(priv->stats));
}

void spwtdp_get_stats(void *spwtdp, struct spwtdp_stats *stats)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	memcpy(stats, &priv->stats, sizeof(priv->stats));
}

/* Clear interrupts */
void spwtdp_clear_irqs(void *spwtdp, int irqs)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	priv->regs->ists = irqs;
}

/* Enable interrupts */
void spwtdp_enable_irqs(void *spwtdp, int irqs)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	priv->regs->ien  = irqs;
}

struct spwtdp_regs *spwtdp_get_regs(void *spwtdp)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	return priv->regs;
}

void spwtdp_int_register(void *spwtdp, spwtdp_isr_t func, void *data)
{
	struct spwtdp_priv *priv = (struct spwtdp_priv *)spwtdp;

	priv->user_isr = func;
	priv->user_isr_arg = data;
}

void spwtdp_isr(void *data)
{
	struct spwtdp_priv *priv = data;
	struct spwtdp_stats *stats = &priv->stats;
	unsigned int ists = priv->regs->ists;

	/* Return if the SPWTDP didn't generate the IRQ */
	if (ists == 0)
		return;

	stats->nirqs++;

	if (ists & SPWTDP_IRQ_DIT)
		stats->irq_tx++;
	if (ists & SPWTDP_IRQ_DIR)
		stats->irq_rx++;
	if (ists & SPWTDP_IRQ_TT)
		stats->tc_tx++;
	if (ists & SPWTDP_IRQ_TM)
		stats->time_ccsds_tx++;
	if (ists & SPWTDP_IRQ_TR)
		stats->tc_rx++;
	if (ists & SPWTDP_IRQ_S)
		stats->sync++;

	/* Let user Handle Interrupt */
	if (priv->user_isr)
		priv->user_isr(ists, priv->user_isr_arg);
}

/*** INTERFACE TO DRIVER MANAGER ***/

int spwtdp_init2(struct drvmgr_dev *dev)
{
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;
	struct spwtdp_priv *priv;
	struct spwtdp_regs *regs;

	priv = dev->priv;
	if (priv == NULL)
		return DRVMGR_NOMEM;
	memset(priv, 0, sizeof(*priv));
	priv->dev = dev;

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)dev->businfo;
	if ( ambadev == NULL ) {
		return DRVMGR_ENORES;
	}
	pnpinfo = &ambadev->info;
	regs = (struct spwtdp_regs *)pnpinfo->apb_slv->start;

	priv->regs = regs;

	spwtdp_hw_reset(priv);

	return DRVMGR_OK;
}

struct drvmgr_drv_ops spwtdp_ops =
{
	{NULL, spwtdp_init2, NULL, NULL},
	NULL,
	NULL
};

struct amba_dev_id spwtdp_ids[] =
{
	{VENDOR_GAISLER, GAISLER_SPWTDP},
	{0, 0}	/* Mark end of table */
};

struct amba_drv_info spwtdp_drv_info =
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_AMBAPP_GAISLER_SPWTDP_ID,/* Driver ID */
		"SPWTDP_DRV",			/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,		/* Bus Type */
		&spwtdp_ops,
		NULL,				/* Funcs */
		0,				/* No devices yet */
		sizeof(struct spwtdp_priv),	/* Let DrvMgr allocate priv */
	},
	&spwtdp_ids[0]
};

/* Register the SPWTDP Driver */
void spwtdp_register(void)
{
	drvmgr_drv_register(&spwtdp_drv_info.general);
}
