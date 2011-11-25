#include <rtems.h>
#include <bsp.h>
#include <bsp/irq-generic.h>
#include <leon.h>

static inline void leon_dispatch_irq(int irq)
{
	bsp_interrupt_handler_entry *e =
		&bsp_interrupt_handler_table[bsp_interrupt_handler_index(irq)];

	while (e != NULL) {
		(*e->handler)(e->arg);
		e = e->next;
	}
}

/* Called directly from IRQ trap handler TRAP[0x10..0x1F] = IRQ[0..15] */
void LEON_ISR_handler(rtems_vector_number vector)
{
	int irq = LEON_TRAP_SOURCE(vector);

	/* Let BSP fixup and/or handle incomming IRQ */
	irq = leon_irq_fixup(irq);

	leon_dispatch_irq(irq);
}

/* Initialize interrupts */
int BSP_shared_interrupt_init(void)
{
	rtems_vector_number vector;
	rtems_isr_entry previous_isr;
	int sc, i;

	for (i=0; i <= BSP_INTERRUPT_VECTOR_MAX_STD; i++) {
		vector = LEON_TRAP_TYPE(i);
		rtems_interrupt_catch(LEON_ISR_handler, vector, &previous_isr);
	}

	/* Initalize interrupt support */
	sc = bsp_interrupt_initialize();
	if (sc != RTEMS_SUCCESSFUL)
		return -1;

	return 0;
}

/* Callback from bsp_interrupt_initialize() */
rtems_status_code bsp_interrupt_facility_initialize(void)
{
	return RTEMS_SUCCESSFUL;
}

/* Spurious IRQ handler */
void bsp_interrupt_handler_default(rtems_vector_number vector)
{
	printk("Spurious IRQ %d\n", (int)vector);
}

rtems_status_code bsp_interrupt_vector_enable(rtems_vector_number vector)
{
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);
	LEON_Unmask_interrupt((int)vector);
	rtems_interrupt_enable(level);

	return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_vector_disable(rtems_vector_number vector)
{
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);
	LEON_Mask_interrupt((int)vector);
	rtems_interrupt_enable(level);

	return RTEMS_SUCCESSFUL;
}

void BSP_shared_interrupt_mask(int irq)
{
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);

	LEON_Mask_interrupt(irq);

	rtems_interrupt_enable(level);
}

void BSP_shared_interrupt_unmask(int irq)
{
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);

	LEON_Unmask_interrupt(irq);

	rtems_interrupt_enable(level);
}

void BSP_shared_interrupt_clear(int irq)
{
	/* We don't have to interrupt lock here, because the register is only
	 * written and self clearing
	 */
	LEON_Clear_interrupt(irq);
}
