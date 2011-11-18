#include <rtems.h>
#include <rtems/rtems/intr.h>
#include <bsp.h>
#include <genirq.h>

extern rtems_isr bsp_spurious_handler(rtems_vector_number trap, CPU_Interrupt_frame *isf);

genirq_t BSP_shared_interrupt_d;

rtems_isr BSP_shared_interrupt_isr(rtems_vector_number v)
{
	genirq_doirq(BSP_shared_interrupt_d, v - 0x10); /* Vector number to IRQ number */
}

int BSP_shared_interrupt_init(void)
{
	int irq_cnt;
#ifdef LEON3
	/* Initialize the Shared Interrupt service */
	if ( LEON3_IrqCtrl_EIrq != 0 ) {
		irq_cnt = 32;
	} else {
		irq_cnt = 16;
	}
#elif defined(LEON2)
	irq_cnt = 16;
#else
#error CPU NOT SUPPORTED
#endif

	BSP_shared_interrupt_d = genirq_init(irq_cnt);
	if ( BSP_shared_interrupt_d == NULL )
		return -1;
	return 0;
}

/* internal function */
int BSP_shared_interrupt_connect(int irq, rtems_isr_entry newisr)
{
	rtems_vector_number vector;
	rtems_isr_entry previous_isr;

	vector = LEON_TRAP_TYPE(irq);

#ifdef LEON3
	/* If we are using extended IRQ controller we let that interrupt service
	 * take care of interrupt registration, else we use the standard facility
	 */
	if ( irq > 15 ) {
		bsp_leon3_ext_isr_connect(vector, newisr, &previous_isr);
	} else {
		rtems_interrupt_catch(newisr, vector, &previous_isr);
	}
#elif defined(LEON2)
	rtems_interrupt_catch(newisr, vector, &previous_isr);
#else
#error CPU NOT SUPPORTED
#endif

	return 0;
}

int BSP_shared_interrupt_register
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	)
{
	int ret;
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);

	ret = genirq_register(BSP_shared_interrupt_d, irq, isr, arg);
	if ( ret == 0 ) {
		/* First time to register IRQ for this interrupt means that
		 * we should install an interrupt handler 
		 */
		ret = BSP_shared_interrupt_connect(irq, BSP_shared_interrupt_isr);
	} else if ( ret == 1 ) {
		ret = 0;
	}

	rtems_interrupt_enable(level);

	return ret;
}

int BSP_shared_interrupt_unregister
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	)
{
	int ret;
	rtems_interrupt_level level;

	BSP_shared_interrupt_disable(irq, isr, arg);

	rtems_interrupt_disable(level);

	ret = genirq_unregister(BSP_shared_interrupt_d, irq, isr, arg);
	if ( ret == 0 ) {
		/* Successfull unregister, and no other handler left on this IRQ. 
		 * Unregister by registering the spurious interrupt handler to
		 * handle unwanted and unexpected IRQs.
		 */
		ret = BSP_shared_interrupt_connect(irq, bsp_spurious_handler);
	} else if ( ret == 1 ) {
		ret = 0;
	}

	rtems_interrupt_enable(level);

	return ret;
}

int BSP_shared_interrupt_enable
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	)
{
	int ret;
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);

	ret = genirq_enable(BSP_shared_interrupt_d, irq, isr, arg);
	if ( ret == 0 ) {
		LEON_Unmask_interrupt(irq);
	} else if ( ret == 1 ) {
		ret = 0;
	}

	rtems_interrupt_enable(level);

	return ret;
}

int BSP_shared_interrupt_disable
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	)
{
	int ret;
	rtems_interrupt_level level;

	rtems_interrupt_disable(level);

	ret = genirq_disable(BSP_shared_interrupt_d, irq, isr, arg);
	if ( ret == 0 ) {
		LEON_Mask_interrupt(irq);
	} else if ( ret == 1 ) {
		ret = 0;
	}

	rtems_interrupt_enable(level);

	return ret;
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

int BSP_shared_interrupt_clear
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	)
{
	rtems_interrupt_level level;

	if ( genirq_check(BSP_shared_interrupt_d, irq) )
		return -1;

	/* We don't have to interrupt lock here, because the register is only written
	 * and self clearing */
	LEON_Clear_interrupt(irq);

	return 0;
}
