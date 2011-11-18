/*
 *  GRLIB/LEON3 extended interrupt controller
 *
 *  COPYRIGHT (c) 2008.
 *  Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#include <bsp.h>

extern int LEON3_Cpu_Index;

extern rtems_isr bsp_spurious_handler(rtems_vector_number trap, CPU_Interrupt_frame *isf);

/* GRLIB extended IRQ controller IRQ number */
int LEON3_IrqCtrl_EIrq = 0;

/* Extended Interrupt handlers */
ISR_Handler_entry LEON3_EIrq_handlers[16] = {0};

void bsp_leon3_ext_isr_connect(
  uint32_t    vector,
  proc_ptr    new_handler,
  proc_ptr   *old_handler
)
{
  int irq;

  if ( !LEON3_IrqCtrl_EIrq ) {
    /* No extended controller */
    return;
  }
  
  irq = vector - 0x20;
  
  LEON3_EIrq_handlers[irq] = new_handler;
}

rtems_isr LEON3_EISR_Handler(rtems_vector_number v)
{
  int eirq;
  ISR_Handler_entry handler;

  /* Get interrupt number from IRQ controller */
  eirq = LEON3_IrqCtrl_Regs->intid[LEON3_Cpu_Index] & 0x1f;
  
  handler = LEON3_EIrq_handlers[eirq-16];
  if ( handler ) {
    /* This is wrong, it will signal the wrong trap, however all interrupt handlers
     * subtract 0x10 to get the IRQ number since the first interrupt trap is at 0x10
     * so this makes the interrupt handlers less complicated.
     */
    handler(eirq + 0x10);
  }
}

void bsp_leon3_ext_irq_register(int irq)
{
  int i;

  /* Extended IRQ controller available */
  LEON3_IrqCtrl_EIrq = irq;

  /* Make all interrupts defualt to the spurious handler, it handles
   * unwanted/unexpected interrupts.
   */
  for (i=0; i<16; i++)
    LEON3_EIrq_handlers[i] = bsp_spurious_handler;

  /* Install IRQ handler for Extended interrupt controller */
  set_vector(LEON3_EISR_Handler, LEON3_IrqCtrl_EIrq+0x10, 1);
}
