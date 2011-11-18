/*  set_vector
 *
 *  This routine installs an interrupt vector on the SPARC simulator.
 *
 *  INPUT PARAMETERS:
 *    handler - interrupt handler entry point
 *    vector  - vector number
 *    type    - 0 indicates raw hardware connect
 *              1 indicates RTEMS interrupt connect
 *              2 indicates RTEMS extended interrupt connect
 *
 *  OUTPUT PARAMETERS:  NONE
 *
 *  RETURNS:
 *    address of previous interrupt handler
 *
 *  COPYRIGHT (c) 1989-1998.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  Ported to LEON implementation of the SPARC by On-Line Applications
 *  Research Corporation (OAR) under contract to the European Space
 *  Agency (ESA).
 *
 *  LEON modifications of respective RTEMS file: COPYRIGHT (c) 1995.
 *  European Space Agency.
 *
 *  $Id$
 */

#include <bsp.h>

extern void bsp_leon3_ext_isr_connect(
  uint32_t    vector,
  proc_ptr    new_handler,
  proc_ptr   *old_handler
);

#define SET_VECTOR_RAW  0  /* Raw trap handler */
#define SET_VECTOR_INT  1  /* Trap handler with _ISR_Handler interrupt handler */
#define SET_VECTOR_EINT 2  /* Trap handler with extended interrupt handler, up to 31 interrupts */

rtems_isr_entry set_vector(                   /* returns old vector */
  rtems_isr_entry     handler,                /* isr routine        */
  rtems_vector_number vector,                 /* vector number      */
  int                 type                    /* RTEMS or RAW intr  */
)
{
  rtems_isr_entry previous_isr;
  uint32_t      real_trap;
  uint32_t      source;

  if ( (type == 2) && ((vector < 0x20) || (vector > 0x2F)) ) {
    /* Use standard interrupt connect facility for standard IRQs */
    type = 1;
  }

  if ( type == 1 )
    rtems_interrupt_catch( handler, vector, &previous_isr );
  else if ( type == 2 ) {
    /* Use Extended interrupt controller for traps 0x20-0x2f, no real
     * IRQ traps on 0x20-0x2f.
     */
    bsp_leon3_ext_isr_connect(vector, handler, (void *)&previous_isr);
  } else
    _CPU_ISR_install_raw_handler( vector, handler, (void *)&previous_isr );

  real_trap = SPARC_REAL_TRAP_NUMBER( vector );

  if ( LEON_INT_TRAP( real_trap ) || ((type==2) && LEON_EINT_TRAP( real_trap )) ) {

    source = LEON_TRAP_SOURCE( real_trap );

    LEON_Clear_interrupt( source );
    LEON_Unmask_interrupt( source );
  }

  return previous_isr;
}
