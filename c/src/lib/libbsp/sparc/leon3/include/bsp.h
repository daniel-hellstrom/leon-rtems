/*  bsp.h
 *
 *  This include file contains all SPARC simulator definitions.
 *
 *  COPYRIGHT (c) 1989-1998.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  Ported to ERC32 implementation of the SPARC by On-Line Applications
 *  Research Corporation (OAR) under contract to the European Space
 *  Agency (ESA).
 *
 *  ERC32 modifications of respective RTEMS file: COPYRIGHT (c) 1995.
 *  European Space Agency.
 *
 *  $Id$
 */

#ifndef _BSP_H
#define _BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bspopts.h>

#include <rtems.h>
#include <leon.h>
#include <rtems/clockdrv.h>
#include <rtems/console.h>

/* SPARC CPU variant: LEON3 */
#define LEON3 1

/*
 *  BSP provides its own Idle thread body
 */
void *bsp_idle_thread( uintptr_t ignored );
#define BSP_IDLE_TASK_BODY bsp_idle_thread

#define CONFIGURE_NUMBER_OF_TERMIOS_PORTS 6

/*
 * Network driver configuration
 */
struct rtems_bsdnet_ifconfig;
extern int rtems_leon_open_eth_driver_attach(
  struct rtems_bsdnet_ifconfig *config,
  int attach
);
extern int rtems_smc91111_driver_attach_leon3(
  struct rtems_bsdnet_ifconfig *config,
  int attach
);
extern int rtems_leon_greth_driver_attach(
  struct rtems_bsdnet_ifconfig *config,
  int attach
);

#define RTEMS_BSP_NETWORK_DRIVER_NAME_OPENETH	"open_eth1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH_OPENETH	 rtems_leon_open_eth_driver_attach
#define RTEMS_BSP_NETWORK_DRIVER_NAME_SMC91111	"smc_eth1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH_SMC91111 rtems_smc91111_driver_attach_leon3
#define RTEMS_BSP_NETWORK_DRIVER_NAME_GRETH	 "gr_eth1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH_GRETH    rtems_leon_greth_driver_attach

#ifndef RTEMS_BSP_NETWORK_DRIVER_NAME
#define RTEMS_BSP_NETWORK_DRIVER_NAME   RTEMS_BSP_NETWORK_DRIVER_NAME_GRETH
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH RTEMS_BSP_NETWORK_DRIVER_ATTACH_GRETH
#endif

extern int   CPU_SPARC_HAS_SNOOPING;


/* Constants */

/*
 *  Information placed in the linkcmds file.
 */

extern int   RAM_START;
extern int   RAM_END;
extern int   RAM_SIZE;

extern int   PROM_START;
extern int   PROM_END;
extern int   PROM_SIZE;

extern int   CLOCK_SPEED;

extern int   end;        /* last address in the program */

/* miscellaneous stuff assumed to exist */

rtems_isr_entry set_vector(                     /* returns old vector */
    rtems_isr_entry     handler,                /* isr routine        */
    rtems_vector_number vector,                 /* vector number      */
    int                 type                    /* RTEMS or RAW intr  */
);

void BSP_fatal_return( void );

void bsp_spurious_initialize( void );

/*** Shared system interrupt handling ***/

/* Interrupt Service Routine (ISR) pointer */
typedef void (*bsp_shared_isr)(int irq, void *arg);

/* Initializes the Shared System Interrupt service */
extern int BSP_shared_interrupt_init(void);

/* Registers a shared IRQ handler, but does not enable it. Multiple
 * interrupt handlers may use the same IRQ number, all enabled ISRs
 * will be called when an interrupt on that line is fired.
 *
 * Arguments
 *  irq       System IRQ number
 *  isr       Function pointer to the ISR
 *  arg       Second argument to function isr
 */
extern int BSP_shared_interrupt_register
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	);

/* Unregister previously registered shared IRQ handler.
 *
 * Arguments
 *  irq       System IRQ number
 *  isr       Function pointer to the ISR
 *  arg       Second argument to function isr
 */
extern int BSP_shared_interrupt_unregister
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	);

/* Enable shared IRQ handler. This function will unmask the interrupt
 * controller and mark this interrupt handler ready to handle interrupts. Note
 * that since it is a shared interrupt handler service the interrupt may
 * already be enabled, however no calls to this specific handler is made
 * until it is enabled.
 *
 * Arguments
 *  irq       System IRQ number
 *  isr       Function pointer to the ISR
 *  arg       Second argument to function isr
 */

extern int BSP_shared_interrupt_enable
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	);

/* Disable shared IRQ handler. This function will mask the interrupt
 * controller and mark this interrupt handler not ready to receive interrupts.
 * Note that since it is a shared interrupt handler service the interrupt may
 * still be enabled, however no calls to this specific handler is made
 * while it is disabled.
 *
 * Arguments
 *  irq         System IRQ number
 *  isr         Function pointer to the ISR
 *  arg         Second argument to function isr
 */
extern int BSP_shared_interrupt_disable
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	);

/* Clear interrupt pending on IRQ controller, this is typically done on a 
 * level triggered interrupt source such as PCI to avoid taking double IRQs.
 * In such a case the interrupt source must be cleared first, before 
 * acknowledging the IRQ with this function.
 *
 * Arguments
 *  irq       System IRQ number
 *  isr       Function pointer to the ISR
 *  arg       Second argument to function isr
 */
extern int BSP_shared_interrupt_clear
	(
	int irq,
	bsp_shared_isr isr,
	void *arg
	);

extern void BSP_shared_interrupt_mask(int irq);

extern void BSP_shared_interrupt_unmask(int irq);

/* Reload watchdog (last timer on the first GPTIMER core), all systems does not
 * feature a watchdog, it is expected that if this function is called the
 * user knows that there is a watchdog available.
 *
 * The prescaler is normally set to number of MHz of system, this is to
 * make the system clock tick be stable.
 *
 * Arguments
 *  watchdog       - Always 0 for now
 *  reload_value   - Number of timer clocks (after prescaler) to count before 
 *                   watchdog is woken.
 */
extern void bsp_watchdog_reload(int watchdog, unsigned int reload_value);

/* Stop watchdog timer */
extern void bsp_watchdog_stop(int watchdog);

#ifdef __cplusplus
}
#endif

#endif


