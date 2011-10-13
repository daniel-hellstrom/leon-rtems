/*
 *  AMBA Plug & Play Bus Driver
 *
 *  This driver hook performs bus scanning.
 *
 *  COPYRIGHT (c) 2008.
 *  Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */
#include <bsp.h>
#include <ambapp.h>

/* AMBA Plug&Play information description.
 *
 * After software has scanned AMBA PnP it builds a tree to make
 * it easier for drivers to work with the bus architecture.
 */
struct ambapp_bus ambapp_plb;

/* If RTEMS_DRVMGR_STARTUP is defined extra code is added that
 * registers the GRLIB AMBA PnP bus driver as root driver.
 */
#ifdef RTEMS_DRVMGR_STARTUP
#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus_grlib.h>

extern void gptimer_register_drv (void);
extern void apbuart_cons_register_drv(void);
/* All drivers included by BSP, this is overridden by the user by including
 * the devmgr_confdefs.h. By default the Timer and UART driver is included.
 */
struct drvmgr_drv_reg_func drvmgr_drivers[] __attribute__((weak)) =
{
  {gptimer_register_drv},
  {apbuart_cons_register_drv},
  {NULL} /* End array with NULL */
};

/* Driver resources configuration for AMBA root bus. It is declared weak
 * so that the user may override it, if the defualt settings are not
 * enough.
 */
struct drvmgr_bus_res grlib_drv_resources __attribute__((weak)) =
{
  .next = NULL,
  .resource =
  {
    RES_EMPTY,
  }
};

/* GRLIB AMBA bus configuration (the LEON3 root bus configuration) */
struct grlib_config grlib_bus_config = 
{
  &ambapp_plb,              /* AMBAPP bus setup */
  &grlib_drv_resources,     /* Driver configuration */
};
#endif

/* GRLIB extended IRQ controller register */
extern void leon3_ext_irq_init(void);

/* Pointers to Interrupt Controller configuration registers */
volatile LEON3_IrqCtrl_Regs_Map *LEON3_IrqCtrl_Regs;

/* AMBA PP find routines */
int find_matching_adev(struct ambapp_dev *dev, int index, void *arg)
{
	*(struct ambapp_dev **)arg = dev;
	return 1; /* Satisfied with first matching device, stop search */
}

/*
 *  amba_initialize
 *
 *  Must be called just before drivers are initialized.
 *  Used to scan system bus. Probes for AHB masters, AHB slaves and
 *  APB slaves. Addresses to configuration areas of the AHB masters,
 *  AHB slaves, APB slaves and APB master are storeds in
 *  amba_ahb_masters, amba_ahb_slaves and amba.
 */

void amba_initialize(void)
{
  int i, icsel;
  struct ambapp_dev *adev;

  /* Scan AMBA Plug&Play information, assuming malloc() works.
   * The routine builds a PnP tree into ambapp_plb.root in RAM, so after this
   * we never access PnP information in Hardware any more.
   * Since on Processor Local Bus (PLB) memory mapping is 1:1
   */
  ambapp_scan(&ambapp_plb, LEON3_IO_AREA, NULL, NULL);

  /* Find LEON3 Interrupt controller */
  i = ambapp_for_each(&ambapp_plb, (OPTIONS_ALL|OPTIONS_APB_SLVS), 
              VENDOR_GAISLER, GAISLER_IRQMP, find_matching_adev, &adev);
  if ( i <= 0 ) {
    /* PANIC IRQ controller not found!
     *
     *  What else can we do but stop ...
     */
    asm volatile( "mov 1, %g1; ta 0x0" );
  }

  LEON3_IrqCtrl_Regs = (volatile LEON3_IrqCtrl_Regs_Map *)
                       ((struct ambapp_apb_info *)adev->devinfo)->start;
  if ( (LEON3_IrqCtrl_Regs->ampctrl >> 28) > 0 ) {
    /* IRQ Controller has support for multiple IRQ Controllers, each
     * CPU can be routed to different Controllers, we find out which
     * controller by looking at the IRQCTRL Select Register for this CPU.
     * Each Controller is located at a 4KByte offset.
     */
    icsel = LEON3_IrqCtrl_Regs->icsel[LEON3_Cpu_Index/8];
    icsel = (icsel >> ((7 - (LEON3_Cpu_Index & 0x7)) * 4)) & 0xf;
    LEON3_IrqCtrl_Regs += icsel;
    LEON3_IrqCtrl_Regs->mask[LEON3_Cpu_Index] = 0;
    LEON3_IrqCtrl_Regs->force[LEON3_Cpu_Index] = 0;
    LEON3_IrqCtrl_Regs->iclear = 0xffffffff;
  }

  /* Init Extended IRQ controller if available */
  leon3_ext_irq_init();

  /* Initialize shared interrupt handling, must be done after extended 
   * interrupt controller has been registered.
   */
  BSP_shared_interrupt_init();

  /* If we are running without Driver Manager at startup, we must still
   * assure that Timer and Console UART is working. So we can not
   * depend on the DrvMgr capable Timer and Console UART drivers,
   * instead we use the small-footprint drivers.
   */
#ifndef RTEMS_DRVMGR_STARTUP

  /* find GP Timer */
  i = ambapp_for_each(&ambapp_plb, (OPTIONS_ALL|OPTIONS_APB_SLVS), 
              VENDOR_GAISLER, GAISLER_GPTIMER, find_matching_adev, &adev);
  if ( i > 0 ){
    LEON3_Timer_Regs = (volatile LEON3_Timer_Regs_Map *)
                        ((struct ambapp_apb_info *)adev->devinfo)->start;

    /* Register AMBA Bus Frequency */
    ambapp_freq_init(&ambapp_plb, adev, 
                     (LEON3_Timer_Regs->scaler_reload + 1) * 1000000);
  }

#else

  /* Register Root bus, Use GRLIB AMBA PnP bus as root bus for LEON3 */
  ambapp_grlib_root_register(&grlib_bus_config);
#endif
}
