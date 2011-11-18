#include <bsp.h>

/* If RTEMS_DRVMGR_STARTUP is defined extra code is added that
 * registers the LEON2 AMBA bus driver as root driver into the
 * driver manager.
 *
 * The structues here are declared weak so that the user can override
 * the configuration and add custom cores in the RTEMS project.
 */
#ifdef RTEMS_DRVMGR_STARTUP
#include <drvmgr/leon2_amba_bus.h>

/* All drivers included by BSP, this is overridden by the user by including
 * the devmgr_confdefs.h. No specifc drivers needed by BSP since IRQ/TIMER/UART
 * is not drvmgr drivers.
 */
struct rtems_drvmgr_drv_reg_func rtems_drvmgr_drivers[] __attribute__((weak)) =
{
          {NULL} /* End array with NULL */
};

/* Defines what cores are avilable on the bus in addition to the standard
 * LEON2 peripherals.
 */
struct leon2_core leon2_amba_custom_cores[] __attribute__((weak)) =
{
        EMPTY_LEON2_CORE
};

/* Configure LEON2 Root bus driver */
struct leon2_bus leon2_bus_config __attribute__((weak)) =
{
        &drv_mgr_leon2_std_cores[0],   /* The standard cores, defined by driver */
        &leon2_amba_custom_cores[0],   /* custom cores, defined by us */
        NULL
};

/* Driver resources on LEON2 AMBA bus. Used to set options for particular
 * LEON2 cores, it is up to the driver to look at the configuration paramters
 * once started.
 */
struct rtems_drvmgr_drv_res leon2_amba_res[] __attribute__((weak)) =
{
        RES_EMPTY
};
#endif

/*
 *  bsp_predriver_hook
 *
 *  BSP predriver hook.  Called just before drivers are initialized.
 *  Is used to initialize shared interrupt handling.
 */
void bsp_predriver_hook(void)
{
  /* Initialize shared interrupt handling */
  BSP_shared_interrupt_init();

#ifdef RTEMS_DRVMGR_STARTUP
	drv_mgr_leon2_init(&leon2_bus_config, &leon2_amba_res[0]);
#endif
}
