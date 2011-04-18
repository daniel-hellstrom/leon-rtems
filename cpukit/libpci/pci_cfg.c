/*  PCI Configuration Library
 *
 *  COPYRIGHT (c) 2010-2011.
 *  Aeroflex Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2011-02-11, Daniel Hellstrom <daniel@gaisler.com>
 *    created
 */

#include <pci/cfg.h>

/* Default to no configuration library, this is overrided in the project
 * configuration or by the BSP if it is not configurable.
 */
extern int (*pci_config_lib_init)(void);
extern void (*pci_config_lib_register)(void *config);

/* Number of buses. This is set from respective library */
int pci_bus_cnt = 0;

/* PCI Address assigned to BARs which failed to fit into the PCI Window or
 * is disabled by any other cause.
 */
uint32_t pci_invalid_address = 0;

/* Configure PCI devices and bridges, and setup the RAM data structures
 * describing the PCI devices currently present in the system
 */
int pci_config_init(void)
{
	if (pci_config_lib_init)
		return pci_config_lib_init();
	else
		return 0;
}

void pci_config_register(void *config)
{
	if (pci_config_lib_register)
		pci_config_lib_register(config);
}

/* Return the number of PCI busses available in the system, note that
 * there are always one bus (bus0) after the PCI library has been
 * initialized and a driver has been registered.
 */
int pci_bus_count(void)
{
	return pci_bus_cnt;
}
