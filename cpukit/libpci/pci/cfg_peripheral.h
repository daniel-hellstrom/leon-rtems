/* PCI Peripheral Configuration Library */

#ifndef __PCI_CFG_PERIPHERAL_H__
#define __PCI_CFG_PERIPHERAL_H__

/* The user must provide a PCI configuration using the "struct pci_bus pci_hb"
 * structure. Nothing else than setting pci_system_type and pci_bus_cnt is done
 * by the peripheral library.
 */
extern int pci_config_peripheral(void);

#endif
