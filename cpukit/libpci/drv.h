/* PCI routines and definiitions for PCI Host drivers */

#ifdef __PCI_DRV_H__
#define __PCI_DRV_H__



/* Get assigned system IRQ to a */



struct pci_drv {

	/* Operations */
	struct pci_cfg_ops cfgops;
	struct pci_mem_ops memops;
	struct pci_io_ops ioops;


	

	unsigned char (*get_assigned_irq)(unsigned char, unsigned char,  unsigned char,
			       unsigned char);
};


/* Register PCI driver to PCI system 
 *
 * Arguments
 *  
 * 
 * Return values
 *  0         Successfully installed PCI driver
 *  -1        Failure to install driver (driver already installed)
 */
extern int PCI_drv_register(struct pci_drv *drv, void *custom);


#endif /* !__PCI_DRV_H__ */
