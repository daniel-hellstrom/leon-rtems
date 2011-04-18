/* Routines to access PCI memory/configuration space and other PCI related
 * functions the PCI Library provides.
 */

#ifndef __PCI_ACCESS_H__
#define __PCI_ACCESS_H__

#include <stdint.h>
#include <libcpu/byteorder.h>
#include <pci.h>

/* Let BSP configure load/store from PCI */
#include <bsp.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Identification of a PCI configuration space device (16-bit) */
typedef uint16_t pci_dev_t;
/* Create a PCI Configuration Space ID */
#define PCI_DEV(bus, slot, func) (((bus)<<8) | ((slot)<<3) | (func))
/* Get Bus of a PCI Configuration Space ID */
#define PCI_DEV_BUS(dev) (((dev) >> 8) & 0xff)
/* Get Slot/Device of a PCI Configuration Space ID */
#define PCI_DEV_SLOT(dev) (((dev) >> 3) & 0x1f)
/* Get Function of a PCI Configuration Space ID */
#define PCI_DEV_FUNC(dev) ((dev) & 0x7)
/* Get Device and Function of a PCI Configuration Space ID */
#define PCI_DEV_DEVFUNC(dev) ((dev) & 0xff)
/* Expand Device into argument lists */
#define PCI_DEV_EXPAND(dev) PCI_DEV_BUS(dev),PCI_DEV_SLOT(dev),PCI_DEV_FUNC(dev)

/* Configuration Space Read/Write Operations */
struct pci_cfg_ops {
	/* Configuration Space Access and Setup Routines */
	int (*read8)(pci_dev_t dev, int ofs, uint8_t *data);
	int (*read16)(pci_dev_t dev, int ofs, uint16_t *data);
	int (*read32)(pci_dev_t dev, int ofs, uint32_t *data);
	int (*write8)(pci_dev_t dev, int ofs, uint8_t data);
	int (*write16)(pci_dev_t dev, int ofs, uint16_t data);
	int (*write32)(pci_dev_t dev, int ofs, uint32_t data);
};

/* Read a register over PCI I/O Space, and swap it if necessary */
struct pci_io_ops {
	uint8_t (*read8)(uint8_t *adr);
	uint16_t(*read16)(uint16_t *adr);
	uint32_t (*read32)(uint32_t *adr);
	void (*write8)(uint8_t *adr, uint8_t data);
	void (*write16)(uint16_t *adr, uint16_t data);
	void (*write32)(uint32_t *adr, uint32_t data);
};

struct pci_access_drv {
	/* Configuration */
	struct pci_cfg_ops cfg;

	/* I/O Access operations */
	struct pci_io_ops io;

	/* Translate from PCI address to CPU address (dir=0) The address
	 * will be used to perform I/O access or memory access by caller.
	 */
	int (*translate)(void **address, int type, int dir);
};

/* Access Routines valid after a PCI-Access-Driver has registered */
extern struct pci_access_drv pci_access_ops;

/* Register PCI Access Driver */
extern int pci_access_drv_register(struct pci_access_drv *drv);

/* Set/unset bits in command and status register of a PCI device */
extern void pci_modify_cmdsts(pci_dev_t dev, uint32_t mask, uint32_t val);

/* Enable Memory in command register */
static inline void pci_mem_enable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_MEMORY, PCI_COMMAND_MEMORY);
}

static inline void pci_mem_disable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_MEMORY, 0);
}

static inline void pci_io_enable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_IO, PCI_COMMAND_IO);
}

static inline void pci_io_disable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_IO, 0);
}

static inline void pci_master_enable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_MASTER, PCI_COMMAND_MASTER);
}

static inline void pci_master_disable(pci_dev_t dev)
{
	pci_modify_cmdsts(dev, PCI_COMMAND_MASTER, 0);
}

/* Configuration Space Access Read Routines */
extern int pci_cfg_r8(pci_dev_t dev, int ofs, uint8_t *data);
extern int pci_cfg_r16(pci_dev_t dev, int ofs, uint16_t *data);
extern int pci_cfg_r32(pci_dev_t dev, int ofs, uint32_t *data);

/* Configuration Space Access Write Routines */
extern int pci_cfg_w8(pci_dev_t dev, int ofs, uint8_t data);
extern int pci_cfg_w16(pci_dev_t dev, int ofs, uint16_t data);
extern int pci_cfg_w32(pci_dev_t dev, int ofs, uint32_t data);

/* Read a register over PCI I/O Space */
extern uint8_t pci_io_r8(uint32_t adr);
extern uint16_t pci_io_r16(uint32_t adr);
extern uint32_t pci_io_r32(uint32_t adr);

/* Write a register over PCI I/O Space */
extern void pci_io_w8(uint32_t adr, uint8_t data);
extern void pci_io_w16(uint32_t adr, uint16_t data);
extern void pci_io_w32(uint32_t adr, uint32_t data);

/* Get Translate PCI address into CPU accessible address */
static inline int pci_address(void **address, int type)
{
	return pci_access_ops.translate(address, type, 0);
}

/*** Read a register over PCI Memory Space ***/

static inline uint8_t pci_ld8(volatile uint8_t *addr)
{
	return *addr;
}

static inline void pci_st8(volatile uint8_t *addr, uint8_t val)
{
	*addr = val;
}

#ifdef CONFIGURE_PCI_BIG_ENDIAN

/* BSP has decided Big Endian PCI Bus (non-standard) */

static inline uint16_t pci_ld_le16(volatile uint16_t *addr)
{
	return ld_be16(addr);
}

static inline void pci_st_le16(volatile uint16_t *addr, uint16_t val)
{
	st_be16(addr, val);
}

static inline uint32_t pci_ld_le32(volatile uint32_t *addr)
{
	return ld_be32(addr);
}

static inline void pci_st_le32(volatile uint32_t *addr, uint32_t val)
{
	st_be32(addr, val);
}

static inline uint16_t pci_ld_be16(volatile uint16_t *addr)
{
	return ld_le16(addr);
}

static inline void pci_st_be16(volatile uint16_t *addr, uint16_t val)
{
	st_le16(addr, val);
}

static inline uint32_t pci_ld_be32(volatile uint32_t *addr)
{
	return ld_le32(addr);
}

static inline void pci_st_be32(volatile uint32_t *addr, uint32_t val)
{
	st_le32(addr, val);
}

#else

/* Little Endian PCI Bus */

static inline uint16_t pci_ld_le16(volatile uint16_t *addr)
{
	return ld_le16(addr);
}

static inline void pci_st_le16(volatile uint16_t *addr, uint16_t val)
{
	st_le16(addr, val);
}

static inline uint32_t pci_ld_le32(volatile uint32_t *addr)
{
	return ld_le32(addr);
}

static inline void pci_st_le32(volatile uint32_t *addr, uint32_t val)
{
	st_le32(addr, val);
}

static inline uint16_t pci_ld_be16(volatile uint16_t *addr)
{
	return ld_be16(addr);
}

static inline void pci_st_be16(volatile uint16_t *addr, uint16_t val)
{
	st_be16(addr, val);
}

static inline uint32_t pci_ld_be32(volatile uint32_t *addr)
{
	return ld_be32(addr);
}

static inline void pci_st_be32(volatile uint32_t *addr, uint32_t val)
{
	st_be32(addr, val);
}

#endif

#ifdef __cplusplus
}
#endif

#endif /* !__PCI_ACCESS_H__ */
