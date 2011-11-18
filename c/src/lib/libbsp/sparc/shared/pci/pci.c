/*  PCI configuration Library
 *
 *  COPYRIGHT (c) 2010.
 *  Aeroflex Gaisler Research
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2009-01-03, Daniel Hellstrom <daniel@gaisler.com>
 *   Host driver extracted, interrupt assignment implemented,
 *   BAR assigment sort implementation speeded up drastically.
 *  2009-04-29, Daniel Hellstrom <daniel@gaisler.com>
 *   Added basic support for multiple PCI buses and bridges,
 *   Added support for I/O areas. Added pci_print() function
 *   for printing current PCI configuration space set up.
 *  2010-02-03, Daniel Hellstrom <daniel@gaisler.com>
 *   Removed old Copyright notice (everything has been
 *   reimplemented), however some of the interface is compatible
 *   with the old interface in pci.h (BusCountPCI, pci_*_enable)
 *  2010-02-03, Daniel Hellstrom <daniel@gaisler.com>
 *   Fixed initialization problem when first device is a bridge.
 *  2010-04-19, Daniel Hellstrom <daniel@gaisler.com>
 *   Fixed autoconf issue when bridges are present
 *  2010-04-19, Daniel Hellstrom <daniel@gaisler.com>
 *   Optimized resource allocation when bridges are present: the
 *   resources lists are sorted by boundary instead of size and a
 *   reorder aligorithm introduced that move resources into unused
 *   areas if possible.
 *  2010-06-10, Daniel Hellstrom <daniel@gaisler.com>
 *   Fix in pci_res_insert(), where the above mentioned optimization
 *   failed due to bad compare statement. Optimization only affects
 *   systems with multiple PCI buses.
 *  2010-09-29, Kristoffer Glembo <kristoffer@gaisler.com>
 *   Fixed I/O BAR size calculation of bridges. Reading/Writing to 0x1C
 *   instead of faulty 0x1E.
 *  2010-06-10, Daniel Hellstrom <daniel@gaisler.com>
 *   Disable MEM and I/O Space accesses for devices which resource
 *   allocation fails for.
 */

#include <pci.h>
#include <stdlib.h>
#include <rtems/bspIo.h>
#include <string.h>

/* Define PCI_INFO to get a listing of configured devices at boot time */
#define PCI_INFO_FUNCTION
#undef PCI_INFO_ON_STARTUP

/* #define DEBUG */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

/* PCI Library */
#define READ_CONFIG_BYTE(args...) pci_read_config_byte(args)
#define READ_CONFIG_WORD(args...) pci_read_config_word(args)
#define READ_CONFIG_DWORD(args...) pci_read_config_dword(args)
#define WRITE_CONFIG_BYTE(args...) pci_write_config_byte(args)
#define WRITE_CONFIG_WORD(args...) pci_write_config_word(args)
#define WRITE_CONFIG_DWORD(args...) pci_write_config_dword(args)
#define GET_ASSIGNED_IRQ(args...) pci_get_assigned_irq(args)

/* For debugging it might be good to use the functions directly */
#if 0
/* GRPCI driver functions */
#define READ_CONFIG_BYTE(args...) grpci_read_config_byte(args)
#define READ_CONFIG_WORD(args...) grpci_read_config_word(args)
#define READ_CONFIG_DWORD(args...) grpci_read_config_dword(args)
#define WRITE_CONFIG_BYTE(args...) grpci_write_config_byte(args)
#define WRITE_CONFIG_WORD(args...) grpci_write_config_word(args)
#define WRITE_CONFIG_DWORD(args...) grpci_write_config_dword(args)
#define GET_ASSIGNED_IRQ(args...) grpci_get_assigned_irq(args)
#endif

/* Number of buses */
unsigned char pci_bus_cnt = 0;

/* PCI Config Library initialized */
int pci_inited = 0;

struct pci_res;
struct pci_dev;

#define PCI_RES_IO 1
#define PCI_RES_MEMIO 2
#define PCI_RES_MEM 3

/* Sorted BAR Resouces list entry */
struct pci_res {
    struct pci_res    *next;
    unsigned int      size;
    unsigned int      boundary;
    unsigned char     type; /* I/O, MEM or MEMIO */
    unsigned char     bar;
    struct pci_dev    *dev;
    /* Assigned Resource, zero if not assigned */
    unsigned int      start;
    unsigned int      end;
};

#define PCI_DEV_BRIDGE      0x01
#define PCI_DEV_IO_EN       0x10
#define PCI_DEV_MEM_EN      0x20
#define PCI_DEV_MEMIO_EN    0x40
#define PCI_DEV_DISABLED    0x80 /*Not enough resources for device (disabled)*/

struct pci_dev {
    struct pci_dev  *next;
    unsigned int    flags;
    unsigned int    busdevfun;
    struct pci_dev  *devs;      /* If device is a bridge */
    struct pci_res  *memio;     /* Non-Prefetchable memory resources on the bus */
    struct pci_res  *mem;       /* Prefetchable memory resources on the bus */
    struct pci_res  *io;        /* I/O resources on the bus */
};

struct pci_dev *pci_dev_root = NULL;
struct pci_res *pci_res_io = NULL, *pci_res_memio = NULL, *pci_res_mem = NULL;

/* Insert BAR into the sorted resources list. The BARs are sorted on the
 * BAR size/alignment need.
 */
struct pci_res *pci_res_insert(
    struct pci_res **root,
    unsigned int size,
    unsigned int boundary,
    struct pci_dev *dev,
    int bar,
    unsigned char type
    )
{
    struct pci_res *res, *curr, *last;
    unsigned int curr_size_resulting_boundary, size_resulting_boundary;

    /* Allocate new resource */
    res = malloc(sizeof(struct pci_res));
    if ( !res )
        return NULL;
    res->size = size;
    res->boundary = boundary;
    res->type = type;
    res->dev = dev;
    res->bar = bar;
    res->next = NULL;
    res->start = 0;
    res->end = 0;

    /* Insert the resources depending on the boundary needs 
     * Normally the boundary=size of the BAR, however when
     * PCI bridges are involved the bridge's boundary may be
     * smaller that the size due to the fact that a bridge
     * may have different-sized BARs behind, the largest BAR
     * (also the BAR with the largest boundary) will decide
     * the alignment need.
     */
    last = NULL;
    curr = *root;

    /* Order List after boundary, the boundary is maintained
     * when the size is on an equal boundary, normally it is
     * but may not be with bridges. So in second hand it is
     * sorted after resulting boundary - the boundary after
     * the resource.
     */
    while ( curr && (curr->boundary >= boundary) ) {
        if ( curr->boundary == boundary ) {
            /* Find Resulting boundary of size */
            size_resulting_boundary = 1;
            while ( (size & size_resulting_boundary) == 0 )
                size_resulting_boundary = size_resulting_boundary<<1;

            /* Find Resulting boundary of curr->size */
            curr_size_resulting_boundary = 1;
            while ( (curr->size & curr_size_resulting_boundary) == 0 )
                curr_size_resulting_boundary = curr_size_resulting_boundary<<1;

            if ( size_resulting_boundary >= curr_size_resulting_boundary ) {
                break;
            }
        }
        last = curr;
        curr = curr->next;
    }

    if ( last == NULL ) {
        /* Insert first in list */
        res->next = *root;
        *root = res;
    } else {
        last->next = res;
        res->next = curr;
    }
    return res;
}

#ifdef DEBUG
void pci_res_list_print(struct pci_res *root)
{
    if ( !root )
        return;

    printf("RESOURCE LIST:\n");
    while ( root ) {
        printf(" SIZE: 0x%08x, BOUNDARY: 0x%08x\n", root->size, root->boundary);
        root = root->next;
    }
}
#endif

/* Reorder a size/alignment ordered resources list. The idea is to
 * avoid unused due to alignment/size restriction.
 *
 * NOTE: The first element is always untouched.
 * NOTE: If less than three elements in list, nothing will be done
 *
 * Normally a BAR has the same alignment requirements as the size of the
 * BAR. However, when bridges are invloved the alignment need may be smaller
 * that the size, because a bridge resource consist or multiple BARs.
 * For example, say that a bridge with a 256Mb and a 16Mb BAR is found, then
 * the alignment is required to be 256Mb but the size 256+16Mb.
 *
 * In order to minimize dead space on the bus, the bounadry ordered list
 * is reordered, example:
 *  BUS0
 *  |           BUS1
 *  |------------|
 *  |            |-- BAR0: SIZE=256Mb, ALIGNMENT=256MB
 *  |            |-- BAR1: SIZE=16Mb, ALIGNMENT=16MB
 *  |            |
 *  |            |
 *  |            |
 *  |            |           BUS2 (BAR_BRIDGE1: SIZE=256+16, ALIGNEMENT=256)
 *  |            |------------|
 *  |            |            |-- BAR2: SIZE=256Mb, ALIGNMENT=256Mb
 *  |            |            |-- BAR3: SIZE=16Mb, ALIGNMENT=16MB
 *
 * A alignement/boundary ordered list of BUS1 will look like:
 *    - BAR_BRIDGE1
 *    - BAR0          (ALIGMENT NEED 256Mb)
 *    - BAR1
 *
 * However, Between BAR_BRIDGE1 and BAR0 will be a unused hole of 256-16Mb.
 * We can put BAR1 before BAR0 to avoid the problem.
 */
void pci_res_reorder(struct pci_res *root)
{
    struct pci_res *curr, *last, *curr2, *last2;
    unsigned int start, start_next, hole_size, hole_boundary;

    curr = root;
    if ( curr ) {
        /* Make up a start address with the boundary of the
         * First element.
         */
        start = curr->boundary + curr->size;
        last = curr;
        curr = curr->next;
    }
    while ( curr ) {

        /* Find start address of resource */
        start_next = (start + (curr->boundary - 1)) & ~(curr->boundary - 1);

        /* Find hole size, the unsed space inbetween last resource and next */
        hole_size = start_next - start;

        /* Find Boundary of START */
        hole_boundary = 1;
        while ( (start & hole_boundary) == 0 )
            hole_boundary = hole_boundary<<1;

        /* Detect dead hole */
        if ( hole_size > 0 ) {
            /* Step through list and try to find a resource that
             * can fit into hole. Take into account hole start
             * boundary and hole size.
             */
            last2 = curr;
            curr2 = curr->next;
            while ( curr2 ) {
                if ( (curr2->boundary <= hole_boundary) && 
                     (curr2->size <= hole_size) ) {
                    /* Found matching resource. Move it first in the
                     * hole. Then rescan, now that the hole has 
                     * changed in size/boundary.
                     */
                    last2->next = curr2->next;
                    curr2->next = curr;
                    last->next = curr2;

                    /* New Start address */
                    start_next = (start + (curr2->boundary - 1)) & 
                                ~(curr2->boundary - 1);
                    /* Since we inserted the resource before curr we need
                     * to re-evaluate curr one more, more resources may fit
                     * into the shruken hole.
                     */
                    curr = curr2;
                    break;
                }
                last2 = curr2;
                curr2 = curr2->next;
            }
        }

        /* No hole or nothing fitted into hole. */
        start = start_next;

        last = curr;
        curr = curr->next;
    }
}

/* Find the total size required in PCI address space needed by a resource list*/
unsigned int pci_res_size(struct pci_res *root)
{
    struct pci_res *curr;
    unsigned int size;

    /* Get total size of all resources */
    size = 0;
    curr = root;
    while ( curr ) {
        size = (size + (curr->boundary - 1)) & ~(curr->boundary - 1);
        size += curr->size;
        curr = curr->next;
    }

    return size;
}

/* Free the complete resource list */
void pci_res_free(struct pci_res **root)
{
    struct pci_res *res, *tmp;

    res = *root;
    while (res) {
        tmp = res;
        res = res->next;
        free(tmp);
    }
    *root = NULL;
}

/* Free the complete device list */
void pci_dev_free(struct pci_dev **root)
{
    struct pci_dev *dev, *tmp;

    dev = *root;
    while (dev) {
        /* Free all resources if bridge */
        if ( dev->memio )
            pci_res_free(&dev->memio);
        if ( dev->mem )
            pci_res_free(&dev->mem);
        if ( dev->io )
            pci_res_free(&dev->io);
        if ( dev->devs )
            pci_dev_free(&dev->devs);

        /* Next device */
        tmp = dev;
        dev = dev->next;
        free(tmp);
    }
    *root = NULL;
}

/* Driver configuration */
pci_config BSP_pci_configuration = 
{
    0,
    0,
    NULL
};

/* PCI Memory configuration */
pci_mem_config BSP_pci_mem_configuration = 
{
    0,
    0,
    0,
    0
};

void pci_mem_enable(unsigned char bus, unsigned char slot, unsigned char function)
{
    unsigned int data;

    READ_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, &data);
    WRITE_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, data | PCI_COMMAND_MEMORY);  

}

void pci_io_enable(unsigned char bus, unsigned char slot, unsigned char function)
{
    unsigned int data;

    READ_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, &data);
    WRITE_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, data | PCI_COMMAND_IO);  

}

void pci_mem_disable(unsigned char bus, unsigned char slot, unsigned char function)
{
    unsigned int data;

    READ_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, &data);
    WRITE_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, data & (~PCI_COMMAND_MEMORY));
}

void pci_io_disable(unsigned char bus, unsigned char slot, unsigned char function)
{
    unsigned int data;

    READ_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, &data);
    WRITE_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, data & (~PCI_COMMAND_IO));
}

void pci_master_enable(unsigned char bus, unsigned char slot, unsigned char function)
{
    unsigned int data;

    READ_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, &data);
    WRITE_CONFIG_DWORD(bus, slot, function, PCI_COMMAND, data | PCI_COMMAND_MASTER);  

}

struct pci_dev *pci_dev_create(void)
{
    struct pci_dev *dev;
    dev = (struct pci_dev *)malloc(sizeof(struct pci_dev));
    if ( !dev )
        exit(-1);
    memset(dev, 0, sizeof(struct pci_dev));
    return dev;
}

int pci_find_devs(int primary, int secondary, int *nrbuses, struct pci_dev **devs)
{
    unsigned int slot, numfuncs, func, id, pos, size, tmp, i, swapped, addr, fn;
    unsigned char header;
    unsigned char irq_line, irq_pin;
    unsigned int end;
    int bus, sbus, nbus;
    struct pci_dev *dev, *prev;

    bus = secondary;
    sbus = secondary;
    *nrbuses = 0;
    *devs = NULL;
    dev = prev = NULL;

    DBG("Scanning bus %x, %x\n", primary, secondary);
    slot = 0;
    if ( bus == 0 ) {
        slot = 1;   /* Skip PCI HOST BRIDGE */
    }
    for(; slot < PCI_MAX_DEVICES; slot++) {

        READ_CONFIG_DWORD(bus, slot, 0, PCI_VENDOR_ID, &id);

        if(id == PCI_INVALID_VENDORDEVICEID || id == 0) {
            /*
             * This slot is empty
             */
            continue;
        }
        DBG("Found PCIDEV 0x%x at (bus %x, slot %x)\n", id, bus, slot);

        READ_CONFIG_BYTE(bus, slot, 0, PCI_HEADER_TYPE, &header);

        if (header & PCI_MULTI_FUNCTION) {
            numfuncs = PCI_MAX_FUNCTIONS;
        }
        else {
            numfuncs = 1;
        }

        for(func = 0; func < numfuncs; func++) {

            READ_CONFIG_DWORD(bus, slot, func, PCI_VENDOR_ID, &id);
            if(id == PCI_INVALID_VENDORDEVICEID || id == 0) {
                continue;
            }

            prev = dev;
            dev = pci_dev_create();
            if ( prev ) {
                prev->next = dev;
            }
            if ( *devs == NULL ) {
                *devs = dev;
            }
            dev->next = NULL;
            dev->flags = 0;
            dev->busdevfun = (bus<<16) | (slot<<3) | func;
            dev->devs = NULL;

            READ_CONFIG_DWORD(bus, slot, func, PCI_CLASS_REVISION, &tmp);
            tmp >>= 16;
            if (tmp == PCI_CLASS_BRIDGE_PCI) {
                DBG("Found PCI-PCI Bridge 0x%x at (bus %x, slot %x)\n", id, bus, slot);

                dev->flags |= PCI_DEV_BRIDGE;
                dev->devs = NULL;
                nbus = 0;
                sbus++;

                /* Configure bridge */
                WRITE_CONFIG_DWORD(bus, slot, func, 0x28, 0);   /* No support for 64 bit */
                WRITE_CONFIG_DWORD(bus, slot, func, 0x2C, 0);
                WRITE_CONFIG_BYTE(bus, slot, func, 0x1a, 0xff);
                WRITE_CONFIG_BYTE(bus, slot, func, 0x19, sbus);
                WRITE_CONFIG_BYTE(bus, slot, func, 0x18, primary);

                pci_find_devs(bus, sbus, &nbus, &dev->devs);
                DBG("PCI-PCI BRIDGE: Primary %x, Secondary %x, Subordinate %x\n", bus, sbus, sbus + nbus);
                sbus += nbus;
                *nrbuses = *nrbuses + nbus + 1;
                WRITE_CONFIG_BYTE(bus, slot, func, 0x1a, sbus);
            }
        }
    }
    return 0;
}

void pci_find_bar(
    struct pci_dev *dev,
    int bar,
    struct pci_res **memio,
    struct pci_res **mem,
    struct pci_res **io
    )
{
    unsigned int size;
    unsigned char type;
    struct pci_res **res;
    int bus, slot, func;
    char *str;

    bus = dev->busdevfun >> 16;
    slot = (dev->busdevfun >> 3) & 0xff;
    func = dev->busdevfun & 0x7;

    DBG("Bus: %x, Slot: %x, function: %x, bar%d\n", bus, slot, func, bar);

    WRITE_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + (bar<<2), 0xffffffff);
    READ_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + (bar<<2), &size);

    /* TODO: Handle other MEM bar-types then 00 */
    if (size == 0 || size == 0xffffffff || (((size & 0x1) == 0) && (size & 0x6)) ) {
        WRITE_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + (bar<<2), 0);
        return;
    } else {
        unsigned int mask = ~0xf;
        if ( size & 0x1 ) {
            /* I/O */
            mask = ~0x3;
            res = io;
            str = "I/O";
            /* Limit size of I/O space to 256 byte */
            size |= 0xffffff00;
            type = PCI_RES_IO;
        } else if ( (size & 0x8) && (bus != 0) ) {
            /* Prefetchable memory behind bridge */
            res = mem;
            type = PCI_RES_MEM;
            str = "PREF MEM BEHIND BRIDGE";
        } else {
             /* non-prefetchable memory or prefetachable memory on host bus */
             res = memio;
             type = PCI_RES_MEMIO;
             if ( size & 0x8 ) 
                   str = "MEM";
               else
                   str = "MEMIO";
        }
        size &= mask;

        /* Insert BAR into the sorted resource list */
        pci_res_insert(res, ~size+1, ~size+1, dev, bar, type);

        DBG("Bus: %x, Slot: %x, function: %x, %s bar%d size: %x\n", bus, slot, func, str, bar, ~size+1);
    }
}

int pci_find_res(
    struct pci_dev *dev,
    struct pci_res **memio,
    struct pci_res **mem,
    struct pci_res **io
    )
{
    struct pci_dev *curr;
    struct pci_res **res;

    unsigned int bus, slot, func, pos, size;
    unsigned char irq_line, irq_pin;

    unsigned int tmp, limit, base, boundary;
    unsigned short tmp16;

    if ( dev->flags & PCI_DEV_BRIDGE) {

        /* PCI-PCI Bridge */

        /* Bridge BARs */
        for (pos = 0; pos < 2; pos++) {
            pci_find_bar(dev, pos, memio, mem, io);
        }

        /* Devices behind bridge */
        curr = dev->devs;
        dev->memio = NULL;
        dev->mem = NULL;
        dev->io = NULL;
        while (curr) {
            pci_find_res(curr, &dev->memio, &dev->mem, &dev->io);
            curr = curr->next;
        }

        /* Reorder resources to fit more optimally, to avoid unused PCI space.
         * Only required when 
         */
        pci_res_reorder(dev->memio);
        pci_res_reorder(dev->mem);
        pci_res_reorder(dev->io);

        /* All resources on bridge is located and stored in the resource list.
         *
         * The required memory space size of the bridge is the sum of all 
         * memory BARs on the bus. The size required is put into the primary
         * bus requirement of this bridge.
         */

        bus = dev->busdevfun >> 16;
        slot = (dev->busdevfun >> 3) & 0xff;
        func = dev->busdevfun & 0x7;

        /* I/O resources */
        size = pci_res_size(dev->io);
        if ( size > 0 ) {
            WRITE_CONFIG_WORD(bus, slot, func, 0x1c, 0xffff);
            tmp16 = 0;
            READ_CONFIG_WORD(bus, slot, func, 0x1c, &tmp16);
            if ( tmp16 != 0 ) {
                limit = (tmp16 & 0xf000);
                base = (tmp16 & 0x00f0) << 8;

                boundary = (limit | 0xfff) + 1; /* At least 4kb */
                size = (size + (boundary-1)) & ~(boundary-1);

                /* The boundary needs must be the needs of the largest BAR within the bus, 
                 * since the resouce list is sorted, the largest BAR boundary is the first
                 * in the list.
                 */
                
                if ( dev->io->boundary > boundary )
                    boundary = dev->io->boundary;
                pci_res_insert(io, size, boundary, dev, 2, PCI_RES_IO);

                DBG("Bus: %x, Slot: %x, function: %x, BRIDGE IO size: %x\n", bus, slot, func, size);
            }
        }
        WRITE_CONFIG_WORD(bus, slot, func, 0x1c, 0x00f0);
        WRITE_CONFIG_DWORD(bus, slot, func, 0x30, 0x0000ffff);

        /* non-prefetchable memory resources */
        size = pci_res_size(dev->memio);
        if ( size > 0 ) {
            WRITE_CONFIG_DWORD(bus, slot, func, 0x20, 0);
            READ_CONFIG_DWORD(bus, slot, func, 0x20, &tmp);
            limit = (tmp & 0xfff00000);
            base = (tmp & 0xfff0) << 16;

            boundary = (limit | 0xfffff) + 1; /* At least 1Mb */
            size = (size + (boundary-1)) & ~(boundary-1);

            /* The bounadry needs must be the needs of the largest BAR within the bus, 
             * since the resouce list is sorted, the largest BAR boundary is the first
             * in the list.
             */
            if ( dev->memio->boundary > boundary )
                    boundary = dev->memio->boundary;
            pci_res_insert(memio, size, boundary, dev, 3, PCI_RES_MEMIO);

            DBG("Bus: %x, Slot: %x, function: %x, BRIDGE MEMIO size: %x\n", bus, slot, func, size);
        }
        WRITE_CONFIG_DWORD(bus, slot, func, 0x24, 0x0000fff0);

        /* prefetchable memory resources */
        size = pci_res_size(dev->mem);
        if ( size > 0 ) {
            WRITE_CONFIG_DWORD(bus, slot, func, 0x24, 0xffffffff);
            READ_CONFIG_DWORD(bus, slot, func, 0x24, &tmp);
            if ( tmp & 0xfff0fff0 ) {
                WRITE_CONFIG_DWORD(bus, slot, func, 0x24, 0);
                READ_CONFIG_DWORD(bus, slot, func, 0x24, &tmp);

                limit = (tmp & 0xfff00000);
                base = (tmp & 0xfff0) << 16;
            
                boundary = (limit | 0xfffff) + 1; /* At least 1Mb */
                size = (size + (boundary-1)) & ~(boundary-1);

                if ( dev->mem->boundary > boundary )
                    boundary = dev->mem->boundary;
                pci_res_insert(mem, size, boundary, dev, 4, PCI_RES_MEM);

                DBG("Bus: %x, Slot: %x, function: %x, BRIDGE MEM size: %x\n", bus, slot, func, size);
            }
        }
        WRITE_CONFIG_DWORD(bus, slot, func, 0x24, 0x0000fff0);

    } else {
        /* Normal PCI Device */

        /* BARs */
        for (pos = 0; pos < 6; pos++) {
            pci_find_bar(dev, pos, memio, mem, io);
        }
    }
    bus = dev->busdevfun >> 16;
    slot = (dev->busdevfun >> 3) & 0xff;
    func = dev->busdevfun & 0x7;

    /* Set latency timer to 64 */
    READ_CONFIG_DWORD(bus, slot, func, 0xC, &tmp);    
    WRITE_CONFIG_DWORD(bus, slot, func, 0xC, (tmp & ~0xff00) | 0x4000);

    /* Put assigned system IRQ into PCI interrupt line information field. This is to
     * make it possible for drivers to read system IRQ from configuration space later on.
     */
    READ_CONFIG_BYTE(bus, slot, func, PCI_INTERRUPT_PIN, &irq_pin); /* Get Interrupt PIN */
    irq_line = GET_ASSIGNED_IRQ(bus, slot, func, irq_pin); /* Get IRQ assignment for PIN */
    WRITE_CONFIG_BYTE(bus, slot, func, PCI_INTERRUPT_LINE, irq_line); /* Set Interrupt LINE */

    return 0;
}

void pci_set_bar(struct pci_res *res)
{
    int bus, dev, fun;
    unsigned int tmp, tmp2;
    
    bus = res->dev->busdevfun >> 16;
    dev = (res->dev->busdevfun >> 3) & 0x1f;
    fun = res->dev->busdevfun & 0x7;

    if ( (res->dev->flags & PCI_DEV_BRIDGE) && (res->bar == 2) ) {
        /* PCI Bridge I/O bar */
        DBG("PCI[%x:%x:%x]: BAR 1C: 0x%x-0x%x\n", bus, dev, fun, res->start, res->end);
        if ( (res->end - res->start == 0) ) {
            /* Limit must be less than Base when invalid ==>
             * Largest Base, Smallest limit
             */
            tmp = 0xf0;
            tmp2 = 0x0000ffff;
        } else {
            /* Limit and Base */
            tmp  = ((res->end-1) & 0x0000f000) | ((res->start & 0x0000f000) >> 8);
            tmp2 = ((res->end-1) & 0xffff0000) | (res->start >> 16);
            /* Enable bridge master */
            pci_master_enable(bus, dev, fun);
        }
        DBG("PCI[%x:%x:%x]: BRIDGE BAR 0x%x: 0x%08x [0x30: 0x%x]\n", bus, dev, fun, 0x1C, tmp, tmp2);
        WRITE_CONFIG_WORD(bus, dev, fun, 0x1C, tmp);
        WRITE_CONFIG_DWORD(bus, dev, fun, 0x30, tmp2);
    } else if ( (res->dev->flags & PCI_DEV_BRIDGE) && (res->bar>2) ) {
        /* PCI Bridge MEM and MEMIO bar */
        if ( res->start == 0 ) {
            /* Limit must be less than Base when invalid ==>
             * Largest Base, Smallest limit
             */
            tmp = 0xfff0;
        } else {
            /* Limit and Base */
            tmp = ((res->end-1) & 0xfff00000) | (res->start >> 16);
            /* Enable bridge master */
            pci_master_enable(bus, dev, fun);
        }
        DBG("PCI[%x:%x:%x]: BRIDGE BAR 0x%x: 0x%08x\n", bus, dev, fun, 0x20+(res->bar-3)*4, tmp);
        WRITE_CONFIG_DWORD(bus, dev, fun, 0x20+(res->bar-3)*4, tmp);
    } else {
        /* PCI Device */
        DBG("PCI[%x:%x:%x]: DEV BAR%d: 0x%08x\n", bus, dev, fun, res->bar, res->start);
        WRITE_CONFIG_DWORD(bus, dev, fun, PCI_BASE_ADDRESS_0+res->bar*4, res->start);
    }
    if ( res->dev->flags & PCI_DEV_DISABLED ) {
        /* Disable I/O and memory access to disabled functions */
        pci_io_disable(bus, dev, fun);
	pci_mem_disable(bus, dev, fun);
    } else {
        /* Enable memory */
        if ( res->type == PCI_RES_IO )  {
            pci_io_enable(bus, dev, fun);
        } else {
            pci_mem_enable(bus, dev, fun);
        }
    }
}

/* Function assumes that base is properly aligned to the requirement of the largest BAR
 * in the system.
 */
unsigned int pci_alloc_res(struct pci_res *busres, unsigned int start, unsigned int end)
{
    struct pci_dev *dev;
    struct pci_res *res;
    unsigned int starttmp;

    int bus, slot, fun;

    /* The resources are sorted on their size (size and alignment is the same) */
    res = busres;
    while ( res ) {

        /* Check that device is enabled */
        if (res->dev->flags & PCI_DEV_DISABLED) {
            res = res->next;
            continue;
        }

        /* Align start to this resource's need, only needed after 
         * a bridge resource has been allocated.
         */
        starttmp = (start + (res->boundary-1)) & ~(res->boundary-1);
        
        if ( (starttmp + res->size - 1) > end ) {
            /* Not enough memory available for this resource */
            bus = res->dev->busdevfun >> 16;
            slot = (res->dev->busdevfun >> 3) & 0x1f;
            fun = res->dev->busdevfun & 0x7;
            printk("PCI[%x:%x:%x]: DEV BAR%d (%d): no resource assigned\n", bus, slot, fun, res->bar, res->type);
            res->start = res->end = 0;
            res->dev->flags |= PCI_DEV_DISABLED;
            pci_set_bar(res);
            res = res->next;
            continue;
        }
        start = starttmp;

        res->start = start;
        res->end = start+res->size;
        pci_set_bar(res);

        dev = res->dev;
        if ( (dev->flags & PCI_DEV_BRIDGE) && (res->bar>1) ) {
            /* A bridge resource need all it's devices configured. */
            if ( res->type == PCI_RES_IO ) {
                pci_alloc_res(dev->io, res->start, res->end);
            } else if ( res->type == PCI_RES_MEMIO ) {
                pci_alloc_res(dev->memio, res->start, res->end);
            } else {
                pci_alloc_res(dev->mem, res->start, res->end);
            }
        }

        start += res->size;

        res = res->next;
    }
    return start;
}

#ifdef PCI_INFO_FUNCTION
void pci_print()
{
    unsigned int bus, slot, func, maxbars, numfuncs, tmp, tmp2, id, pos;
    unsigned char header;
    char *str;
    unsigned int base, limit;

    printf("\nPCI devices found and configured:\n");
    for (bus = 0; bus < BusCountPCI(); bus++) {
        slot = 0;
        if ( bus == 0)
            slot = 1;
        for (; slot < PCI_MAX_DEVICES; slot++) {

            READ_CONFIG_BYTE(bus, slot, 0, PCI_HEADER_TYPE, &header); 

            if (header & PCI_MULTI_FUNCTION) {
                numfuncs = PCI_MAX_FUNCTIONS;
            }
            else {
                numfuncs = 1;
            }

            for (func = 0; func < numfuncs; func++) {

                READ_CONFIG_DWORD(bus, slot, func, PCI_VENDOR_ID,  &id);

                if (id == PCI_INVALID_VENDORDEVICEID || id == 0) continue;

                maxbars = 6;
                str = "";
                READ_CONFIG_DWORD(bus, slot, func, PCI_CLASS_REVISION, &tmp);
                tmp >>= 16;
                if (tmp == PCI_CLASS_BRIDGE_PCI) {
                    maxbars = 2;
                    str = "(BRIDGE)";
                }

                printf("\nBus %x Slot %x function: %x [0x%x] %s\nVendor id: 0x%x, device id: 0x%x\n", bus, slot, func, ((slot<<11) | (func<<8)), str, id & 0xffff, id>>16);

                for (pos = 0; pos < maxbars; pos++) {
                    READ_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + pos*4, &tmp);
                    WRITE_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + pos*4, 0xffffffff);
                    READ_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + pos*4, &tmp2);
                    WRITE_CONFIG_DWORD(bus, slot, func, PCI_BASE_ADDRESS_0 + pos*4, tmp);

                    if (tmp2 != 0 && tmp2 != 0xffffffff && ((tmp2 & 0x1) || ((tmp2 & 0x6) == 0))) {
                        unsigned int mask = ~0xf;
                        if ( (tmp2 & 0x1) == 1 ) {
                            /* I/O Bar */
                            mask = ~3;
                            tmp2 = tmp2 | 0xffffff00;
                        }
                        tmp2 &= mask;
                        tmp2 = ~tmp2+1; /* Size of BAR */
                        if ( tmp2 < 0x1000 ) {
                            str = "B";
                        } else if ( tmp2 < 0x100000 ) {
                            str = "kB";
                            tmp2 = tmp2 / 1024;
                        } else {
                            str = "MB";
                            tmp2 = tmp2 / (1024*1024);
                        }
                        printf("\tBAR %d: %x [%d%s]\n", pos, tmp, tmp2, str);
                    }
                }
                if ( maxbars == 2) {
                    /* Print Bridge addresses */
                    tmp = 0;
                    READ_CONFIG_DWORD(bus, slot, func, 0x1C, &tmp);
                    if ( tmp != 0 ) {
                        base = (tmp & 0x00f0) << 8;
                        limit = (tmp & 0xf000) | 0xfff;
                        READ_CONFIG_DWORD(bus, slot, func, 0x30, &tmp);
                        base |= (tmp & 0xffff) << 16;
                        limit |= (tmp & 0xffff0000);
                        str = "ENABLED";
                        if ( limit < base )
                            str = "DISABLED";
                        printf("\tI/O:   BASE: 0x%08x, LIMIT: 0x%08x (%s)\n", base, limit, str);
                    }

                    READ_CONFIG_DWORD(bus, slot, func, 0x20, &tmp);
                    if ( tmp != 0 ) {
                        base = (tmp & 0xfff0) << 16;
                        limit = (tmp & 0xfff00000) | 0xfffff;
                        str = "ENABLED";
                        if ( limit < base )
                            str = "DISABLED";
                        printf("\tMEMIO: BASE: 0x%08x, LIMIT: 0x%08x (%s)\n", base, limit, str);
                    }

                    READ_CONFIG_DWORD(bus, slot, func, 0x24, &tmp);
                    if ( tmp != 0 ) {
                        base = (tmp & 0xfff0) << 16;
                        limit = (tmp & 0xfff00000) | 0xfffff;
                        str = "ENABLED";
                        if ( limit < base )
                            str = "DISABLED";
                        printf("\tMEM:   BASE: 0x%08x, LIMIT: 0x%08x (%s)\n", base, limit, str);
                    }
                }
                printf("\n");
            }
        }
    }
    printf("\n");
}
#endif

/* This routine assumes that PCI has been successfully initialized */
int init_pci()
{
    unsigned int nrbuses;
    struct pci_dev *dev;
    unsigned int start, end;
    unsigned int startmemio, endmemio;
    unsigned int startmem, endmem;
    unsigned int startio, endio;

    DBG("\n--- PCI MEMORY AVAILABLE ---\n");
    start = BSP_pci_mem_configuration.pci_mem_start;
    end = BSP_pci_mem_configuration.pci_mem_start + BSP_pci_mem_configuration.pci_mem_size;
    DBG(" [0x%08x-0x%08x]\n", start, end);

    DBG("\n--- PCI SCANNING ---\n");
    pci_dev_root = NULL;
    pci_find_devs(0, 0, &nrbuses, &pci_dev_root);
    if ( pci_dev_root == NULL )
        return 0;
    pci_bus_cnt = nrbuses + 1;

    DBG("\n\n--- PCI RESOURCES ---\n");

    pci_res_memio = pci_res_mem = pci_res_io = NULL;
    dev = pci_dev_root;
    while( dev ) {
        pci_find_res(dev, &pci_res_memio, &pci_res_mem, &pci_res_io);
        dev = dev->next;
    }

    /* Reorder resources to fit more optimally, to avoid unused PCI space.
     * Only required when 
     */
    pci_res_reorder(pci_res_memio);
    pci_res_reorder(pci_res_mem);
    pci_res_reorder(pci_res_io);

    start = BSP_pci_mem_configuration.pci_mem_start;
    end = BSP_pci_mem_configuration.pci_mem_start + BSP_pci_mem_configuration.pci_mem_size;

    /* Assign resources to non-prefetchable memory */
    startmemio = start;
    endmemio = pci_alloc_res(pci_res_memio, start, end);

    /* Assign resources to prefetchable memory */
    startmem = endmemio;
    endmem = pci_alloc_res(pci_res_mem, startmem, end);

    /* Assign resources to I/O areas */
    startio = BSP_pci_mem_configuration.pci_io_start;
    endio = BSP_pci_mem_configuration.pci_io_start + BSP_pci_mem_configuration.pci_io_size;
    DBG("IO RANGE [0x%x-0x%x]\n", startio, endio);
    endio = pci_alloc_res(pci_res_io, startio, endio);

    DBG("\n--- PCI MEMORY RANGE ---\n");
    DBG(" MEM NON-PREFETCHABLE: [0x%08x-0x%08x]\n", startmemio, endmemio);
    DBG(" MEM PREFETCHABLE:     [0x%08x-0x%08x]\n", startmem, endmem);
    DBG(" IO:                   [0x%08x-0x%08x]\n", startio, endio);

    DBG("PCI resource allocation done\n");

#ifdef PCI_INFO_ON_STARTUP
    pci_print();
#endif

    return 0;
}

/* Return the number of PCI busses available in the system, note that
 * there are always one bus (bus0) after the PCI library has been
 * initialized and a driver has been registered.
 */
unsigned char BusCountPCI()
{
    return pci_bus_cnt;
}

int pci_register_drv(
        pci_config *pcidrv,
        pci_mem_config *pcimemcfg,
        void *custom
        )
{
    if ( !pcidrv || !pcimemcfg || pci_inited )
        return -1;

    /* Register driver */
    BSP_pci_configuration = *pcidrv;

    /* Config memory space */
    BSP_pci_mem_configuration = *pcimemcfg;

    /* Allow only one registeration */
    pci_inited = 1;

    return 0;
}
