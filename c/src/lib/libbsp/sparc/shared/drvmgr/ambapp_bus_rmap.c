/*  RMAP AMBA Plug & Play bus driver. The AMBA bus is accessed over a SpaceWire network.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE. 
 *
 *  2009-11-20, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 *  2010-02-03, Daniel Hellstrom <daniel@gaisler.com>
 *    Added support for prefix in device name.
 *
 * MEMORY PARTITIONS
 * -----------------
 *  A partition represents a memory area, for example the SRAM of a system, the drivers
 *  may allocate from different partitions to make sure that the descriptors of buffers
 *  are located at the correct place.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <drvmgr/spw_bus.h>
#include <drvmgr/ambapp_bus.h>
#include <drvmgr/ambapp_bus_rmap.h>
#include <genirq.h>
#include <grlib.h>

#include <bsp.h>

/* Remote RMAP Access macros */
#define WRITE_REG(pDev, adr, value) priv->rw_w32((uint32_t *)adr, (uint32_t)value, &pDev->rw_arg)
#define READ_REG(pDev, adr) priv->rw_r32((uint32_t *)adr, &pDev->rw_arg)

#define PARTITION_MAX 4
#define INHERIT_LENGTH 12

#define DBG(args...)
/*#define DBG(args...) printk(args)*/

int ambapp_bus_rmap_debug = 0;

struct mem_block;

struct mem_block {
	struct mem_block	*next;
	struct mem_block	*prev;
	unsigned int		start;
	unsigned int		length;	
};

struct mem_partition {
	/* Free memory */
	struct mem_block	*head;
	struct mem_block	*tail;
};

struct ambapp_rmap_priv {
	struct drvmgr_dev	*dev;
	char			prefix[32];
	int			minor;
	struct ambapp_config	config;
	struct ambapp_bus	abus;

	/* MEMORY ALLOCATION */
	unsigned int		partition_valid;
	struct mem_partition	partitions[PARTITION_MAX];

	/* IRQ HANDLING */
	genirq_t		genirq;
	LEON3_IrqCtrl_Regs_Map	*irq;

	/* Access routines */
	struct drvmgr_func	funcs[INHERIT_LENGTH+3];
	struct drvmgr_rw_arg	rw_arg;
	spwbus_w32		rw_w32;
	spwbus_r32		rw_r32;
	spwbus_rmem		rw_rmem;
};

int ambapp_rmap_int_register(
	struct drvmgr_dev *dev,
	int irq,
	const char *info,
	drvmgr_isr isr,
	void *arg);
int ambapp_rmap_int_unregister(
	struct drvmgr_dev *dev,
	int irq,
	drvmgr_isr isr,
	void *arg);
int ambapp_rmap_int_unmask(struct drvmgr_dev *dev, int irq);
int ambapp_rmap_int_mask(struct drvmgr_dev *dev, int irq);
int ambapp_rmap_int_clear(struct drvmgr_dev *dev, int irq);
int ambapp_rmap_get_params(
	struct drvmgr_dev *dev,
	struct drvmgr_bus_params *params);
void ambapp_rmap_isr(void *arg);

int ambapp_rmap_init1(struct drvmgr_dev *dev);
int ambapp_rmap_init2(struct drvmgr_dev *dev);

void *ambapp_rmap_rw_arg(struct drvmgr_dev *dev);
void ambapp_rmap_rw_err(struct drvmgr_rw_arg *a, struct drvmgr_bus *bus,
			int funcid, void *adr);

struct ambapp_ops ambapp_rmap_ops = {
	.int_register = ambapp_rmap_int_register,
	.int_unregister = ambapp_rmap_int_unregister,
	.int_clear = ambapp_rmap_int_clear,
	.int_unmask = ambapp_rmap_int_unmask,
	.int_mask = ambapp_rmap_int_mask,
	.get_params = ambapp_rmap_get_params
};

struct spw_id spw_rmap_ids[] =
{
	{SPW_NODE_ID_GRLIB},
	{SPW_NODE_ID_NONE}
};

struct drvmgr_drv_ops ambapp_rmap_drv_ops = 
{
	.init = {ambapp_rmap_init1, ambapp_rmap_init2, NULL, NULL},
	.remove = NULL,
	.info = NULL,
};

struct spw_bus_drv_info ambapp_bus_drv_rmap = 
{
	{
		DRVMGR_OBJ_DRV,			/* Driver */
		NULL,				/* Next driver */
		NULL,				/* Device list */
		DRIVER_SPW_RMAP_AMBAPP_ID,	/* Driver ID */
		"AMBAPP_RMAP_DRV",		/* Driver Name */
		DRVMGR_BUS_TYPE_SPW_RMAP,	/* Bus Type */
		&ambapp_rmap_drv_ops,
		NULL,				/* Funcs */
		0,
		0,
	},
	&spw_rmap_ids[0],
	
};

int ambapp_rmap_inherit_list[INHERIT_LENGTH] =
{
	SPWBUS_R8,
	SPWBUS_R16,
	SPWBUS_R32,
	SPWBUS_R64,
	SPWBUS_W8,
	SPWBUS_W16,
	SPWBUS_W32,
	SPWBUS_W64,
	SPWBUS_RMEM,
	SPWBUS_WMEM,
	SPWBUS_MEMSET,
};

struct drvmgr_bus_res **ambapp_rmap_resources = NULL;
int ambapp_rmap_resources_cnt = 0;

void ambapp_rmap_register(void)
{
	drvmgr_drv_register(&ambapp_bus_drv_rmap.general);
}

void ambapp_rmap_set_resources(struct drvmgr_bus_res **resources, int cnt)
{
	ambapp_rmap_resources = resources;
	ambapp_rmap_resources_cnt = cnt;
}

void *ambapp_rmap_memcpy(
	void *dest,
	const void *src,
	int n,
	struct ambapp_bus *abus)
{
	struct ambapp_rmap_priv *priv = (struct ambapp_rmap_priv *)
		((unsigned int)abus -
		offsetof(struct ambapp_rmap_priv, abus));

	priv->rw_rmem(dest, src, n, &priv->rw_arg);

	return dest;
}

/* AMBA PP find routines */
int ambapp_rmap_dev_find(struct ambapp_dev *dev, int index, int maxdepth, void *arg)
{
	/* Found IRQ/GRPCI controller, stop */
	*(struct ambapp_dev **)arg = dev;
	return 1;
}

/* Function called from Driver Manager Initialization Stage 1 */
int ambapp_rmap_init1(struct drvmgr_dev *dev)
{
	struct ambapp_config *config;
	union drvmgr_key_value *value;
	int status, i, funcid;
	unsigned int ioarea, freq;
	struct ambapp_rmap_priv *priv;
	struct spw_bus_dev_info *businfo;
	struct ambapp_dev *tmp;
	char prefix[32];

	dev->priv = NULL;
	dev->name = "RMAP AMBA PnP";
	businfo = (struct spw_bus_dev_info *)dev->businfo;

	DBG("AMBAPP RMAP: intializing\n");

	/* Get Configuration */
	ioarea = 0xfff00000; /* Defualt IO Area */
	value = drvmgr_dev_key_get(dev, "IOArea", KEY_TYPE_INT);
	if ( value ) {
		ioarea = value->i;
	}
	freq = 0;
	value = drvmgr_dev_key_get(dev, "BusFreq", KEY_TYPE_INT);
	if ( value )
		freq = value->i;

	dev->priv = priv = malloc(sizeof(struct ambapp_rmap_priv));
	if ( !priv )
		return RTEMS_NO_MEMORY;
	memset(priv, 0, sizeof(struct ambapp_rmap_priv));

	priv->dev = dev;
	priv->genirq = genirq_init(16);

	/* Scan Amba Bus */
	status = ambapp_scan(&priv->abus, ioarea, ambapp_rmap_memcpy, NULL);
	if ( status ) {
		return -1;
	}

	if ( ambapp_bus_rmap_debug )
		ambapp_print(&priv->abus, 10);

	/* Find IRQ controller */
	tmp = NULL;
	status = ambapp_for_each(&priv->abus, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER, GAISLER_IRQMP, 10, ambapp_rmap_dev_find, &tmp);
	if ( (status != 1) || !tmp ) {
		return -4;
	}
	priv->irq = (LEON3_IrqCtrl_Regs_Map *)(((struct ambapp_apb_info *)tmp->devinfo)->start);
	/* Set up IRQ controller */
	WRITE_REG(priv, &priv->irq->iclear, 0xffff);
	WRITE_REG(priv, &priv->irq->ilevel, 0);
	WRITE_REG(priv, &priv->irq->mask[0], 0);

	/* Clear any old interrupt requests (IRQ IS LEVEL) */
	drvmgr_interrupt_clear(priv->dev, 0);

	/* Get Filesystem name prefix */
	prefix[0] = '\0';
	if ( drvmgr_get_dev_prefix(dev, prefix) ) {
		/* Failed to get prefix, make sure of a unique FS name
		 * by using the driver minor.
		 */
		sprintf(priv->prefix, "/dev/rmap_%02x",
			(unsigned char)businfo->dstadr);
	} else {
		/* Got special prefix, this means we have a bus prefix
		 * And we should use our "bus minor"
		 */
		sprintf(priv->prefix, "/dev/%srmap_%02x",
			prefix, (unsigned char)businfo->dstadr);
	}

	mkdir(priv->prefix, S_IRWXU | S_IRWXG | S_IRWXO);
	strcat(priv->prefix, "/");

	printf("\n\n--- RMAP AMBAPnP[%d] ---\n", dev->minor_drv);
	printf(" SpW ADDRESS: %d\n", businfo->dstadr);
	printf(" DEV BASE: %s\n", priv->prefix);
	printf(" FREQ: %u [Hz]\n", freq);
	printf(" I/O AREA: 0x%08x\n", ioarea);
	printf(" IRQ REGS: 0x%08x\n", (unsigned int)priv->irq);

	/* Initialize the Frequency of the AMBA bus */
	ambapp_freq_init(&priv->abus, NULL, freq);

	/* Get Read/Write operations for bus */
	priv->rw_arg.dev = dev;
	drvmgr_func_call(dev, SPWBUS_RW_ARG, &priv->rw_arg.arg, NULL, NULL, NULL);
	drvmgr_func_get(dev, SPWBUS_R32, &priv->rw_r32);
	drvmgr_func_get(dev, SPWBUS_W32, &priv->rw_w32);
	drvmgr_func_get(dev, SPWBUS_RMEM, &priv->rw_rmem);

	/* Inherit R/W functions from parent */
	for (i = 0; i < INHERIT_LENGTH; i++) {
		funcid = ambapp_rmap_inherit_list[i];
		drvmgr_func_get(dev, funcid, &priv->funcs[i].func);
		priv->funcs[i].funcid = funcid;
	}
	priv->funcs[i].funcid = AMBAPP_RMAP_RW_ARG;
	priv->funcs[i].func = ambapp_rmap_rw_arg;
	priv->funcs[i+1].funcid = AMBAPP_RMAP_RW_ERR;
	priv->funcs[i+1].func = ambapp_rmap_rw_err;
	priv->funcs[i+2].funcid = DRVMGR_FUNCID_NONE;

	config = &priv->config;
	config->ops = &ambapp_rmap_ops;
	config->mmaps = NULL;
	config->abus = &priv->abus;
	config->bus_type = DRVMGR_BUS_TYPE_AMBAPP_RMAP;
	config->funcs = priv->funcs;
	/* Set this AMBA Bus driver resources */
	if ( ambapp_rmap_resources && (priv->dev->minor_drv < ambapp_rmap_resources_cnt) ) {
		config->resources = ambapp_rmap_resources[priv->dev->minor_drv];
	} else {
		value = drvmgr_dev_key_get(dev, "BusRes", KEY_TYPE_POINTER);
		if ( value )
			config->resources = value->ptr;
		else
			config->resources = NULL;
	}

	/* Target is assumed to be equipped with 256Mb RAM */
	/*ambapp_rmap_partition_create(dev, 0, 0x40000000, 0x10000000);*/
	/*ambapp_rmap_partition_create_internal(priv, 0, 0x40000000, 0x10000000);*/

	/* Initialize the AMBA Bus */
	return ambapp_bus_register(dev, config);
}

int ambapp_rmap_init2(struct drvmgr_dev *dev)
{
	struct ambapp_rmap_priv *priv = dev->priv;

	/* Enable System IRQ so that the SpW Node Core's interrupt goes through.
	 *
	 * It is important to enable it in stage init2. If interrupts were
	 * enabled in init1 this might hang the system when more than one SpW
	 * Node is connected to the same IRQ line, this is because interrupts
	 * might be shared and Node 2 have not initialized and might therefore
	 * drive interrupt already when entering init1().
	 */
	drvmgr_interrupt_register(priv->dev, 0, "ambapp_rmap", ambapp_rmap_isr, (void *)priv);

	return 0;
}

/* The ISR is executed on the SpW-BUS ISR Task, ie. not in interrupt context */
void ambapp_rmap_isr (void *arg)
{
	struct ambapp_rmap_priv *priv = arg;
	unsigned int status, tmp;
	int irq;
	tmp = status = READ_REG(priv, &priv->irq->ipend);

	/* DBG("AMBAPP-RMAP-ISR: IRQ 0x%x\n",status); */

	/* Clear handled IRQs at remote IRQ controller, These IRQs are not
	 * level sensitive which makes it ok to clear the before they are
	 * handled.
	 */
	if ( status )
		WRITE_REG(priv, &priv->irq->iclear, status);

	for (irq=0; irq<16; irq++) {
		if ( status & (1<<irq) ) {
			genirq_doirq(priv->genirq, irq);
			status &= ~(1<<irq);
			if ( status == 0 )
				break;
		}
	}

	/* ACK interrupt, this is because Interrupt is Level, so the IRQ
	 * Controller still drives the IRQ. 
	 */
	if ( tmp )
		drvmgr_interrupt_clear(priv->dev, 0);

	DBG("AMBAPP-RMAP-ISR: 0x%x\n", tmp);
}

void *ambapp_rmap_rw_arg(struct drvmgr_dev *dev)
{
	struct ambapp_rmap_priv *priv;

	if (dev == NULL || dev->parent || dev->parent->dev)
		return (void *)DRVMGR_FAIL;

	priv = dev->parent->dev->priv;

	/* Use same argument as for ourselves */
	return priv->rw_arg.arg;
}

/* Called by SpaceWire Lay if a device directly on this bus has been witness
 * to an error.
 */
void ambapp_rmap_rw_err(struct drvmgr_rw_arg *a, struct drvmgr_bus *bus,
			int funcid, void *adr)
{
	printk("AMBAPP_RMAP: erraccess %p with 0x%08x (amba: %p, dev: %p)\n",
		adr, funcid, a->dev->parent, a->dev);
	/* Take some error action here?  Remove bus or just faulting device? */
}

int ambapp_rmap_int_register(
	struct drvmgr_dev *dev,
	int irq,
	const char *info,
	drvmgr_isr isr,
	void *arg)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;
	int status;
	unsigned int tmp;

	DBG("AMBAPP-RMAP-INT_REG: %d\n", irq);

	status = genirq_register(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Disable and clear IRQ for first registered handler */
		WRITE_REG(priv, &priv->irq->iclear, (1<<irq));
		tmp = READ_REG(priv, &priv->irq->mask[0]);
		WRITE_REG(priv, &priv->irq->mask[0], (tmp & ~(1<<irq))); /* mask interrupt source */
	} else if ( status == 1 )
		status = 0;

	if (status != 0)
		return DRVMGR_FAIL;

	status = genirq_disable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Enable IRQ for first enabled handler only */

		/* unmask interrupt source */
		tmp = READ_REG(priv, &priv->irq->mask[0]);
		WRITE_REG(priv, &priv->irq->mask[0], tmp | (1<<irq)); 
	} else if ( status == 1 )
		status = DRVMGR_OK;

	return status;
}

int ambapp_rmap_int_unregister(
	struct drvmgr_dev *dev,
	int irq,
	drvmgr_isr isr,
	void *arg)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;
	int status;
	unsigned int tmp;

	DBG("AMBAPP-RMAP-INT_UNREG: %d\n", irq);

	status = genirq_disable(priv->genirq, irq, isr, arg);
	if ( status == 0 ) {
		/* Disable IRQ only when no enabled handler exists */

		/* mask interrupt source */
		tmp = READ_REG(priv, &priv->irq->mask[0]);
		WRITE_REG(priv, &priv->irq->mask[0], tmp & ~(1<<irq));
	}

	status = genirq_unregister(priv->genirq, irq, isr, arg);
	if ( status != 0 )
		status = DRVMGR_FAIL;

	return status;
}

int ambapp_rmap_int_unmask(struct drvmgr_dev *dev, int irq)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;
	unsigned int tmp;

	DBG("AMBAPP-RMAP-INT_UNMASK: %d\n", irq);

	if ( genirq_check(priv->genirq, irq) )
		return DRVMGR_FAIL;

	/* unmask interrupt source */
	tmp = READ_REG(priv, &priv->irq->mask[0]);
	WRITE_REG(priv, &priv->irq->mask[0], tmp | (1<<irq)); 

	return DRVMGR_OK;
}

int ambapp_rmap_int_mask(struct drvmgr_dev *dev, int irq)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;
	unsigned int tmp;

	DBG("AMBAPP-RMAP-INT_MASK: %d\n", irq);

	if ( genirq_check(priv->genirq, irq) )
		return DRVMGR_FAIL;

	/* mask interrupt source */
	tmp = READ_REG(priv, &priv->irq->mask[0]);
	WRITE_REG(priv, &priv->irq->mask[0], tmp & ~(1<<irq)); 

	return DRVMGR_OK;
}

int ambapp_rmap_int_clear(struct drvmgr_dev *dev, int irq)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;

	if ( genirq_check(priv->genirq, irq) )
		return DRVMGR_FAIL;

	WRITE_REG(priv, &priv->irq->iclear, (1<<irq));

	return 0;
}

int ambapp_rmap_get_params(struct drvmgr_dev *dev, struct drvmgr_bus_params *params)
{
	struct ambapp_rmap_priv *priv = dev->parent->dev->priv;

	params->dev_prefix = &priv->prefix[5];
	return 0;
}

/************* MEMORY SERVICES FOR REMOTE TARGETS *************
 *
 * This was written with resource usage in mind, and alignment needs
 * that has to be fullfilled. It is typically used by drivers to allocate
 * large data areas that need a certain alignment.
 */

/* Caller know that there is enough memory within block, this function allocates it */
void *alloc_block(struct mem_partition *part, struct mem_block *block, unsigned int start, size_t size)
{
	struct mem_block *newblock;
	unsigned int block_end;
	
	DBG("ALLOC_BLOCK: 0x%x - 0x%x\n", start, start + size);

	/* At start of block? */
	if ( block->start == start ) {
		block->start += size;
		block->length -= size;
		return (void *)start;
	}

	/* At End of block? */
	block_end = block->start + block->length;
	if ( block_end == (start + size) ) {
		block->length -= size;
		return (void *)start;
	}

	/* Not at start or end means that we must insert a new block describing
	 * the area in between the two blocks.
	 */
	newblock = (struct mem_block *)malloc(sizeof(struct mem_block));
	if ( !newblock ) {
		return NULL;
	}

	/* 1. assign unused area after the allocated area to newblock
	 * 2. shink the first block that the area is taken from
	 * 3. insert newblock into free memory list.
	 *
	 * 1.
	 */
	newblock->start = start + size;
	newblock->length = block_end - (start + size);

	/* 2. */
	block->length = start - block->start;

	/* 3. */
	newblock->prev = block;
	newblock->next = block->next;
	if ( block->next ) {
		block->next->prev = newblock;
	} else {
		part->tail = newblock;
	}
	block->next = newblock;

	return (void *)start;
}

/* Try to allocate from a block */
void *alloc_try(struct mem_partition *part, struct mem_block *block, unsigned int start, size_t size)
{
	unsigned int end = start + size;
	unsigned int block_end = block->start + block->length;

	/* Is the section available in this block? */
	if ( (start >= block->start) && (start <= block_end) && /* Check start and end */
	     (end <= block_end) && (end > block->start) ) {
	     /* Found a block that is available, allocate it */
	     return alloc_block(part, block, start, size);
	}

	return NULL;
}

/* Iterate over all free memory blocks from the tail to head*/
void *alloc_back(struct mem_partition *part, size_t boundary, size_t size)
{
	struct mem_block *block;
	unsigned int start, block_end;
	void *mem;

	/* Find a section of memory matching the alignment and size */
	block = part->tail;
	while ( block ) {
		if ( block->length >= size ) {
			/* Calculate the last address that is needed within this block */
			block_end = block->start + block->length;
			start = (block_end - size) & ~(boundary - 1);

			if ( (mem=alloc_try(part, block, start, size)) != NULL ) {
				return mem;
			}
		}

		/* memory not available within this block */
		block = block->prev;
	}

	DBG("AMBAPP_RMAP_MEMALIGN_BACK: FAILED TO ALLOCATE %dBytes at a 0x%x boundary\n", size, boundary);

	return NULL;
}

void *alloc_front(struct mem_partition *part, size_t boundary, size_t size)
{
	struct mem_block *block;
	unsigned int start;
	void *mem;

	/* Find a section of memory matching the alignment and size */
	block = part->head;
	while ( block ) {

		if ( block->length >= size ) {
			/* Calculate the first address that is needed 
			 * within this block
			 */
			start = (block->start + (boundary - 1)) & 
				~(boundary - 1);

			if ((mem=alloc_try(part, block, start, size)) != NULL) {
				return mem;
			}
		}
		/* memory not available within this block */
		block = block->next;
	}

	DBG("AMBAPP_RMAP_MEMALIGN_FRONT: FAILED TO ALLOCATE %dBytes at a 0x%x boundary\n", size, boundary);

	return NULL;
}

void *ambapp_rmap_partition_memalign(
	struct drvmgr_dev *dev,
	int partition,
	size_t boundary,
	size_t size)
{
	struct ambapp_rmap_priv *priv;
	struct mem_partition *part = NULL;

	if ( size < 1 ) {
		DBG("AMBAPP_RMAP_MEMALIGN: size < 1\n");
		return NULL;
	}

	/* Never deal with smaller units than 16 bytes */
	if ( boundary < 0x10 ) {
		boundary = 0x10;
	}
	
	DBG("RMAP MEMALIGN: partition %d, boundary 0x%x, size 0x%x\n", partition, boundary, size);

	/* Get Partition */
	priv = dev->parent->dev->priv;
	if ( (partition >= PARTITION_MAX) || (priv->partition_valid & (1<<partition)) == 0 ) {
		DBG("RMAP MEMALIGN: partition invalid\n");
		return NULL;
	}
	part = &priv->partitions[partition];

	if ( (boundary >= 0x10000) || (size >= 0x10000) ) {
		return alloc_front(part, boundary, size);
	} else {
		return alloc_back(part, boundary, size);
	}
}

int ambapp_rmap_partition_create_internal(
	struct ambapp_rmap_priv *priv,
	int partition,
	unsigned int start,
	size_t size)
{
	struct mem_partition *part;
	struct mem_block *block;

	/* Get Partition */
	if ( (partition >= PARTITION_MAX) || (priv->partition_valid & (1<<partition)) ) {
		DBG("RMAP PART CREATE: invalid part num %d\n", partition);
		return -1;
	}
	part = &priv->partitions[partition];
	if ( part->head || part->tail ) { 
		DBG("RMAP PART CREATE: part %d already initied\n", partition);
		return -1;
	}

	/* Init on free memory block */
	block = (struct mem_block *)malloc(sizeof(struct mem_block));
	part->head = part->tail = block;
	block->next = block->prev = NULL;
	block->start = start;
	block->length = size;
	priv->partition_valid |= 1 << partition;

	DBG("RMAP PART CREATE: DONE initiing part %d\n", partition);

	return 0;
}

int ambapp_rmap_partition_create(
	struct drvmgr_dev *dev,
	int partition,
	unsigned int start,
	size_t size)
{
	struct ambapp_rmap_priv *priv;

	priv = dev->parent->dev->priv;

	return ambapp_rmap_partition_create_internal(priv, partition, start, size);
}
