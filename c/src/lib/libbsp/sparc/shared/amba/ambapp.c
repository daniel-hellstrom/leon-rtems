/*
 *  AMBA Plug & Play routines
 *
 *  COPYRIGHT (c) 2009.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id: ambapp.c,v 1.2 2009/11/29 15:33:27 ralf Exp $
 *
 *
 *  2008-12-01, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 */

#include <string.h>
#include <stdlib.h>
#include <string.h>

#include <ambapp.h>

#define AMBA_CONF_AREA 0xff000
#define AMBA_AHB_SLAVE_CONF_AREA (1 << 11)
#define AMBA_APB_SLAVES 16

/* Allocate */
struct ambapp_dev *ambapp_alloc_dev_struct(int dev_type)
{
	int size = sizeof(struct ambapp_dev);
	struct ambapp_dev *dev;

	if ( dev_type == DEV_APB_SLV ) {
		size += sizeof(struct ambapp_apb_info);
	} else {
		/* AHB */
		size += sizeof(struct ambapp_ahb_info);
	}
	dev = malloc(size);
	if ( dev == NULL )
		return NULL;
	memset(dev, 0 , size);
	dev->dev_type = dev_type;
	return dev;
}

unsigned int
ambapp_addr_from (struct ambapp_mmap *mmaps, unsigned int address)
{
  /* no translation? */
  if (!mmaps)
    return address;

  while (mmaps->size) {
    if ((address >= mmaps->remote_adr)
        && (address <= (mmaps->remote_adr + (mmaps->size - 1)))) {
      return (address - mmaps->remote_adr) + mmaps->local_adr;
    }
    mmaps++;
  }
  return 1;
}

void ambapp_ahb_dev_init(
	unsigned int ioarea,
	struct ambapp_mmap *mmaps,
	struct ambapp_pnp_ahb *ahb,
	struct ambapp_dev *dev,
	int ahbidx
	)
{
	int bar;
	struct ambapp_ahb_info *ahb_info;
	unsigned int addr, mask, mbar;

	/* Setup device struct */
	dev->vendor = ambapp_pnp_vendor(ahb->id);
	dev->device = ambapp_pnp_device(ahb->id);
	ahb_info = (struct ambapp_ahb_info *)dev->devinfo;
	ahb_info->ver = ambapp_pnp_ver(ahb->id);
	ahb_info->irq = ambapp_pnp_irq(ahb->id);
	ahb_info->ahbidx = ahbidx;
	ahb_info->custom[0] = (unsigned int)ahb->custom[0];
	ahb_info->custom[1] = (unsigned int)ahb->custom[1];
	ahb_info->custom[2] = (unsigned int)ahb->custom[2];

	/* Memory BARs */
	for (bar=0; bar<4; bar++) {
		mbar = ahb->mbar[bar];
		if ( mbar == 0 ) {
			addr = 0;
			mask = 0;
		} else {
			addr = ambapp_pnp_start(mbar);
			if ( ambapp_pnp_mbar_type(mbar) == AMBA_TYPE_AHBIO ) {
				/* AHB I/O area is releative IO_AREA */
				addr = AMBA_TYPE_AHBIO_ADDR(addr, ioarea);
				mask = (((unsigned int)
					(ambapp_pnp_mbar_mask((~mbar))<<8) |
					0xff)) + 1;
			} else {
				/* AHB memory area, absolute address */
				addr = ambapp_addr_from(mmaps, addr);
				mask = (~((unsigned int)
					(ambapp_pnp_mbar_mask(mbar)<<20))) + 1;
			}
		}
		ahb_info->start[bar] = addr;
		ahb_info->mask[bar] = mask;
		ahb_info->type[bar] = ambapp_pnp_mbar_type(mbar);
	}
}

void ambapp_apb_dev_init(
	unsigned int base,
	struct ambapp_mmap *mmaps,
	struct ambapp_pnp_apb *apb,
	struct ambapp_dev *dev,
	int ahbidx
	)
{
	struct ambapp_apb_info *apb_info;

	/* Setup device struct */
	dev->vendor = ambapp_pnp_vendor(apb->id);
	dev->device = ambapp_pnp_device(apb->id);
	apb_info = (struct ambapp_apb_info *)dev->devinfo;
	apb_info->ver = ambapp_pnp_ver(apb->id);
	apb_info->irq = ambapp_pnp_irq(apb->id);
	apb_info->ahbidx = ahbidx;
	apb_info->start = ambapp_pnp_apb_start(apb->iobar, base);
	apb_info->mask = ambapp_pnp_apb_mask(apb->iobar);
}

int ambapp_add_ahbbus(
	struct ambapp_bus *abus,
	unsigned int ioarea
	)
{
	int i;
	for (i=0; i<AHB_BUS_MAX; i++) {
		if ( abus->ahbs[i].ioarea == 0 ) {
			abus->ahbs[i].ioarea = ioarea;
			return i;
		} else if ( abus->ahbs[i].ioarea == ioarea ) {
			/* Bus already added */
			return -1;
		}
	}
	return -1;
}

/* Internal AMBA Scanning Function */
static int ambapp_scan2(
	struct ambapp_bus *abus,
	unsigned int ioarea,
	ambapp_memcpy_t memfunc,
	struct ambapp_dev *parent,
	struct ambapp_dev **root
	)
{
	struct ambapp_pnp_ahb *ahb, ahb_buf;
	struct ambapp_pnp_apb *apb, apb_buf;
	struct ambapp_dev *dev, *prev, *prevapb, *apbdev;
	struct ambapp_ahb_info *ahb_info;
	int maxloops = 64;
	unsigned int apbbase, bridge_adr;
	int i, j, ahbidx;

	*root = NULL;

	if ( parent ) {
		/* scan first bus for 64 devices, rest for 16 devices */
		maxloops = 16;
	}

	ahbidx = ambapp_add_ahbbus(abus, ioarea);
	if ( ahbidx < 0 ) {
		/* Bus already scanned, stop */
		return 0;
	}

	prev = parent;

	/* AHB MASTERS */
	ahb = (struct ambapp_pnp_ahb *) (ioarea | AMBA_CONF_AREA);
	for (i = 0; i < maxloops; i++, ahb++) {
		memfunc(&ahb_buf, ahb, sizeof(struct ambapp_pnp_ahb), abus);
		if ( ahb_buf.id == 0 )
			continue;

		/* An AHB device present here */
		dev = ambapp_alloc_dev_struct(DEV_AHB_MST);
		if ( !dev )
			return -1;

		ambapp_ahb_dev_init(ioarea, abus->mmaps, &ahb_buf, dev, ahbidx);

		if ( *root == NULL) 
			*root = dev;

		if ( prev != parent )
			prev->next = dev;
		dev->prev = prev;
		prev = dev;
	}

	/* AHB SLAVES */
	ahb = (struct ambapp_pnp_ahb *)
		(ioarea | AMBA_CONF_AREA | AMBA_AHB_SLAVE_CONF_AREA);
	for (i = 0; i < maxloops; i++, ahb++) {
		memfunc(&ahb_buf, ahb, sizeof(struct ambapp_pnp_ahb), abus);
		if ( ahb_buf.id == 0 )
			continue;

		/* An AHB device present here */
		dev = ambapp_alloc_dev_struct(DEV_AHB_SLV);
		if ( !dev )
			return -1;

		ambapp_ahb_dev_init(ioarea, abus->mmaps, &ahb_buf, dev, ahbidx);

		if ( *root == NULL)
			*root = dev;

		if ( prev != parent )
			prev->next = dev;
		dev->prev = prev;
		prev = dev;

		ahb_info = (struct ambapp_ahb_info *)dev->devinfo;

		/* Is it a AHB/AHB Bridge ? */
		if ( ((dev->device == GAISLER_AHB2AHB) &&
		     (dev->vendor == VENDOR_GAISLER) && (ahb_info->ver>0)) ||
		     ((dev->device == GAISLER_L2CACHE) &&
		     (dev->vendor == VENDOR_GAISLER)) ||
		     ((dev->device == GAISLER_GRIOMMU) &&
		     (dev->vendor == VENDOR_GAISLER))
		   ) {
			/* AHB/AHB Bridge Found, recurse down the
			 * Bridge
			 */
			if (ahb_info->custom[1] != 0) {
				bridge_adr = ambapp_addr_from(abus->mmaps,
							ahb_info->custom[1]);
				/* Scan next bus if not already scanned */
				if (ambapp_scan2(abus, bridge_adr, memfunc, dev,
						&dev->children) )
					return -1;
			}
		} else if ( (dev->device == GAISLER_APBMST) &&
		            (dev->vendor == VENDOR_GAISLER) ) {
			/* AHB/APB Bridge Found, add the APB devices to this
			 * AHB Slave's children
			 */
			prevapb = dev;
			apbbase = ahb_info->start[0];

			/* APB SLAVES */
			apb = (struct ambapp_pnp_apb *)
				(apbbase | AMBA_CONF_AREA);
			for (j=0; j<AMBA_APB_SLAVES; j++, apb++) {
				memfunc(&apb_buf, apb, sizeof(*apb), abus);
				if ( apb_buf.id == 0 )
					continue;

				apbdev = ambapp_alloc_dev_struct(DEV_APB_SLV);
				if ( !dev )
					return -1;

				ambapp_apb_dev_init(apbbase, abus->mmaps, 								&apb_buf, apbdev, ahbidx);

				if ( prevapb != dev )
					prevapb->next = apbdev;
				else
					dev->children = apbdev;
				apbdev->prev = prevapb;
				prevapb = apbdev;
			}
		}
	}

	/* Remember first AHB MST/SLV device on bus and Parent Bridge */
	abus->ahbs[ahbidx].dev = *root;
	abus->ahbs[ahbidx].bridge = parent;

	return 0;
}

/* Build AMBA Plug & Play device graph 
 * 
 * The root device is stored into '*root'.
 */
int ambapp_scan(
	struct ambapp_bus *abus,
	unsigned int ioarea,
	ambapp_memcpy_t memfunc,
	struct ambapp_mmap *mmaps
	)
{
	memset(abus, 0, sizeof(*abus));
	abus->mmaps = mmaps;

	/* Default to memcpy() */
	if ( !memfunc )
		memfunc = (ambapp_memcpy_t)memcpy;

	return ambapp_scan2(abus, ioarea, memfunc, NULL, &abus->root);
}

       
/* Match search options againt device */
int ambapp_dev_match_options(struct ambapp_dev *dev, unsigned int options, int vendor, int device)
{
	if (	(((options & (OPTIONS_ALL_DEVS)) == OPTIONS_ALL_DEVS) ||		/* Match TYPE */
		 ((options & OPTIONS_AHB_MSTS) && (dev->dev_type == DEV_AHB_MST)) || 
		 ((options & OPTIONS_AHB_SLVS) && (dev->dev_type == DEV_AHB_SLV)) || 
		 ((options & OPTIONS_APB_SLVS) && (dev->dev_type == DEV_APB_SLV))) &&
		((vendor == -1) || (vendor == dev->vendor)) && 				/* Match ID */
		((device == -1)  || (device == dev->device)) &&
		(((options & OPTIONS_ALL) == OPTIONS_ALL) || 				/* Match Allocated State */
		 ((options & OPTIONS_FREE) && DEV_IS_FREE(dev)) || 
		 ((options & OPTIONS_ALLOCATED) && DEV_IS_ALLOCATED(dev)))
		) {
		return 1;
	}
	return 0;
}

/* If device is an APB bridge all devices on the APB bridge is processed */
static int ambapp_for_each_apb(
	struct ambapp_dev *dev,
	unsigned int options,
	int vendor,
	int device,
	int maxdepth,
	ambapp_func_t func,
	void *arg)
{
	int index;
	struct ambapp_dev *apbslv;

	if ( maxdepth < 0 )
		return 0;

	if ( dev->children && (dev->children->dev_type == DEV_APB_SLV) ) {
		/* Found a APB Bridge */
		index = 0;
		apbslv = dev->children;
		while(apbslv){
			if ( ambapp_dev_match_options(apbslv, options,
			                              vendor, device) == 1 ) {
				if ( func(apbslv, index, maxdepth, arg) == 1 )
					return 1; /* Signalled stopped */
			}
			index++;
			apbslv = apbslv->next;
		}
	}
	return 0;
}
			
/* Traverse the prescanned device information */
int ambapp_for_each(
	struct ambapp_dev *root,
	unsigned int options,
	int vendor,
	int device,
	int maxdepth,
	ambapp_func_t func,
	void *arg)
{
	struct ambapp_dev *dev;
	int ahb_slave = 0;
	int index;

	if ( maxdepth < 0 )
		return 0;

	/* Start at device 'root' and process downwards.
	 *
	 * Breadth first search, search order
	 * 1. AHB MSTS
	 * 2. AHB SLVS
	 * 3. APB SLVS on primary bus
	 * 4. AHB/AHB secondary... -> step to 1.
	 */

	/* AHB MST / AHB SLV */
	if (options & (OPTIONS_AHB_MSTS|OPTIONS_AHB_SLVS|OPTIONS_DEPTH_FIRST)) {
		index = 0;
		dev = root;
		while ( dev ) {
			if ( (dev->dev_type == DEV_AHB_SLV) && !ahb_slave) {
				/* First AHB Slave */
				ahb_slave = 1;
				index = 0;
			}

			/* Conditions must be fullfilled for function to be
			 * called 
			 */
			if ( ambapp_dev_match_options(dev, options, vendor,
			                              device) == 1 ) {
				/* Correct device and vendor ID */
				if ( func(dev, index, maxdepth, arg) == 1 )
					return 1; /* Signalled stopped */
			}

			if ( (options & OPTIONS_DEPTH_FIRST) &&
			     (options & OPTIONS_APB_SLVS) ) {
				/* Check is APB bridge, and process all APB
				 * Slaves in that case
				 */
				if ( ambapp_for_each_apb(dev, options, vendor,
				                         device, (maxdepth-1),
				                         func, arg) == 1 )
					return 1; /* Signalled stopped */
			}

			if ( options & OPTIONS_DEPTH_FIRST ) {
				if ( dev->children &&
				     (dev->children->dev_type != DEV_APB_SLV)) {
					/* Found AHB Bridge, recurse */
					if ( ambapp_for_each(dev->children,
					                     options, vendor,
					                     device, 
					                     (maxdepth-1), func,
					                     arg) == 1 )
						return 1;
				}
			}

			index++;
			dev = dev->next;
		}
	}

	/* Find APB Bridges */
	if ((options & OPTIONS_APB_SLVS) && !(options & OPTIONS_DEPTH_FIRST)) {
		dev = root;
		while( dev ) {
			/* Check is APB bridge, and process all APB Slaves in
			 * that case
			 */
			if ( ambapp_for_each_apb(dev, options, vendor, device,
			                         (maxdepth-1), func, arg)==1 )
				return 1; /* Signalled stopped */
			dev = dev->next;
		}
	}

	/* Find AHB Bridges */
	if ( !(options & OPTIONS_DEPTH_FIRST) ) {
		dev = root;
		while( dev ) {
			if ( dev->children &&
			     (dev->children->dev_type != DEV_APB_SLV) ) {
				/* Found AHB Bridge, recurse */
				if ( ambapp_for_each(dev->children, options,
				                     vendor, device,
				                     (maxdepth-1),func,arg)==1 )
					return 1;
			}
			dev = dev->next;
		}
	}

	return 0;
}

int ambapp_alloc_dev(struct ambapp_dev *dev, void *owner)
{
	if ( dev->owner )
		return -1;
	dev->owner = owner;
	return 0;
}

void ambapp_free_dev(struct ambapp_dev *dev)
{
	dev->owner = NULL;
}

struct ambapp_dev *ambapp_find_parent(struct ambapp_dev *dev)
{
	while( dev->prev ) {
		if ( dev == dev->prev->children ) {
			return dev->prev;
		}
		dev = dev->prev;
	}
	return NULL;
}

/* Calculate AHB Bus frequency of
 *   - Bus[0] (inverse=1), relative to the frequency of Bus[ahbidx]
 *     NOTE: set freq_hz to frequency of Bus[ahbidx].
 *   or
 *   - Bus[ahbidx] (inverse=0), relative to the frequency of Bus[0]
 *     NOTE: set freq_hz to frequency of Bus[0].
 *
 * If a unsupported bridge is found the invalid frequncy of 0Hz is
 * returned.
 */
unsigned int ambapp_freq_calc(
	struct ambapp_bus *abus,
	int ahbidx,
	unsigned int freq_hz,
	int inverse)
{
	struct ambapp_ahb_info *ahb;
	struct ambapp_dev *bridge;
	unsigned char ffact;
	int dir;

	/* Found Bus0? */
	bridge = abus->ahbs[ahbidx].bridge;
	if ( !bridge )
		return freq_hz;

	/* Find this bus frequency relative to freq_hz */
	if ( (bridge->vendor == VENDOR_GAISLER) && (
	     (bridge->device == GAISLER_AHB2AHB) ||
	     (bridge->device == GAISLER_L2CACHE)) ) {
		ahb = (struct ambapp_ahb_info *)bridge->devinfo;
		ffact = (ahb->custom[0] & AMBAPP_FLAG_FFACT) >> 4;
		if ( ffact != 0 ) {
			dir = ahb->custom[0] & AMBAPP_FLAG_FFACT_DIR;

			/* Calculate frequency by dividing or
			 * multiplying system frequency
			 */
			if ( (dir && !inverse) || (!dir && inverse) ) {
				freq_hz = freq_hz * ffact;
			} else {
				freq_hz = freq_hz / ffact;
			}
		}
		return ambapp_freq_calc(abus, ahb->ahbidx, freq_hz, inverse);
	} else {
		/* Unknown bridge, impossible to calc frequency */
		return 0;
	}
}

/* Find the frequency of all AHB Buses from knowing the frequency of one
 * particular APB/AHB Device.
 */
void ambapp_freq_init(
	struct ambapp_bus *abus,
	struct ambapp_dev *dev,
	unsigned int freq_hz)
{
	struct ambapp_common_info *info;
	int i;

	for (i=0; i<AHB_BUS_MAX; i++)
		abus->ahbs[i].freq_hz = 0;

	/* Register Frequency at the AHB bus that the device the user gave us
	 * is located at.
	 */
	if ( dev ) {
		info = (struct ambapp_common_info *)dev->devinfo;
		abus->ahbs[info->ahbidx].freq_hz = freq_hz;

		/* Find Frequency of Bus 0 */
		abus->ahbs[0].freq_hz =
			ambapp_freq_calc(abus, info->ahbidx, freq_hz, 1);
	} else {
		abus->ahbs[0].freq_hz = freq_hz;
	}

	/* Find Frequency of all except for Bus0 and the bus which frequency
	 * was reported at
	 */
	for (i=1; i<AHB_BUS_MAX; i++) {
		if ( abus->ahbs[i].ioarea == 0 ) {
			break;
		}
		if ( abus->ahbs[i].freq_hz != 0 ) {
			continue;
		}
		abus->ahbs[i].freq_hz =
			ambapp_freq_calc(abus, i, abus->ahbs[0].freq_hz, 0);
	}
}

/* Assign a AMBA Bus a frequency but reporting the frequency of a
 * particular AHB/APB device */
unsigned int ambapp_freq_get(struct ambapp_bus *abus, struct ambapp_dev *dev)
{
	struct ambapp_common_info *info = 
		(struct ambapp_common_info *)dev->devinfo;
	return abus->ahbs[info->ahbidx].freq_hz;
}
