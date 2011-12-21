/* GRAES Packetwire
 * 
 * --------------------------------------------------------------------------
 *  --  This file is a part of GAISLER RESEARCH source code.
 *  --  Copyright (C) 2009, Gaisler Research AB - all rights reserved.
 *  --
 *  -- ANY USE OR REDISTRIBUTION IN PART OR IN WHOLE MUST BE HANDLED IN
 *  -- ACCORDANCE WITH THE GAISLER LICENSE AGREEMENT AND MUST BE APPROVED
 *  -- IN ADVANCE IN WRITING.
 *  --
 *  -- BY DEFAULT, DISTRIBUTION OR DISCLOSURE IS NOT PERMITTED.
 *  -------------------------------------------------------------------------- 
 *
 */

#ifndef __GRAES_H__
#define __GRAES_H__

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAES_IOC_UNUSED                0

/* Driver operation controlling commands */
#define GRAES_IOC_START			1
#define GRAES_IOC_STOP			2
#define GRAES_IOC_ISSTARTED		3
#define GRAES_IOC_SET_BLOCKING_MODE	4
#define GRAES_IOC_SET_TIMEOUT		5

/* Available only in RUNNING mode */
#define GRAES_IOC_ENCRYPT	        17

/* Available only in STOPPED mode */
#define GRAES_IOC_SET_CONFIG		32

/* Available in both running and stopped mode */
#define GRAES_IOC_RECLAIM		64
#define GRAES_IOC_GET_CONFIG		65
#define GRAES_IOC_GET_HW_IMPL		66
#define GRAES_IOC_GET_HW_STATUS		67	/* Not implemented */
#define GRAES_IOC_GET_STATS		69
#define GRAES_IOC_CLR_STATS		70

#define GRAES_IOC_PRINT_STATUS		71

	

/* Args to GRTC_IOC_SET_BLOCKING_MODE */
enum {
	GRAES_BLKMODE_POLL	= 0,	/* Never block (polling mode) */
	GRAES_BLKMODE_BLK	= 1,	/* Block until at least 1 byte can be read */
};

struct graes_ioc_hw {
	unsigned short	key_size;	/* KEY Size */
};

struct graes_print_status {
	unsigned short	printbd;
};


/* Argument of GRAES_IOC_SET_CONFIG and GRAES_IOC_GET_CONFIG.
 * Driver and Hardware configuration.
 *
 * Pointer to:
 */
struct graes_ioc_config {

	unsigned short	key_size;	

	/* Interrupt options */
	unsigned int	enable_cnt;	/* Number of frames in between Interrupt is generated, Zero disables interrupt */
	int		isr_desc_proc;	/* Enable ISR to process descriptors */
	int		blocking;	/* Blocking mode select (POLL,BLK..) */
	rtems_interval	timeout;	/* Blocking mode timeout */
};

struct graes_block;

struct graes_list {
	struct graes_block *head;	/* First Frame in list */
	struct graes_block *tail;	/* Last Frame in list */
};

#define GRAES_FLAGS_PROCESSED		0x01
#define GRRM_FLAGS_ERR		0x02

#define GRAES_FLAGS_TRANSLATE	(1<<31)	/* Translate block payload address from CPU address to remote bus (the bus GRAES is resident on) */
#define GRAES_FLAGS_TRANSLATE_AND_REMEMBER	(1<<30) /* As GRAES_FLAGS_TRANSLATE, however if the translated payload address equals the payload address
							 * the GRAES_FLAGS_TRANSLATE_AND_REMEMBER bit is cleared and the GRAES_FLAGS_TRANSLATE bit is set */
#define GRAES_FLAGS_COPY_DATA	(1<<29)	/* Where available: Transfer Frame payload to target, may be used for SpaceWire, where the GRAES driver transfer 
					 * the payload to a buffer on the SpaceWire target. 
					 */

#define GRAES_FLAGS_MASK        (GRAES_BD_ED)

#define GRAES_BD_ED_BIT         4

#define GRAES_BD_ED		(1<<GRAES_BD_ED_BIT)

	
/* The GRAES software representation of a Frame */
struct graes_block {
	/* Options and status */
	unsigned int		flags;		/* bypass options, and sent/error status */
	struct graes_block	*next;		/* Next packet in chain */
	int                     length;
	unsigned char		*key;
	unsigned char		*iv;
	unsigned char		*payload;	/* in */
	unsigned char		*out;	        /* out */
};

#define FRAME_SIZE(payloadlen)	(sizeof(struct graes_block)+payloadlen)

struct graes_ioc_stats {
	unsigned long long	blocks_processed;
	unsigned int		err_underrun;
	unsigned int		err_tx;
	unsigned int		err_ahb;
	unsigned int		err_transfer_frame;
};

/* Register GRAES driver at driver manager */
void graes_register_drv(void);

#ifdef __cplusplus
}
#endif

#endif /* __GRAES_H__ */
