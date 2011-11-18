/* GRPWRX Packetwire
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

#ifndef __GRPWRX_H__
#define __GRPWRX_H__

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRPWRX_IOC_UNUSED			0

/* Driver operation controlling commands */
#define GRPWRX_IOC_START			1
#define GRPWRX_IOC_STOP			2
#define GRPWRX_IOC_ISSTARTED		3
#define GRPWRX_IOC_SET_BLOCKING_MODE	4
#define GRPWRX_IOC_SET_TIMEOUT		5

/* Available only in RUNNING mode */
#define GRPWRX_IOC_RECV			17

/* Available only in STOPPED mode */
#define GRPWRX_IOC_SET_CONFIG		32

/* Available in both running and stopped mode */
#define GRPWRX_IOC_RECLAIM		64
#define GRPWRX_IOC_GET_CONFIG		65
#define GRPWRX_IOC_GET_HW_IMPL		66
#define GRPWRX_IOC_GET_HW_STATUS		67	/* Not implemented */
#define GRPWRX_IOC_GET_STATS		69
#define GRPWRX_IOC_CLR_STATS		70

#define GRPWRX_IOC_PRINT_STATUS		71


/* Args to GRTC_IOC_SET_BLOCKING_MODE */
enum {
	GRPWRX_BLKMODE_POLL	= 0,	/* Never block (polling mode) */
	GRPWRX_BLKMODE_BLK	= 1,	/* Block until at least 1 byte can be read */
};

struct grpwrx_ioc_hw {
	
	unsigned short	fifo_size;	/* FIFO Size */
	unsigned short  mode;           /* frame mode = 1, packet mode = 0 */
	unsigned short	clkdivide;	/* Clock divide */
	
};

struct grpwrx_print_status {
	unsigned short	printbd;
	
};

/* Argument of GRPWRX_IOC_SET_CONFIG and GRPWRX_IOC_GET_CONFIG.
 * Driver and Hardware configuration.
 *
 * Pointer to:
 */
struct grpwrx_ioc_config {

	int             framing;
	
	/* Physical layer options */
	unsigned short	phy_clkrise;	
	unsigned short	phy_validpos;	
	unsigned short	phy_readypos;	
	unsigned short	phy_busypos;	

	/* Interrupt options */
	unsigned int	enable_cnt;	/* Number of frames in between Interrupt is generated, Zero disables interrupt */
	int		isr_desc_proc;	/* Enable ISR to process descriptors */
	int		blocking;	/* Blocking mode select (POLL,BLK..) */
	rtems_interval	timeout;	/* Blocking mode timeout */
};

struct grpwrx_packet;

struct grpwrx_list {
	struct grpwrx_packet *head;	/* First Frame in list */
	struct grpwrx_packet *tail;	/* Last Frame in list */
};

#define GRPWRX_FLAGS_RECEIVED		0x01
#define GRRM_FLAGS_ERR		0x02

#define GRPWRX_FLAGS_TRANSLATE	(1<<31)	/* Translate frame payload address from CPU address to remote bus (the bus GRPWRX is resident on) */
#define GRPWRX_FLAGS_TRANSLATE_AND_REMEMBER	(1<<30) /* As GRPWRX_FLAGS_TRANSLATE, however if the translated payload address equals the payload address
							 * the GRPWRX_FLAGS_TRANSLATE_AND_REMEMBER bit is cleared and the GRPWRX_FLAGS_TRANSLATE bit is set */
#define GRPWRX_FLAGS_COPY_DATA	(1<<29)	/* Where available: Transfer Frame payload to target, may be used for SpaceWire, where the GRPWRX driver transfer 
					 * the payload to a buffer on the SpaceWire target. 
					 */

#define GRPWRX_FLAGS_FHP		(1<<3)

#define GRPWRX_FLAGS_MASK		(GRPWRX_FLAGS_FHP)

/* The GRPWRX software representation of a Frame */
struct grpwrx_packet {
	/* Options and status */
	unsigned int		flags;		/* bypass options, and sent/error status */
	struct grpwrx_packet	*next;		/* Next packet in chain */
	int length;
	unsigned char		*payload;	/* The Headers and Payload,  Frame data and header must be word aligned */
};

#define FRAME_SIZE(payloadlen)	(sizeof(struct grpwrx_packet)+payloadlen)

struct grpwrx_ioc_stats {
	unsigned long long	packets_received;
	unsigned int		err_underrun;
	unsigned int		err_tx;
	unsigned int		err_ahb;
	unsigned int		err_transfer_frame;
};

/* Register GRPWRX driver at driver manager */
void grpwrx_register_drv(void);

#ifdef __cplusplus
}
#endif

#endif /* __GRPWRX_H__ */
