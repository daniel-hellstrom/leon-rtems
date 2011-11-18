/* RMAP stack and RMAP driver interface
 *
 *  COPYRIGHT (c) 2009
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __RMAP_H__
#define __RMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*** RMAP Driver interface ***/

#define PKT_OPTION_HDR_CRC	0x1 /* Enable Header CRC by Hardware */
#define PKT_OPTION_DATA_CRC	0x2 /* Enable Data CRC by Hardware */
/* skip len of header when calculating CRC by hardware */
#define PKT_OPTION_HDR_CRC_SKIPLEN(len)	((len & 0xf) << 8)
#define PKT_OPTION_HDR_CRC_SKIPLEN_MASK 0xf00

/* RMAP SpW packet */
struct rmap_spw_pkt {
	int		options;	/* Data CRC | Header CRC */
	int		hlen;
	unsigned char	*hdr;
	int		dlen;
	unsigned char	*data;
	int		reserved[2];	/* May be used internally by driver layer */
};

struct rmap_drv_ops {
	int	(*init)(void *cookie);						/* Initialize driver */
	int	(*ioctl)(void *cookie, int command, void *arg);			/* Configure driver */
	int	(*send)(void *cookie, struct rmap_spw_pkt *pkt);		/* Schedule one packet for transmission */
	int	(*recv)(void *cookie, struct rmap_spw_pkt *pkt);		/* Receive one packet */
};

/* RMAP driver description */
struct rmap_drv {
	struct rmap_drv_ops	ops;
	void			*cookie;
};

#define RMAP_TIMEOUT_SET_RTIME 0x01
#define RMAP_TIMEOUT_SET_WTIME 0x02
struct rmap_drv_timeout {
	unsigned int options;
	unsigned int rtimeout;
	unsigned int wtimeout;
};

/* RMAP driver ioctl commands */
#define RMAP_DRV_IOCTL_START		1	/* Start operation, packets will be sent/received after this command */
#define RMAP_DRV_IOCTL_STOP		2	/* Stop operation, no more packets will be sent before next START */
#define RMAP_DRV_IOCTL_BLOCK		3	/* Set RX/TX blocking */
#define RMAP_DRV_IOCTL_TIMEOUT		4	/* Set timeout in blocing mode */
#define RMAP_DRV_IOCTL_GET_CAP		5	/* Get capabilities of the driver/hardware */
#define RMAP_IOCTL_GET_CONFIG		6	/* Get Stack Configuration */

/* Capabilities */
#define DRV_CAP_HDR_CRC			0x1	/* Hardware supports HEADER CRC generation */
#define DRV_CAP_DATA_CRC		0x2	/* Hardware supports DATA CRC generation */


/*** RMAP Stack interface ***/

/* Function that will generate path addressing.
 *
 * \param cookie   Identifies RMAP stack, 
 * \param dir      Direction (0=to dst, 1=from dst)
 * \param srcadr   Source address (the SpW address of the interface used)
 * \param dstadr   Destination address to translate. Note that it is actually
 *                 a "unsigned short" given here, so this is rather a Node-ID.
 * \param buf      Where path address is put
 * \param len      On calling it indicates max length. On return it must be set
 *                 reflect the number of bytes was written to buf.
 */
typedef int (*rmap_route_t)(void *cookie, int dir, int srcadr, int dstadr, void *buf, int *len);

struct rmap_config {
	rmap_route_t	route_func;	/* Function that will generate path addressing */
	int		tid_msb;	/* 8 most significant bits in TID.
					 * Set to -1 for normal operation using
					 * all bits in TID for sequence counting.
					 */
	int		spw_adr;	/* The SpW Address of the SpW interface used */
	struct rmap_drv	*drv;		/* Driver used for transmission */
	int		max_rx_len;	/* Maximum data length of received packets */
	int		max_tx_len;	/* Maximum data length of transmitted packets */
	int		thread_safe;	/* Set this to non-zero to enable the RMAP stack to create a
					 * semaphore used to protect from multiple tasks entering the
					 * transfer function(s) of the stack at the same time */
};

struct rmap_command {
	char			type;		/* READ/WRITE/MODIFY ... */
	unsigned char		dstkey;		/* Destination key */
	unsigned char		status;		/* Error/Status response. 0 if no response is expected */
	unsigned short		dstadr;		/* Destination address */
	unsigned short		tid;		/* TID assigned to packet */
	unsigned long long	address;	/* READ/WRITE address, 32-bit + 8-bit extended */
	union {
		struct {
			unsigned int length;	/* 24-bit data length */
			unsigned char *data;	/* Data bytes, NOTE that 1 byte must be available
						 * for the stack to write the DATA CRC in the end
						 * of the data buffer. Stack write data[length] = CRC
						 */
		} write;
		struct {
			unsigned int length;	/* 24-bit length */
			unsigned int datalength;/* Response 24-bit length, may be different from requested length */
			unsigned int *data;	/* Response data is stored in this buffer */
		} read;
		struct {
			unsigned int length;	/* length 1,2 or 4 valid */
			unsigned int data;	/* 1,2 or 4 bytes data */
			unsigned int mask;	/* 1,2 or 4 bytes data mask */
			unsigned int oldlength;	/* Response, old data length */
			unsigned int olddata;	/* Response, old data before write */
		} read_m_write;
	} data;
};

struct rmap_command_write {
	char			type;		/* WRITE (Increment, Single, ACK, Verify) */
	unsigned char		dstkey;		/* Destination key */
	unsigned char		status;		/* Status response, set if ACK, non-zero indicates error */
	unsigned short		dstadr;		/* Destination address */
	unsigned short		tid;		/* TID assigned to packet */
	unsigned long long	address;	/* READ/WRITE address */
	unsigned int 		length;		/* 24-bit data length */
	unsigned char		*data;		/* Data bytes, NOTE that 1 byte must be available 
						 * for the stack to write the DATA CRC in the end 
						 * of the data buffer. Stack write data[length] = CRC
						 */
};

struct rmap_command_read {
	char			type;		/* READ (Single, Increment) */
	unsigned char		dstkey;		/* Destination key */
	unsigned char		status;		/* Status response, always set, non-zero indicates error */
	unsigned short		dstadr;		/* Destination address */
	unsigned short		tid;		/* TID assigned to packet */
	unsigned long long	address;	/* READ/WRITE address */
	unsigned int 		length;		/* 24-bit data length */
	unsigned int		datalength;	/* Response 24-bit length, may be different from requested length */
	unsigned int		*data;		/* Response data is stored in this buffer */
};

struct rmap_command_rmw {
	char			type;		/* RMAP_CMD_RMWI */
	unsigned char		dstkey;		/* Destination key */
	unsigned char		status;		/* Status response, always set, non-zero indicates error */
	unsigned short		dstadr;		/* Destination address */
	unsigned short		tid;		/* TID assigned to packet */
	unsigned long long	address;	/* READ/WRITE address */
	unsigned int		length;		/* length 1,2 or 4 valid */
	unsigned int		data;		/* 1,2 or 4 bytes data */
	unsigned int		mask;		/* 1,2 or 4 bytes data mask */
	unsigned int		oldlength;	/* Response, old data length */
	unsigned int		olddata;	/* Response, old data before write */
};

/* Command definition.
 *
 * R = READ
 * W = WRITE
 * M = MODIFY
 * S = SINGLE
 * I = INCREMENTING ADDRESSES
 * V = VERIFY
 * A = ACKNOWLEDGE
 */
enum {
	RMAP_CMD_UNUSED1 	= 0x0,	/* Not used */
	RMAP_CMD_UNUSED2 	= 0x1,	/* Not used */
	RMAP_CMD_RS		= 0x2,	/* Read single address */
	RMAP_CMD_RI		= 0x3,	/* Read incrementing addresses */
	RMAP_CMD_UNUSED3	= 0x4,	/* Not used */
	RMAP_CMD_UNUSED4	= 0x5,	/* Not used */
	RMAP_CMD_UNUSED5	= 0x6,	/* Not used */
	RMAP_CMD_RMWI		= 0x7,	/* Read-Modify-Write incrementing addresses */
	RMAP_CMD_WS		= 0x8,	/* Write, single address, don't verify before writing, no acknowledge */
	RMAP_CMD_WI		= 0x9,	/* Write, incrementing addresses, don't verify before writing, no acknowledge */
	RMAP_CMD_WSA		= 0xa,	/* Write, single address, don't verify before writing, send acknowledge */
	RMAP_CMD_WIA		= 0xb,	/* Write, incrementing addresses, don't verify before writing, send acknowledge */
	RMAP_CMD_WSV		= 0xc,	/* Write, single address, verify before writing, no acknowledge */
	RMAP_CMD_WIV		= 0xd,	/* Write, incrementing addresses, verify before writing, no acknowledge */
	RMAP_CMD_WSVA		= 0xe,	/* Write, single address, verify before writing, send acknowledge */
	RMAP_CMD_WIVA		= 0xf,	/* Write, incrementing addresses, verify before writing, send acknowledge */
};

#define RMAP_CMD_INCREMENT	0x1
#define RMAP_CMD_ACKNOWLEDGE	0x2
#define RMAP_CMD_VERIFY		0x4
#define RMAP_CMD_WRITE		0x8

/* Initialize the stack 
 *
 * \param route_func   Function that will generate path addressing
 * \param drv		Set driver used for transmission
 */
void *rmap_init(struct rmap_config *config);

/* Enables the RMAP stack to send/receive commands */

/* Disables the RMAP stack to send/receive commands */

/* Configure stack (blocking etc)*/
extern int rmap_ioctl(void *cookie, int command, void *arg);

/* Send a command and optionally block waiting for response.
 *
 * The transaction ID is incremented 
 *
 * \param cmd   RMAP command to transmit
 * \param resp  Response (stack blocking mode only)
 * \param tid   Pointer to where the Transaction ID is stored (stack non-blocking mode only)
 */
extern int rmap_send(void *cookie, struct rmap_command *cmd);

#if NOT_IMPLEMENTED
/** OPTIONAL PART 1 **/

/* Wait for response on a previously sent RMAP command, the command was sent
 * using rmap_send and was assigned a transaction ID (tid). The TID is used'
 * to separate different RMAP commands from each other.
 *
 * This is a blocking call.
 *
 * Only available when,
 *  - stack in non-blocking mode
 *  - driver in blocking mode.
 *
 * \param timeout  Timeout in ticks. Currently NOT used.
 * \param resp     Response on the Transaction ID
 * \param tid      Transaction ID
 */
extern int rmap_resp_wait(void *cookie, int timeout, struct rmap_command *resp, unsigned int tid);

/** OPTIONAL PART 2 **/

/* Check for response on a previously sent RMAP command, the command was sent
 * using rmap_send and was assigned a transaction ID (tid). The TID is used'
 * to separate different RMAP commands from each other.
 *
 * This function will never block.
 *
 * Only available when,
 *  - stack in non-blocking mode
 *  - driver in non-blocking mode.
 *
 * \param resp  Response on the Transaction ID
 * \param tid   Transaction ID
 */
extern int rmap_resp_poll(void *cookie, struct rmap_command *resp, unsigned int tid);
#endif

/* rmap_ioctl commands */
#define RMAP_IOCTL_START	0	/* Start RMAP stack, enables stack to send/receive packets */
#define RMAP_IOCTL_STOP		1	/* Stop RMAP stack, no packets will be sent/received */
#define RMAP_IOCTL_BLOCK	2	/* Set blocking mode */
#define RMAP_IOCTL_TIMEOUT	3	/* Set timeout for command responses in number of ticks */

#define RMAP_IOCTRL_BLOCK_ALL	0


/*** RMAP Stack help interface ***/

/* Parse a Write command */

/* Return the CRC of a RMAP data buffer of length len */
extern unsigned char rmap_crc_calc(unsigned char *data, unsigned int len);

/* A handy write function using the RMAP stack */
extern int rmap_write(void *cookie, void *dst, void *buf, int length, int dstadr, int dstkey);

/* A handy read function using the RMAP stack */
extern int rmap_read(void *cookie, void *src, void *buf, int length, int dstadr, int dstkey);

#ifdef __cplusplus
}
#endif

#endif
