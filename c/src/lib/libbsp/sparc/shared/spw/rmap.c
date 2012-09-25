/*  RMAP stack and RMAP driver interface implementation
 *
 *  COPYRIGHT (c) 2009
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2009-11-17, Daniel Hellstrom <daniel@gaisler.com>
 *    Created
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include "rmap.h"

#ifdef DEBUG
 #define DBG(args...) printf(args);
#else
 #define DBG(args...)
#endif

static int rmap_stack_count = 0;

static unsigned char RMAP_CRCTable[256] = {
    0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75,
    0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
    0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69,
    0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
    0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d,
    0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
    0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51,
    0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
    0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05,
    0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
    0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19,
    0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
    0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d,
    0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
    0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21,
    0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
    0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95,
    0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
    0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89,
    0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
    0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad,
    0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
    0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1,
    0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
    0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5,
    0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
    0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9,
    0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
    0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd,
    0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
    0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1,
    0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf,
};

unsigned char rmap_crc_calc(unsigned char *data, unsigned int len)
{
	unsigned char crc = 0;
	unsigned int i;
	for (i = 0; i < len; i++) {
		crc = RMAP_CRCTable[(crc ^ data[i]) & 0xff];
	}
	return crc;
}

struct rmap_priv {
	struct rmap_config	*config;		/* Configuration */
	void			*drv_cookie;		/* Driver private structure */
	struct rmap_drv_timeout timeout;		/* Driver timeout */
	unsigned short		tid;			/* Current TID */
	char			drv_cap;		/* Driver capabiliteis */
	char			running;		/* 1 is started, 0 if stopped */
	unsigned char		blocking;		/* Blocking mode */
	unsigned char		tx_pkt_hdr[256+9];	/* Packet header used for transmission, last 9 bytes is for RMW commands */
	unsigned int		_rx_pkt_buf;		/* RX packet Buffer */
	unsigned int		*rx_pkt_buf;		/* RX packet Buffer aligned to 64b*/
	unsigned int		rx_pkt_buf_len;		/* RX packet Buffer length */
	struct rmap_spw_pkt	rxpkt;			/* RX packet */
	struct rmap_spw_pkt	txpkt;			/* TX packet */
	rtems_id		lock;			/* Optional Semaphore protection against multiple threads (thread-safe) */
};

void *rmap_init(struct rmap_config *config)
{
	struct rmap_priv *priv;
	int status;

	priv = (struct rmap_priv *)malloc(sizeof(struct rmap_priv));
	if ( !priv )
		return NULL;

	memset(priv, 0, sizeof(struct rmap_priv));
	priv->config = config;
	if ( config->tid_msb >= 0 ) {
		priv->tid = config->tid_msb << 8;
	}
	priv->drv_cookie = priv->config->drv->cookie;

	/* Create Semaphore used to make RMAP layer Thread-safe */
	if ( config->thread_safe ) {
		status = rtems_semaphore_create(
			rtems_build_name('R', 'M', 'A', 'P' + rmap_stack_count++),
			1,
			RTEMS_FIFO | RTEMS_COUNTING_SEMAPHORE | RTEMS_NO_INHERIT_PRIORITY | \
			RTEMS_LOCAL | RTEMS_NO_PRIORITY_CEILING,
			0,
			&priv->lock);
		if ( status != RTEMS_SUCCESSFUL ) {
			printf("RMAP_INIT: Failed to create semaphore: %d\n", status);
			free(priv);
			return NULL;
		}
		DBG("RMAP_INIT: semaphore ID: 0x%x (%p)\n", priv->lock, &priv->lock);
	}

	return priv;
}

void rmap_init_rxpkt(struct rmap_priv *priv, struct rmap_spw_pkt *pkt)
{
	pkt->data = (unsigned char *)priv->rx_pkt_buf;
	pkt->dlen = priv->rx_pkt_buf_len;
	pkt->hlen = 0;
	pkt->hdr = 0;
	pkt->options = 0;
}


int rmap_start(struct rmap_priv *priv)
{
	if ( priv->running == 0 ) {
		if ( !priv->_rx_pkt_buf ) {
			/* Header length + Data CRC + 4 extra */
			priv->rx_pkt_buf_len = priv->config->max_rx_len + 16 + 1 + 4;
			priv->_rx_pkt_buf = (unsigned int)malloc(priv->rx_pkt_buf_len+4);
			priv->rx_pkt_buf = (unsigned int*)((priv->_rx_pkt_buf+7)&~7);
			if ( priv->rx_pkt_buf == NULL )
				return -1;

		}

		/* Set blocking mode */
		if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_BLOCK, (void *)(int)priv->blocking) ){
			/* Failed to set blocking mode */
			return -1;
		}

		/* Set timeout, defualt is zero = Wait forever */
		if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_TIMEOUT, (void *)&priv->timeout) ){
			/* Failed to set timeout */
			return -1;
		}

		if ( priv->drv_cap == 0 ) {
			unsigned int capabilities;
			if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_GET_CAP, &capabilities) ){
				/* Failed to get capabilities, assume none */
				priv->drv_cap = 0;
			} else {
				priv->drv_cap = capabilities & 0xff;
			}
		}

		if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_START, 0) ){
			/* Failed to start */
			return -1;
		}

	}
	priv->running = 1;

	return 0;
}


/* Disables the RMAP stack to send/receive commands */
int rmap_stop(struct rmap_priv *priv)
{
	if ( priv->running == 0 )
		return 0;

	priv->running = 0;
	if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_STOP, 0) ){
		/* Failed to start */
		return -1;
	}

	return 0;
}

/* Set operating mode */
int rmap_set_blocking(struct rmap_priv *priv, unsigned int mode)
{
	if ( priv->running )
		return -1;

	if ( mode != RMAP_IOCTRL_BLOCK_ALL ) {
		return -1;
	}

	priv->blocking = mode;

	return 0;
}

/* Set Timeout */
int rmap_set_timeout(struct rmap_priv *priv, struct rmap_drv_timeout *timeout)
{
	priv->timeout = *timeout;
	if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_TIMEOUT, timeout) ){
		/* Failed to set timeout */
		return -1;
	}
	return 0;
}

int rmap_get_config(struct rmap_priv *priv, struct rmap_config *result)
{
	if ( !result || !priv )
		return -1;
	*result = *priv->config;
	return 0;
}

/* Configure stack (blocking etc)*/
int rmap_ioctl(void *cookie, int command, void *arg)
{
	switch (command) {
		case RMAP_IOCTL_START:
			return rmap_start((struct rmap_priv *)cookie);
		case RMAP_IOCTL_STOP:
			return rmap_stop((struct rmap_priv *)cookie);
		case RMAP_IOCTL_BLOCK:
			return rmap_set_blocking((struct rmap_priv *)cookie, (unsigned int)arg);
		case RMAP_IOCTL_TIMEOUT:
			return rmap_set_timeout((struct rmap_priv *)cookie, (struct rmap_drv_timeout *)arg);
		case RMAP_IOCTL_GET_CONFIG:
			return rmap_get_config((struct rmap_priv *)cookie, (struct rmap_config *)arg);
		default:
			return -1;
	}
}

int rmap_build(struct rmap_priv *priv, struct rmap_command *cmd, struct rmap_spw_pkt *txpkt)
{
	int len1, len2;
	unsigned char *pos, *pkt = (unsigned char *)txpkt->hdr;
	unsigned int length, src_path_len;
	int options;

	/* Begin with address translation */
	if ( priv->config->route_func ) {
		/* Custom translation */

		/* Generate path addressing to destination */
		len1 = 120;
		priv->config->route_func(priv, 0, priv->config->spw_adr, cmd->dstadr, pkt, &len1);

		/* Generate path addressing from destination */
		len2 = 13;
		priv->config->route_func(priv, 1, priv->config->spw_adr, cmd->dstadr, &pkt[len1+3], &len2);

	} else {
		pkt[0] = cmd->dstadr & 0xff;
		pkt[4] = priv->config->spw_adr;
		len1 = 1;
		len2 = 1;
	}
	/* Protocol Identifier */
	if ( len2 == 1 ) {
		src_path_len = 0;
	} else {
		src_path_len = ((len2-1)+3) / 4;
	}
	pos = &pkt[len1];
	*pos++ = 0x01;
	*pos++ = 0x40 | (cmd->type << 2) | src_path_len;
	*pos++ = cmd->dstkey;
	pos += len2; /* Jump over address,  len1 + 3 + len2 */
	*pos++ = (priv->tid >> 8);
	*pos++ = priv->tid & 0xff;
	cmd->tid = priv->tid; /* Remember TID */
	/* Increment TID */
	if ( priv->config->tid_msb < 0 ) {
		priv->tid++;
	} else {
		if ( (priv->tid & 0xff) == 0xff) {
			priv->tid = priv->tid & 0xff00;
		} else {
			priv->tid++;
		}
	}

	/* Address */
	*pos++ = (cmd->address >> 32) & 0xff;	/* Extended Address */
	*pos++ = (cmd->address >> 24) & 0xff;
	*pos++ = (cmd->address >> 16) & 0xff;
	*pos++ = (cmd->address >> 8) & 0xff;
	*pos++ = cmd->address & 0xff;
	/* pos = pkt[len1+3+len2+7] */
	if ( cmd->type & RMAP_CMD_WRITE ) {		/* WRITE */
		length = cmd->data.write.length;
	} else if ( cmd->type == RMAP_CMD_RMWI ) {	/* READ-MODIFY_WRITE */
		length = cmd->data.read_m_write.length * 2;
	} else {					/* READ */
		length = cmd->data.read.length;
	}
	*pos++ = (length >> 16) & 0xff;
	*pos++ = (length >> 8) & 0xff;
	*pos++ = length & 0xff;
	txpkt->hlen = pos - pkt; /* HDR CRC is fixed later */

	if ( cmd->type & RMAP_CMD_WRITE ) {		/* WRITE */
		txpkt->dlen = length;
		if ( cmd->data.write.data > 0 ) {
			txpkt->data = cmd->data.write.data; /* CRC will be added later */
		} else {
			txpkt->data = ((char *)&txpkt->dlen + 1); /* Temporary storage of CRC when no data */
		}
	} else if ( cmd->type == RMAP_CMD_RMWI ) {	/* READ-MODIFY_WRITE */
		txpkt->dlen = length; /* CRC will be added later */
		txpkt->data = &txpkt->hdr[256];
		if ( cmd->data.read_m_write.length == 4 ) {
			/* 4byte */
			txpkt->hdr[256] = (cmd->data.read_m_write.data >> 24) & 0xff;
			txpkt->hdr[257] = (cmd->data.read_m_write.data >> 16) & 0xff;
			txpkt->hdr[258] = (cmd->data.read_m_write.data >> 8) & 0xff;
			txpkt->hdr[259] = cmd->data.read_m_write.data & 0xff;
			
			txpkt->hdr[260] = (cmd->data.read_m_write.mask >> 24) & 0xff;
			txpkt->hdr[261] = (cmd->data.read_m_write.mask >> 16) & 0xff;
			txpkt->hdr[262] = (cmd->data.read_m_write.mask >> 8) & 0xff;
			txpkt->hdr[263] = cmd->data.read_m_write.mask & 0xff;
		} else if ( cmd->data.read_m_write.length == 1 ) {
			/* 1byte */
			txpkt->hdr[256] = cmd->data.read_m_write.data & 0xff;
			txpkt->hdr[257] = cmd->data.read_m_write.mask & 0xff;
		} else if ( cmd->data.read_m_write.length == 2 ) {
			/* 2byte */
			txpkt->hdr[256] = (cmd->data.read_m_write.data >> 8) & 0xff;
			txpkt->hdr[257] = cmd->data.read_m_write.data & 0xff;
			
			txpkt->hdr[258] = (cmd->data.read_m_write.mask >> 8) & 0xff;
			txpkt->hdr[259] = cmd->data.read_m_write.mask & 0xff;
		} else {
			/* Wrong input */
			return -1;
		}
	} else {					/* READ */
		txpkt->dlen = 0;
		txpkt->data = NULL;
	}

	/*** CRC ***/
	if ( len1 > 1 ) /* PATH target address not part of Header CRC */
		options = PKT_OPTION_HDR_CRC_SKIPLEN(len1-1);
	else
		options = 0;

	/* HEADER CRC */
	if ( priv->drv_cap & DRV_CAP_HDR_CRC ) {
		options |= PKT_OPTION_HDR_CRC;
	} else {
		/* Generate CRC and put it at the end of data buffer, Don't calculate
		 * CRC on Destination Path address. 
		 */
		unsigned char crc;
		crc = rmap_crc_calc(&txpkt->hdr[len1-1], txpkt->hlen - (len1-1));
		((unsigned char *)txpkt->hdr)[txpkt->hlen] = crc;
		txpkt->hlen++;
	}
	/* DATA CRC */
	if ( txpkt->data ) {
		if ( txpkt->dlen == 0 ) {
			/* Need to put CRC even when no data available,
			 * CRC will always be 0x00 
			 */
			((unsigned char *)txpkt->data)[0] = 0x00;
			txpkt->dlen = 1;
		} else if ( priv->drv_cap & DRV_CAP_DATA_CRC ) {
			options |= PKT_OPTION_DATA_CRC;
		} else {
			/* Generate CRC and put it at the end of data buffer */
			unsigned char crc;
			crc = rmap_crc_calc(txpkt->data, txpkt->dlen);
			((unsigned char *)txpkt->data)[txpkt->dlen] = crc;
			txpkt->dlen++;
		}
	}

	txpkt->options = options;

	return 0;
}

int rmap_parse(struct rmap_priv *priv, struct rmap_command *cmd, struct rmap_spw_pkt *rxpkt)
{
	unsigned char *pkt;
	unsigned int length;

	pkt = rxpkt->data;

	/* Check that the protocol ID, Replay and TID matches before proceeding */
	if (	(pkt[1] != 0x01) || ((pkt[2] & 0x40) != 0) || 
		(pkt[5] != (cmd->tid>>8)) || (pkt[6] != (cmd->tid & 0xff))) {
		return -1;
	}

	cmd->status = pkt[3];
	if ( cmd->type & RMAP_CMD_WRITE ){
		return 0;
	}

	if ( cmd->type == RMAP_CMD_RMWI ) {
		/* Copy data content */
		cmd->data.read_m_write.oldlength = length = pkt[10];
		if ( length == 4 ) {
			cmd->data.read_m_write.olddata = ntohl(*(unsigned int *)&pkt[12]);
		} else if ( length == 1 ){
			cmd->data.read_m_write.olddata = pkt[12];
		} else if ( length == 2 ) {
			cmd->data.read_m_write.olddata = ntohs(*(unsigned short *)&pkt[12]);
		} else {
			cmd->data.read_m_write.olddata = 0;
		}
		return 0;
	}

	/* READ COMMAND RESPONSE - copy data */
	if ( cmd->status != 0 ) {
		/* Error status, skip other bytes */
		return 0;
	}

	/* Length may be equal or less than request nembuer of bytes */
	cmd->data.read.datalength = length = (pkt[8]<<16) | (pkt[9]<<8) | pkt[10];
	if ( length > cmd->data.read.length ) {
		printf("Response DATA Length != Command Data Length [%p, %p]\n",
			cmd, pkt);
		return -1;
	}
	if ( length > 0 ) {
		memcpy(cmd->data.read.data, &pkt[12], length);
	}

	return 0;
}

int rmap_send(void *cookie, struct rmap_command *cmd)
{
	struct rmap_priv *priv = cookie;
	struct rmap_drv *drv;
	int wait_response;
	int ret=0, status;
	struct rmap_spw_pkt *txpkt, *rxpkt;

	if ( !priv || !cmd || !priv->running )
		return -1;

	/*** Check invalid command type ***/
	if (	(cmd->type > RMAP_CMD_WIVA) ||
		((cmd->type < RMAP_CMD_RMWI) && (cmd->type != RMAP_CMD_RS) && (cmd->type != RMAP_CMD_RI))
	   )
		return -1;
	if (	(cmd->type == RMAP_CMD_RMWI) && (cmd->data.read_m_write.length != 1) && 
		(cmd->data.read_m_write.length != 2) && (cmd->data.read_m_write.length != 4)
	   )
		return -1;

	/*** Determine if stack must wait for response to arrive after transmission ***/
	wait_response = 0;
	if (	(cmd->type & RMAP_CMD_WRITE == 0) ||	/* READ ALWAYS RETURNS DATA */
		(cmd->type & RMAP_CMD_ACKNOWLEDGE)	/* WAIT FOR ACK */
	   )
		wait_response = 1;

	/*** Check that READ/WRITE length is within boundaries ***/
	if ( (cmd->type & RMAP_CMD_WRITE) && (cmd->data.write.length > (priv->config->max_tx_len-4)) ) {
		return -1;
	}
	if ( ((cmd->type & RMAP_CMD_WRITE) == 0) && (cmd->data.read.length > (priv->config->max_rx_len)) ) {
		return -1;
	}

	if ( priv->lock ) {
		status = rtems_semaphore_obtain(priv->lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		if ( status != RTEMS_SUCCESSFUL ) {
			printf("RMAP-STACK: FAILED TO OBTAIN LOCK: %d (%p)\n",
				status, priv->lock);
			return -1;
		}
	}

	/*** Alloc RX/TX packet ***/
	txpkt = &priv->txpkt;
	rxpkt = &priv->rxpkt;

	/*** Build command ***/
	txpkt->hdr = &priv->tx_pkt_hdr[0];
	txpkt->hlen = 0;
	txpkt->data = NULL;
	txpkt->dlen = 0;
	txpkt->options = 0;
	if ( rmap_build(priv, cmd, txpkt) ){
		ret = -1;
		goto out_sem;
	}

	/*** Send command ***/
	drv = priv->config->drv;
	if ( !drv->ops.send ){
		ret = -1;
		goto out_sem;
	}

	/* Schedule packet for transmission */
	status = drv->ops.send(priv->drv_cookie, txpkt);
	if ( status ) {
		/* Failed to schedule packet */
		ret = -1;
		goto out_sem;
	}
	cmd->status = 0;

	/*** Wait for response ***/
	if ( wait_response ) {
retry:
		rmap_init_rxpkt(priv, rxpkt);
		status = drv->ops.recv(priv->drv_cookie, rxpkt);
		if ( status < 1 ) {
			/* Failed to receive packet, for example timeout driver error 
			 * or driver not in blocking mode.
			 */
			ret = -2;
			goto out_sem;
		}
		/*** ONE RECEIVED PACKET ***/

		/* Parse input packet */
		if ( rmap_parse(priv, cmd, rxpkt) == -1 ) {
			/* Got wrong packet, for example wrong TID */
			DBG("RMAP: wrong packet\n");
			goto retry;
		}
	}

out_sem:
	if ( priv->lock ) {
		rtems_semaphore_release(priv->lock);
	}
	return ret;
}

int rmap_write(void *cookie, void *dst, void *buf, int length, int dstadr, int dstkey)
{
	struct rmap_command_write writecmd;
	int status;

	/* Build RMAP Write Command. Do not use Verify Write */
	writecmd.type = RMAP_CMD_WI;
	/*writecmd.type = RMAP_CMD_WIV;*/
	writecmd.dstadr = dstadr;
	writecmd.dstkey = dstkey;
	writecmd.address = (unsigned int)dst;
	writecmd.length = length;
	writecmd.data = (unsigned char *)buf;

#ifdef DEBUG
	if ( length == 4 ) {
		printf("RMAP_WRITE(4): 0x%08x <== 0x%08x\n", dst, *(unsigned int *)buf);
	} else {
		printf("RMAP_WRITE: 0x%08x - 0x%08x\n", dst, ((unsigned int)dst)+(length-1));
	}
#endif

	/* Send Command */
	status = rmap_send(cookie, (struct rmap_command *)&writecmd);
	if ( status ) {
		printf("GRTMPAHB WRITE: Failed to send/receive command %d\n", status);
		return -1;
	}

	/* Read Data */
	if ( writecmd.status != 0 ) {
		printf("GRTMPAHB WRITE: Status non-zero 0x%x, address: 0x%llx\n",
			writecmd.status, writecmd.address);
		return -1;
	}

	/* Return sucessful */
	return 0;
}

int rmap_read(void *cookie, void *src, void *buf, int length, int dstadr, int dstkey)
{
	/* Send it, and wait for result */
	struct rmap_command_read readcmd;
	int status;

	/* Build RMAP READ Command */
	readcmd.type = RMAP_CMD_RI;
	readcmd.dstadr = dstadr;
	readcmd.dstkey = dstkey;
	readcmd.address = (unsigned int)src;
	readcmd.length = length;
	readcmd.datalength = 0;
	readcmd.data = buf;

	DBG("RMAP READ1: 0x%08x - 0x%08x\n", (unsigned int)src, ((unsigned int)src + (readcmd.length - 1)));

	/* Send Command */
	status = rmap_send(cookie, (struct rmap_command *)&readcmd);
	if ( status ) {
		printf("RMAP READ: Failed to send/receive command %d\n", status);
		memset(buf, 0, length);
		/* Should we remove device? */
		return -1;
	}

	/* Read Data */
	if ( readcmd.status != 0 ) {
		printf("RMAP READ: Status non-zero 0x%x, dlen: 0x%x\n", readcmd.status, readcmd.datalength);
		memset(buf, 0, length);
		/* Should we remove device? */
		return -1;
	}

#ifdef DEBUG
	if ( length == 4 ) {
		printf("RMAP READ2: 0x%08x(4): 0x%08x\n", (unsigned int)src, *(unsigned int *)buf);
	} else {
		printf("RMAP READ2: 0x%08x - 0x%08x\n", (unsigned int)src, ((unsigned int)src)+(length-1));
	}
#endif

	/* Return sucessful */
	return 0;
}
