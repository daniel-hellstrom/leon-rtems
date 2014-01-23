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

struct rmap_priv {
	/* Common structure */
	struct rmap_config	*config;		/* Configuration */
	unsigned short		tid;			/* Current TID */
	unsigned char		drv_cap;		/* Driver capabilities */
	rtems_id		lock;			/* Optional Semaphore protection against multiple threads (thread-safe) */

	void			*drv_cookie;		/* Driver private structure */
	struct rmap_drv_timeout timeout;		/* Driver timeout */
	char			running;		/* 1 is started, 0 if stopped */
	unsigned char		blocking;		/* Blocking mode */
	unsigned char		tx_pkt_hdr[256+9];	/* Packet header used for transmission, last 9 bytes is for RMW commands */
	unsigned int		_rx_pkt_buf;		/* RX packet Buffer */
	unsigned int		*rx_pkt_buf;		/* RX packet Buffer aligned to 64b*/
	unsigned int		rx_pkt_buf_len;		/* RX packet Buffer length */
	struct rmap_spw_pkt	rxpkt;			/* RX packet */
	struct rmap_spw_pkt	txpkt;			/* TX packet */

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
			if (priv->config->drv_cap != 0) {
				priv->drv_cap = priv->config->drv_cap;
			} else if ( priv->config->drv->ops.ioctl(priv->drv_cookie, RMAP_DRV_IOCTL_GET_CAP, &capabilities) ){
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
	if (	((cmd->type & RMAP_CMD_WRITE) == 0) ||	/* READ ALWAYS RETURNS DATA */
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
			printf("RMAP-STACK: FAILED TO OBTAIN LOCK: %d (%x)\n",
				status, (unsigned int)priv->lock);
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
	if ( rmap_build((struct rmap_common_priv *)priv, cmd, txpkt) ){
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
		if (rmap_parse((struct rmap_common_priv *)priv, cmd, rxpkt)
		    == -1) {
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
