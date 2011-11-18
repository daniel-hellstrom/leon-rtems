/*  GRSPW RMAP Driver it use the standard GRSPW RTEMS DRIVER
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

#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <grspw.h>
#include "rmap.h"
#include "rmap_drv_grspw.h"

struct rmap_drv_grspw_priv {
	struct rmap_drv_grspw_config *config;
};

void *rmap_drv_grspw_init(struct rmap_drv_grspw_config *config)
{
	struct rmap_drv_grspw_priv *priv;

	priv = (struct rmap_drv_grspw_priv *)malloc(sizeof(struct rmap_drv_grspw_priv));
	if ( !priv )
		return NULL;
	memset(priv, 0, sizeof(struct rmap_drv_grspw_priv));

	priv->config = config;

	return priv;
}

int rmap_drv_grspw_ops_init(void *cookie)
{
	struct rmap_drv_grspw_priv *priv = cookie;
	return 0;
}

int rmap_drv_grspw_get_cap(struct rmap_drv_grspw_priv *priv, unsigned int *result)
{
	spw_config grspw_config;
	int status;
	unsigned int options;

	status = ioctl(priv->config->fd, SPACEWIRE_IOCTRL_GET_CONFIG, &grspw_config);
	if ( status != 0 ) {
		*result = 0;
		return -1;
	}

	/* Get capabilities from GRSPW driver */	
	options = 0;
	if ( grspw_config.is_rmap || grspw_config.is_rmapcrc )
		options |= (PKT_OPTION_HDR_CRC | PKT_OPTION_DATA_CRC);

	*result = options;

	return 0;
}

int rmap_drv_grspw_set_rtimeout(struct rmap_drv_grspw_priv *priv, unsigned int timeout)
{
	int status;

	status = ioctl(priv->config->fd, SPACEWIRE_IOCTRL_SET_READ_TIMEOUT, timeout);
	if ( status != 0 ) {
		return -1;
	}

	return 0;
}


int rmap_drv_grspw_ops_ioctl(void *cookie, int command, void *arg)
{
	struct rmap_drv_grspw_priv *priv = cookie;
	struct rmap_drv_timeout *timeout;

	switch ( command ) {
		case RMAP_DRV_IOCTL_START:
		case RMAP_DRV_IOCTL_STOP:
		case RMAP_DRV_IOCTL_BLOCK:		/* Only blocking mode supported */
			break;
		case RMAP_DRV_IOCTL_TIMEOUT:		/* Only Read timeout support */
			timeout = arg;
			if ( timeout->options & RMAP_TIMEOUT_SET_RTIME ) {
				/* Set read timeout */
				rmap_drv_grspw_set_rtimeout(priv, timeout->rtimeout);
			}
			break;
		case RMAP_DRV_IOCTL_GET_CAP:		/* No DATA/HEADER CRC capabilities */
			return rmap_drv_grspw_get_cap(cookie, (unsigned int *)arg);
		default:
			return -1;
	}
	return 0;
}

int rmap_drv_grspw_ops_send(void *cookie, struct rmap_spw_pkt *pkt)
{
	struct rmap_drv_grspw_priv *priv = cookie;
	spw_ioctl_pkt_send *grspw_pkt;
	int status;

	grspw_pkt = (spw_ioctl_pkt_send *)&pkt->hlen;
	grspw_pkt->options = pkt->options & (PKT_OPTION_HDR_CRC_SKIPLEN_MASK|
					PKT_OPTION_DATA_CRC|PKT_OPTION_HDR_CRC);
	grspw_pkt->sent = 0;

	/* Send packet by using the IOCTL SEND method, it takes separate header and data arguments */
	status = ioctl(priv->config->fd, SPACEWIRE_IOCTRL_SEND, grspw_pkt);
	if ( status < 0 ) {
		return -1;
	}
	return 0;
}

int rmap_drv_grspw_ops_recv(void *cookie, struct rmap_spw_pkt *pkt)
{
	struct rmap_drv_grspw_priv *priv = cookie;
	int len;

	if ( !pkt->data ) {
		return -1;
	}

	len = read(priv->config->fd, pkt->data, pkt->dlen);
	if ( len < 0 ) {
		printf("RMAP_DRV_GRSPW: Error reading: %s (%d, %d)", strerror(errno), errno, len);
		return -1;
	} else if ( len == 0 ) {
		if ( errno == ETIMEDOUT ) {
			/* Read Timeout */
			return -2;
		}
		return -1;
	}

	/* Update packet size */
	pkt->dlen = len;

	return 1; /* Received one packet */
}

struct rmap_drv_ops rmap_grspw_ops = 
{
	.init	= rmap_drv_grspw_ops_init,
	.ioctl	= rmap_drv_grspw_ops_ioctl,
	.send	= rmap_drv_grspw_ops_send,
	.recv	= rmap_drv_grspw_ops_recv
};
