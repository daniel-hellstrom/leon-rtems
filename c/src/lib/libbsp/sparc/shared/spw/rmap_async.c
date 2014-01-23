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
#include "rmap.h"

#ifdef DEBUG
 #define DBG(args...) printf(args);
#else
 #define DBG(args...)
#endif

struct rmap_async_priv {
	struct rmap_config	*config;	/* Configuration */
	unsigned short		tid;		/* Current TID */
	char			drv_cap;	/* Driver capabiliteis */
	rtems_id		lock;		/* Optional Semaphore protection against multiple threads (thread-safe) */

	int			resp_array_len;	/* Length of cmds array */
	struct rmap_command	*cmds[];	/* RMAP commands waiting for response */
};

void *rmap_async_init(struct rmap_config *config, int response_array_length)
{
	struct rmap_async_priv *priv;
	int status;

	priv = (struct rmap_async_priv *)malloc(sizeof(struct rmap_async_priv));
	if ( !priv )
		return NULL;

	memset(priv, 0, sizeof(struct rmap_async_priv) +
	       (response_array_length * sizeof(struct rmap_command *)));
	priv->config = config;
	if (config->tid_msb >= 0) {
		priv->tid = config->tid_msb << 8;
	}
	priv->drv_cap = config->drv_cap;
	priv->resp_array_len = response_array_length;

	/* Create Semaphore used to make RMAP layer Thread-safe */
	if (config->thread_safe) {
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

void rmap_async_reset(void *cookie)
{
	struct rmap_async_priv *priv = cookie;

	memset(priv->cmds, 0, 
	       priv->resp_array_len * sizeof(struct rmap_command *));
}

static struct rmap_command *rmap_find_async(void *cookie, unsigned short tid)
{
	struct rmap_async_priv *priv = cookie;
	struct rmap_command *cmd;
	int i;

	for (i = 0; i<priv->resp_array_len; i++) {
		cmd = priv->cmds[i];
		if ((cmd != NULL) && (cmd->tid == tid)) {
		    	priv->cmds[i] = NULL;
			return cmd;
		}
	}

	return NULL;
}

static int rmap_add_async(void *cookie, struct rmap_command *cmd)
{
	struct rmap_async_priv *priv = cookie;
	int i, status, ret;

	if (priv->lock) {
		status = rtems_semaphore_obtain(priv->lock, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
		if (status != RTEMS_SUCCESSFUL) {
			printf("RMAP-STACK: FAILED TO OBTAIN LOCK: %d (%x)\n",
				status, (unsigned int)priv->lock);
			return -2;
		}
	}

	for (i = 0; i<priv->resp_array_len; i++) {
		if (priv->cmds[i] == NULL) {
			priv->cmds[i] = cmd;
			ret = 0;
			goto out_sem;
		}
	}
	ret = -1;

out_sem:
	if (priv->lock)
		rtems_semaphore_release(priv->lock);

	return ret;
}

int rmap_send_async(void *cookie, struct rmap_command *cmd, struct rmap_spw_pkt *pkt)
{
	struct rmap_async_priv *priv = cookie;
	int wait_response;

	if ( !priv || !cmd || !pkt )
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

	/*** Build command ***/
	pkt->hlen = 0;
	pkt->dlen = 0;
	pkt->options = 0;
	if ( rmap_build((struct rmap_common_priv *)priv, cmd, pkt) ) {
		return -2;
	}

	/* Add packet to response list */
	if (wait_response) {
		cmd->status = 0xff; /* indicate to be updated by response */
		if (rmap_add_async(priv, cmd))
			return -3;
	} else {
		cmd->status = 0; /* no response expected. Ok status. */
	}

	return wait_response;
}

int rmap_recv_async(void *cookie, struct rmap_spw_pkt *pkt, struct rmap_command **pcmd)
{
	struct rmap_async_priv *priv = cookie;
	struct rmap_command *cmd;
	unsigned char *data = pkt->data;
	unsigned short tid;

	if (!priv || !pkt || !data)
		return -1;

	/*** From TID find Command structure ***/
	tid = (data[5] << 8) |data[6];

	/* if *pcmd is not NULL, user wants to find a specific command */
	if (pcmd && (*pcmd != NULL) && (tid != (*pcmd)->tid))
		return 1;

	/* Fuind in table of old requests */
	cmd = rmap_find_async(priv, tid);
	if (!cmd)
		return -2;
	if (pcmd && (*pcmd == NULL))
		*pcmd = cmd;

	/* Parse input packet */
	if (rmap_parse((struct rmap_common_priv *)priv, cmd, pkt) != 0)
		return -3; /* Got wrong packet, for example wrong TID */

	return 0;
}

int rmap_cancel_async(void *cookie, struct rmap_command *cmd)
{
	struct rmap_async_priv *priv = cookie;
	int i;

	for (i = 0; i<priv->resp_array_len; i++) {
		if (priv->cmds[i] == cmd) {
		    	priv->cmds[i] = NULL;
			return 0;
		}
	}

	return -1; /* didn't find command in table */
}
