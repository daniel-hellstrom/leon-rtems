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

int rmap_stack_count = 0;

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

int rmap_build(struct rmap_common_priv *priv, struct rmap_command *cmd, struct rmap_spw_pkt *txpkt)
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
		if ( cmd->data.write.data != 0 ) {
			txpkt->data = cmd->data.write.data; /* CRC will be added later */
		} else {
			txpkt->data = ((unsigned char *)&txpkt->dlen + 1); /* Temporary storage of CRC when no data */
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

int rmap_parse(struct rmap_common_priv *priv, struct rmap_command *cmd, struct rmap_spw_pkt *rxpkt)
{
	unsigned char *pkt;
	unsigned int length;

	pkt = rxpkt->data;

	/* Check that the protocol ID, Reply and TID matches before proceeding */
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
		if (cmd->data.read.data == NULL)
			cmd->data.read.data = (unsigned int)&pkt[12];
		else
			memcpy(cmd->data.read.data, &pkt[12], length);
	}

	return 0;
}
