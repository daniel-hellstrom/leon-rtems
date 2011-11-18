/*
 * Header file for RTEMS CAN_MUX driver 
 * 
 * --------------------------------------------------------------------------
 *  --  This file is a part of GAISLER RESEARCH source code.
 *  --  Copyright (C) 2008, Gaisler Research AB - all rights reserved.
 *  --
 *  -- ANY USE OR REDISTRIBUTION IN PART OR IN WHOLE MUST BE HANDLED IN
 *  -- ACCORDANCE WITH THE GAISLER LICENSE AGREEMENT AND MUST BE APPROVED
 *  -- IN ADVANCE IN WRITING.
 *  --
 *  -- BY DEFAULT, DISTRIBUTION OR DISCLOSURE IS NOT PERMITTED.
 *  -------------------------------------------------------------------------- 
 *
 */

#ifndef __CANMUX_H__
#define __CANMUX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Driver interface */
int canmux_register(void);

/* ioctl calls */
#define CANMUX_IOC_BUSA_SATCAN 1
#define CANMUX_IOC_BUSA_OCCAN1 2
#define CANMUX_IOC_BUSB_SATCAN 3
#define CANMUX_IOC_BUSB_OCCAN2 4

#ifdef __cplusplus
}
#endif

#endif /* __CANMUX_H__ */
