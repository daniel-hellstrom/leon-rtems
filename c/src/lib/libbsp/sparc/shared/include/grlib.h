/*
 *  GRLIB AMBA Core Register definitions
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifndef __GRLIB_H__
#define __GRLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ESA MEMORY CONTROLLER */
typedef struct {
  unsigned int mcfg1;
  unsigned int mcfg2;
  unsigned int mcfg3;
} ambapp_regmap_mctrl;

/* APB UART */
typedef struct {
  volatile unsigned int data;
  volatile unsigned int status;
  volatile unsigned int ctrl;
  volatile unsigned int scaler;
} ambapp_apb_uart;

/* IRQ MP */
typedef struct {
  volatile unsigned int ilevel;      /* 0x00 */
  volatile unsigned int ipend;       /* 0x04 */
  volatile unsigned int iforce;      /* 0x08 */
  volatile unsigned int iclear;      /* 0x0c */
  volatile unsigned int mpstat;      /* 0x10 */
  volatile unsigned int bcast;       /* 0x14 */
  volatile unsigned int notused02;   /* 0x18 */
  volatile unsigned int notused03;   /* 0x1c */
  volatile unsigned int ampctrl;     /* 0x20 */
  volatile unsigned int icsel[2];    /* 0x24,0x28 */
  volatile unsigned int notused13;   /* 0x2c */
  volatile unsigned int notused20;   /* 0x30 */
  volatile unsigned int notused21;   /* 0x34 */
  volatile unsigned int notused22;   /* 0x38 */
  volatile unsigned int notused23;   /* 0x3c */
  volatile unsigned int mask[16];    /* 0x40 */
  volatile unsigned int force[16];   /* 0x80 */
  /* Extended IRQ registers */
  volatile unsigned int intid[16];   /* 0xc0 */
  /* 0x100, align to 4Kb boundary */
  volatile unsigned int resv1[(0x1000-0x100)/4];
} LEON3_IrqCtrl_Regs_Map;

#ifdef __cplusplus
}
#endif

#endif
