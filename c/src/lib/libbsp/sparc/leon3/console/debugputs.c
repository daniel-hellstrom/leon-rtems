/*
 *  Console Debug interface driver for printk()
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  Modified for LEON3 BSP.
 *  COPYRIGHT (c) 2011.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include <bsp.h>
#include <rtems/libio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

/* Let user override which on-chip APBUART will be debug UART
 * 0 = Default APBUART. On MP system CPU0=APBUART0, CPU1=APBUART1...
 * 1 = APBUART[0]
 * 2 = APBUART[1]
 * 3 = APBUART[2]
 * ...
 */
int debug_uart_index __attribute__((weak)) = 0;

ambapp_apb_uart *dbg_uart = NULL;

/* Before UART driver has registered calls to printk that gets to
 * console_dbg_out_char() will be filling data into the pre_printk_dbgbuf[]
 * buffer, hopefully the buffer can help in debugging the cause. At least
 * the last printk() will be caught.
 */
char pre_printk_dbgbuf[32] = {0};
int pre_printk_pos = 0;

/* This routine polls for one character, return EOF if no char is available */
int apbuart_inbyte_nonblocking(ambapp_apb_uart *regs)
{
	if (regs->status & LEON_REG_UART_STATUS_ERR)
		regs->status = ~LEON_REG_UART_STATUS_ERR;

	if ((regs->status & LEON_REG_UART_STATUS_DR) == 0)
		return EOF;

	return (int)regs->data;
}

/* This routine transmits a character, it will busy-wait until on character
 * fits in the APBUART Transmit FIFO
 */
void apbuart_outbyte_polled(
  ambapp_apb_uart *regs,
  unsigned char ch,
  int do_cr_on_newline,
  int wait_sent)
{
send:
	while ((regs->status & LEON_REG_UART_STATUS_THE) == 0) {
		/* Lower bus utilization while waiting for UART */
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
	}
	regs->data = (unsigned int) ch;

	if ((ch == '\n') && do_cr_on_newline) {
		ch = '\r';
		goto send;
	}

	/* Wait until the character has been sent? */
	if (wait_sent) {
		while ((regs->status & LEON_REG_UART_STATUS_THE) == 0)
			;
	}
}

/* AMBA PP find routine. Extract APBUART information into data structure. */
int find_matching_debug_apbuart(struct ambapp_dev *dev, int index, void *arg)
{
	struct ambapp_apb_info *apb;

	if (debug_uart_index == 0)
		return 1;

	if (--debug_uart_index != 0)
		return 0;

	apb = (struct ambapp_apb_info *)dev->devinfo;

	/* Extract needed information of one APBUART */
	dbg_uart = (ambapp_apb_uart *)apb->start;

	/* We assume that the baud rate and transmission paramters already have
	 * been configured, but the transmitter/receiver is enabled once on
	 * debug UART initialization
	 */
	dbg_uart->ctrl |= LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE;

	return 1; /* stop search */
}

/* Scan AMBA Plu&Play for debug APBUART */
void bsp_debug_uart_init(void)
{
	if (debug_uart_index == 0)
#if defined(RTEMS_MULTIPROCESSING)
	/* On RTEMS AMP systems, CPU0 use APBUART0, CPU1 use APBUART1 ...
	 * unless user has overrided default configuration
	 */
		debug_uart_index = LEON3_Cpu_Index + 1;
#else
		debug_uart_index = 1;
#endif

	/* Find APBUART core */
	ambapp_for_each(&ambapp_plb, (OPTIONS_ALL|OPTIONS_APB_SLVS),
			VENDOR_GAISLER, GAISLER_APBUART,
			find_matching_debug_apbuart, NULL);
}


/* putchar/getchar for printk */
static void bsp_out_char(char c)
{
	if (dbg_uart == NULL) {
		/* Local debug buffer when UART driver has not registered */
		pre_printk_dbgbuf[pre_printk_pos++] = c;
		pre_printk_pos = pre_printk_pos & (sizeof(pre_printk_dbgbuf)-1);
		return;
	}
	apbuart_outbyte_polled(dbg_uart, c, 1, 1);
}

static int bsp_in_char(void)
{
	int tmp;

	if (dbg_uart == NULL)
		return EOF;

	while ((tmp = apbuart_inbyte_nonblocking(dbg_uart)) < 0)
		;
	return tmp;
}

/*
 *  To support printk
 */

#include <rtems/bspIo.h>

BSP_output_char_function_type BSP_output_char = bsp_out_char;
BSP_polling_getchar_function_type BSP_poll_char = bsp_in_char;
