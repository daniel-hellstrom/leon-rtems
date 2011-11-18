/*  This file contains the driver for the GRLIB APBUART serial port. The driver
 *  is implemented by using the cons.c console layer. Interrupt/Polling/Task
 *  driven mode can be configured using driver resources:
 *
 *  - mode   (0=Polling, 1=Interrupt, 2=Task-Driven-Interrupt Mode)
 *  - syscon (0=Force not Ssystem Console, 1=Suggest System Console)
 *  - dbgcon (0=Force not Debug Console, 1=Suggest Debug Console)
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2010-09-27, Daniel Hellstrom <daniel@gaisler.com>
 *   created
 */

/******************* Driver manager interface ***********************/
#include <bsp.h>
#include <rtems/libio.h>
#include <stdlib.h>
#include <assert.h>
#include <rtems/bspIo.h>
#include <string.h>
#include <stdio.h>

#include <drvmgr/drvmgr.h>
#include <drvmgr/ambapp_bus.h>
#include <apbuart.h>
#include <ambapp.h>
#include <grlib.h>
#include <cons.h>
#include <rtems/termiostypes.h>

/*#define DEBUG 1  */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

struct apbuart_priv {
	struct console_dev condev;
	struct rtems_drvmgr_dev_info *dev;
	ambapp_apb_uart *regs;
	char devName[32];
	void *cookie;
	int sending;
	int mode;
	int dbg;
};

/* TERMIOS Layer Callback functions */
int apbuart_set_attributes(int minor, const struct termios *t);
int apbuart_write_polled(int minor, const char *buf, int len);
int apbuart_pollRead(int minor);
int apbuart_write_intr(int minor, const char *buf, int len);
int apbuart_pollRead_task(int minor);
int apbuart_firstOpen(int major, int minor, void *arg);
int apbuart_lastClose(int major, int minor, void *arg);

/* Printk Debug functions */
void apbuart_dbg_init(struct console_dev *);
char apbuart_dbg_in_char(struct console_dev *);
void apbuart_dbg_out_char(struct console_dev *, char c);

void apbuart_isr(int irqno, void *arg);

int apbuart_init1(struct rtems_drvmgr_dev_info *dev);

struct rtems_drvmgr_drv_ops apbuart_ops = 
{
	.init = {apbuart_init1, NULL, NULL, NULL},
	.remove = NULL,
	.info = NULL
};

static struct amba_dev_id apbuart_ids[] =
{
	{VENDOR_GAISLER, GAISLER_APBUART},
	{0, 0}		/* Mark end of table */
};

static struct amba_drv_info apbuart_drv_info =
{
	{
		NULL,					/* Next driver */
		NULL,					/* Device list */
		DRIVER_AMBAPP_GAISLER_APBUART_ID,	/* Driver ID */
		"APBUART_DRV",				/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,			/* Bus Type */
		&apbuart_ops,
		0,					/* No devices yet */
		sizeof(struct apbuart_priv),		/*DrvMgr alloc private*/
	},
	&apbuart_ids[0]
};

void apbuart_cons_register_drv (void)
{
	DBG("Registering APBUART Console driver\n");
	rtems_drvmgr_drv_register(&apbuart_drv_info.general);
}

/* Interrupt mode routines */
static const rtems_termios_callbacks Callbacks_intr = {
    apbuart_firstOpen,           /* firstOpen */
    NULL,                        /* lastClose */
    NULL,                        /* pollRead */
    apbuart_write_intr,          /* write */
    apbuart_set_attributes,      /* setAttributes */
    NULL,                        /* stopRemoteTx */
    NULL,                        /* startRemoteTx */
    TERMIOS_IRQ_DRIVEN           /* outputUsesInterrupts */
};

/* Polling mode routines */
static const rtems_termios_callbacks Callbacks_task = {
    apbuart_firstOpen,           /* firstOpen */
    NULL,                        /* lastClose */
    apbuart_pollRead_task,       /* pollRead */
    apbuart_write_intr,          /* write */
    apbuart_set_attributes,      /* setAttributes */
    NULL,                        /* stopRemoteTx */
    NULL,                        /* startRemoteTx */
    TERMIOS_TASK_DRIVEN          /* outputUsesInterrupts */
};

/* Polling mode routines */
static const rtems_termios_callbacks Callbacks_poll = {
    apbuart_firstOpen,           /* firstOpen */
    NULL,                        /* lastClose */
    apbuart_pollRead,            /* pollRead */
    apbuart_write_polled,        /* write */
    apbuart_set_attributes,      /* setAttributes */
    NULL,                        /* stopRemoteTx */
    NULL,                        /* startRemoteTx */
    TERMIOS_POLLED               /* outputUsesInterrupts */
};

struct console_dbg_ops apbuart_dbg_ops = {
	.init = apbuart_dbg_init,
	.in_char = apbuart_dbg_in_char,
	.out_char = apbuart_dbg_out_char,
};

int apbuart_init1(struct rtems_drvmgr_dev_info *dev)
{
	struct apbuart_priv *priv;
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;
	union rtems_drvmgr_key_value *value;
	char prefix[32];
	unsigned int db;

	DBG("APBUART[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);
	/* Private data was allocated and zeroed by driver manager */
	priv = dev->priv;
	priv->dev = dev;

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)priv->dev->businfo;
	if ( ambadev == NULL ) {
		return -1;
	}
	pnpinfo = &ambadev->info;
	priv->regs = (ambapp_apb_uart *)pnpinfo->apb_slv->start;

	/* Clear HW regs, leave baudrate register as it is */
	priv->regs->status = 0;
	db = priv->regs->ctrl & LEON_REG_UART_CTRL_DB; /*leave debug bit*/
	priv->regs->ctrl = db;

	/* The system console and Debug console may depend on this device, so
	 * initialize it straight away.
	 *
	 * We default to have System Console on first APBUART, user may override
	 * this behaviour by setting the syscon option to 0, same goes for
	 * printk() debug console (dbgcon=0).
	 */
	if ( rtems_drvmgr_on_rootbus(dev) && (dev->minor_drv == 0) )
		priv->condev.flags = CONSOLE_FLAG_SYSCON | CONSOLE_FLAG_DBGCON;
	else
		priv->condev.flags = 0;

	value = rtems_drvmgr_dev_key_get(priv->dev, "syscon", KEY_TYPE_INT);
	if ( value ) {
		if ( value->i )
			priv->condev.flags |= CONSOLE_FLAG_SYSCON;
		else
			priv->condev.flags &= ~CONSOLE_FLAG_SYSCON;
	}
	value = rtems_drvmgr_dev_key_get(priv->dev, "dbgcon", KEY_TYPE_INT);
	if ( value ) {
		if ( value->i )
			priv->condev.flags |= CONSOLE_FLAG_DBGCON;
		else
			priv->condev.flags &= ~CONSOLE_FLAG_DBGCON;
	}

	priv->condev.dbgops = &apbuart_dbg_ops;
	priv->condev.fsname = NULL;

	/* Select 0=Polled, 1=IRQ, 2=Task-Driven UART Mode */
	value = rtems_drvmgr_dev_key_get(priv->dev, "mode", KEY_TYPE_INT);
	if ( value )
		priv->mode = value->i;
	else
		priv->mode = TERMIOS_POLLED;
	if ( priv->mode == TERMIOS_IRQ_DRIVEN ) {
		priv->condev.callbacks = &Callbacks_intr;
	} else if ( priv->mode == TERMIOS_TASK_DRIVEN ) {
		priv->condev.callbacks = &Callbacks_task;
	} else {
		priv->condev.callbacks = &Callbacks_poll;
	}

	/* Get Filesystem name prefix */
	prefix[0] = '\0';
	if ( rtems_drvmgr_get_dev_prefix(dev, prefix) ) {
		/* Got special prefix, this means we have a bus prefix
		 * And we should use our "bus minor"
		 */
		sprintf(priv->devName, "/dev/%sapbuart%d", prefix, dev->minor_bus);
		priv->condev.fsname = priv->devName;
	}

	/* Register it as a console device, the console driver will register
	 * a termios device as well
	 */
	console_dev_register(&priv->condev);

	return DRVMGR_OK;
}

/* This routine transmits a character, it will busy-wait until on character
 * fits in the APBUART Transmit FIFO
 */
void apbuart_outbyte_polled(
  struct apbuart_priv *uart,
  unsigned char ch,
  int do_cr_on_newline,
  int wait_sent)
{
send:
	while ( (uart->regs->status & LEON_REG_UART_STATUS_THE) == 0 ) {
		/* Lower bus utilization while waiting for UART */
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
		asm volatile ("nop"::);	asm volatile ("nop"::);
	}
	uart->regs->data = (unsigned int) ch;

	if ((ch == '\n') && do_cr_on_newline) {
		ch = '\r';
		goto send;
	}

	/* Wait until the character has been sent? */
	if ( wait_sent ) {
		while ( (uart->regs->status & LEON_REG_UART_STATUS_THE) == 0 )
			;
	}
}

/* This routine polls for one character, return EOF if no character is available */
int apbuart_inbyte_nonblocking(struct apbuart_priv *uart)
{
	if (uart->regs->status & LEON_REG_UART_STATUS_ERR) {
		uart->regs->status = ~LEON_REG_UART_STATUS_ERR;
	}

	if ((uart->regs->status & LEON_REG_UART_STATUS_DR) == 0)
		return EOF;

	return (int)uart->regs->data;
}

int apbuart_firstOpen(int major, int minor, void *arg)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;
	rtems_libio_open_close_args_t *ioarg = arg;

	if ( ioarg && ioarg->iop )
		uart->cookie = ioarg->iop->data1;
	else
		uart->cookie = NULL;

	/* Enable TX/RX */
	uart->regs->ctrl |= LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE;

	if ( uart->mode != TERMIOS_POLLED ) {
		/* Register interrupt and enable it */
		rtems_drvmgr_interrupt_register(uart->dev, 0, apbuart_isr, uart);
		rtems_drvmgr_interrupt_enable(uart->dev, 0, apbuart_isr, uart);

		uart->sending = 0;
		/* Turn on RX interrupts */
		uart->regs->ctrl |= LEON_REG_UART_CTRL_RI;
	}

	return 0;
}

int apbuart_lastClose(int major, int minor, void *arg)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;

	if ( uart->mode != TERMIOS_POLLED ) {
		/* Turn off RX interrupts */
		uart->regs->ctrl &= ~(LEON_REG_UART_CTRL_RI);

		/**** Flush device ****/
		while ( uart->sending ){
			/* Wait until all data has been sent */
		}

		/* Disable and unregister interrupt handler */
		rtems_drvmgr_interrupt_disable(uart->dev, 0, apbuart_isr, uart);
		rtems_drvmgr_interrupt_unregister(uart->dev, 0, apbuart_isr, uart);
	}

	/* Disable TX/RX if not used for DEBUG */
	if (uart->dbg == 0)
		uart->regs->ctrl &= ~(LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE);

	return 0;
}

int apbuart_pollRead(int minor)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;

	return apbuart_inbyte_nonblocking(uart);
}

int apbuart_pollRead_task(int minor)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;
	int c, tot;
	char buf[32];

	tot = 0;
	while ( (c=apbuart_inbyte_nonblocking(uart)) != EOF ) {
		buf[tot] = c;
		tot++;
		if ( tot > 31 ) {
			rtems_termios_enqueue_raw_characters(uart->cookie, buf, tot);
			tot = 0;
		}
	}
	if ( tot > 0 ) {
		rtems_termios_enqueue_raw_characters(uart->cookie, buf, tot);
	}
	return EOF;
}

int apbuart_set_attributes(int minor, const struct termios *t)
{
	unsigned int core_clk_hz;
	unsigned int scaler;
	unsigned int ctrl;
	int baud;
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;

	switch(t->c_cflag & CSIZE) {
		default:
		case CS5:
		case CS6:
		case CS7:
			/* Hardware doesn't support other than CS8 */
			return -1;
		case CS8:
			break;
	}

	/* Read out current value */
	ctrl = uart->regs->ctrl;

	switch(t->c_cflag & (PARENB|PARODD)){
		case (PARENB|PARODD):
			/* Odd parity */
			ctrl |= LEON_REG_UART_CTRL_PE|LEON_REG_UART_CTRL_PS;
			break;

		case PARENB:
			/* Even parity */
			ctrl &= ~LEON_REG_UART_CTRL_PS;
			ctrl |= LEON_REG_UART_CTRL_PE;
			break;

		default:
		case 0:
		case PARODD:
			/* No Parity */
			ctrl &= ~(LEON_REG_UART_CTRL_PS|LEON_REG_UART_CTRL_PE);
	}

	if ( !(t->c_cflag & CLOCAL) ){
		ctrl |= LEON_REG_UART_CTRL_FL;
	}else{
		ctrl &= ~LEON_REG_UART_CTRL_FL;
	}

	/* Update new settings */
	uart->regs->ctrl = ctrl;

	/* Baud rate */
	switch(t->c_cflag & CBAUD){
		default:	baud = -1;	break;
		case B50:	baud = 50;	break;
		case B75:	baud = 75;	break;
		case B110:	baud = 110;	break;
		case B134:	baud = 134;	break;
		case B150:	baud = 150;	break;
		case B200:	baud = 200;	break;
		case B300:	baud = 300;	break;
		case B600:	baud = 600;	break;
		case B1200:	baud = 1200;	break;
		case B1800:	baud = 1800;	break;
		case B2400:	baud = 2400;	break;
		case B4800:	baud = 4800;	break;
		case B9600:	baud = 9600;	break;
		case B19200:	baud = 19200;	break;
		case B38400:	baud = 38400;	break;
		case B57600:	baud = 57600;	break;
		case B115200:	baud = 115200;	break;
		case B230400:	baud = 230400;	break;
		case B460800:	baud = 460800;	break;
	}

	if ( baud > 0 ){
		/* Get APBUART core frequency */
		rtems_drvmgr_freq_get(uart->dev, DEV_APB_SLV, &core_clk_hz);

		/* Calculate Baud rate generator "scaler" number */
		scaler = (((core_clk_hz*10)/(baud*8))-5)/10;

		/* Set new baud rate by setting scaler */
		uart->regs->scaler = scaler;
	}

	return 0;
}

int apbuart_write_polled(int minor, const char *buf, int len)
{
	int nwrite = 0;
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;

	while (nwrite < len) {
		apbuart_outbyte_polled(uart, *buf++, 1, 0);
		nwrite++;
	}
	return nwrite;
}

int apbuart_write_intr(int minor, const char *buf, int len)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)minor;
	unsigned int oldLevel;
	unsigned int ctrl;

	rtems_interrupt_disable(oldLevel);

	/* Enable TX interrupt */
	ctrl = uart->regs->ctrl;
	uart->regs->ctrl = ctrl | LEON_REG_UART_CTRL_TI;

	if (ctrl & LEON_REG_UART_CTRL_FA) {
		/* APBUART with FIFO.. Fill as many as FIFO allows */
		uart->sending = 0;
		while (((uart->regs->status & LEON_REG_UART_STATUS_TF) == 0) &&
		       (uart->sending < len)) {
			uart->regs->data = *buf;
			buf++;
			uart->sending++;
		}
	} else {
		/* start UART TX, this will result in an interrupt when done */
		uart->regs->data = *buf;

		uart->sending = 1;
	}

	rtems_interrupt_enable(oldLevel);

	return 0;
}

/* Handle UART interrupts */
void apbuart_isr(int irqno, void *arg)
{
	struct apbuart_priv *uart = arg;
	unsigned int status;
	char data;
	int cnt;

	/* Get all received characters */
	if ( uart->mode == TERMIOS_TASK_DRIVEN ) {
		if ( (status=uart->regs->status) & LEON_REG_UART_STATUS_DR ) {
			rtems_termios_rxirq_occured(uart->cookie);
		}
	} else {
		while ( (status=uart->regs->status) & LEON_REG_UART_STATUS_DR ) {
			/* Data has arrived, get new data */
			data = uart->regs->data;

			/* Tell termios layer about new character */
			rtems_termios_enqueue_raw_characters(uart->cookie, &data, 1);
		}
	}

	if ( uart->sending && (status & LEON_REG_UART_STATUS_THE) ) {
		/* Sent the one char, we disable TX interrupts */
		uart->regs->ctrl &= ~LEON_REG_UART_CTRL_TI;

		/* Tell close that we sent everything */
		cnt = uart->sending;
		uart->sending = 0;

		/* apbuart_write_intr() will get called from this function */
		rtems_termios_dequeue_characters(uart->cookie, cnt);
	}
}


/* Printk Debug functions */
void apbuart_dbg_init(struct console_dev *condev)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)condev;

	uart->dbg = 1;
	/* Enable transmitter once on debug UART initialization */
	uart->regs->ctrl |= LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE;
}

char apbuart_dbg_in_char(struct console_dev *condev)
{
	int tmp;
	struct apbuart_priv *uart = (struct apbuart_priv *)condev;

	while ((tmp = apbuart_inbyte_nonblocking(uart)) < 0)
		;

	return (char)tmp;
}

void apbuart_dbg_out_char(struct console_dev *condev, char c)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)condev;

	apbuart_outbyte_polled(uart, c, 1, 1);
}
