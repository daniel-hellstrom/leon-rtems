/*
 *  This file contains the TTY driver for the serial ports on the LEON.
 *
 *  This driver uses the termios pseudo driver.
 *
 *  COPYRIGHT (c) 1989-1998.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  Modified for LEON3 BSP.
 *  COPYRIGHT (c) 2004.
 *  Gaisler Research.
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
#include <rtems/bspIo.h>
#include <amba.h>

#ifndef RTEMS_DRVMGR_STARTUP

#if CONSOLE_USE_INTERRUPTS && defined(RTEMS_MULTIPROCESSING)
#error LEON3 console driver does not support interrupt mode in multi processor systems
#endif

extern void apbuart_outbyte_polled(
  ambapp_apb_uart *regs,
  unsigned char ch,
  int do_cr_on_newline,
  int wait_sent);
extern int apbuart_inbyte_nonblocking(ambapp_apb_uart *regs);

/* Note that it is not possible to use the interrupt mode of the driver
 * together with the "old" APBUART and -u to GRMON. However the new
 * APBUART core (from 1.0.17-b2710) has the GRMON debug bit and can 
 * handle interrupts.
 */

struct apbuart_priv {
  ambapp_apb_uart *regs;
  int irq;
  void *cookie;
  unsigned int freq_hz;
#if CONSOLE_USE_INTERRUPTS
  volatile int sending;
  char *buf;
#endif
};
static struct apbuart_priv apbuarts[CONFIGURE_NUMBER_OF_TERMIOS_PORTS];
static int uarts = 0;

/*
 *  Should we use a polled or interrupt drived console?
 *
 *  NOTE: This is defined in the custom/leon.cfg file.
 */

int console_pollRead( int minor )
{
  return apbuart_inbyte_nonblocking(apbuarts[minor+LEON3_Cpu_Index].regs);
}

#if CONSOLE_USE_INTERRUPTS

/* Handle UART interrupts */
void console_isr(void *arg)
{
  struct apbuart_priv *uart = arg;
  unsigned int status;
  char data;

  /* Clear interrupt */
  LEON_Clear_interrupt(uart->irq);

  /* Get all received characters */
  while ( (status=uart->regs->status) & LEON_REG_UART_STATUS_DR ){
    /* Data has arrived, get new data */
    data = uart->regs->data;

    /* Tell termios layer about new character */
    rtems_termios_enqueue_raw_characters(uart->cookie,&data,1);
  }

  if ( status & LEON_REG_UART_STATUS_THE ){
    /* Sent the one char, we disable TX interrupts */
    uart->regs->ctrl &= ~LEON_REG_UART_CTRL_TI;

    /* Tell close that we sent everything */
    uart->sending = 0;

    /* write_interrupt will get called from this function */
    rtems_termios_dequeue_characters(uart->cookie,1);
  }
}

int console_write_interrupt (int minor, const char *buf, int len)
{
  struct apbuart_priv *uart;
  unsigned int oldLevel;

  if ( minor == 0 )
    minor = LEON3_Cpu_Index;

  uart = &apbuarts[minor];

  /* Remember what position in buffer */

  rtems_interrupt_disable(oldLevel);

  /* Enable TX interrupt */
  uart->regs->ctrl |= LEON_REG_UART_CTRL_TI;

  /* start UART TX, this will result in an interrupt when done */
  uart->regs->data = *buf;

  uart->sending = 1;

  rtems_interrupt_enable(oldLevel);

  return 0;
}
#endif
/*
 *  Console Termios Support Entry Points
 *
 */

ssize_t console_write_polled (int minor, const char *buf, size_t len)
{
  int nwrite = 0;
  struct apbuart_priv *uart;

  if ( minor == 0 )
    minor = LEON3_Cpu_Index;

  if ( minor >= uarts )
    return -1;

  uart = &apbuarts[minor];

  while (nwrite < len) {
    apbuart_outbyte_polled( uart->regs, *buf++, 1, 0 );
    nwrite++;
  }
  return nwrite;
}

int console_set_attributes(int minor, const struct termios *t)
{
  unsigned int scaler;
  unsigned int ctrl;
  int baud;
  struct apbuart_priv *uart;

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

  if ( minor == 0 )
    minor = LEON3_Cpu_Index;

  if ( minor >= uarts )
    return -1;

  uart = &apbuarts[minor];

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
    default:      baud = -1;      break;
    case B50:     baud = 50;      break;
    case B75:     baud = 75;      break;
    case B110:    baud = 110;     break;
    case B134:    baud = 134;     break;
    case B150:    baud = 150;     break;
    case B200:    baud = 200;     break;
    case B300:    baud = 300;     break;
    case B600:    baud = 600;     break;
    case B1200:   baud = 1200;    break;
    case B1800:   baud = 1800;    break;
    case B2400:   baud = 2400;    break;
    case B4800:   baud = 4800;    break;
    case B9600:   baud = 9600;    break;
    case B19200:  baud = 19200;   break;
    case B38400:  baud = 38400;   break;
    case B57600:  baud = 57600;   break;
    case B115200: baud = 115200;  break;
    case B230400: baud = 230400;  break;
    case B460800: baud = 460800;  break;
  }

  if ( baud > 0 ){
    /* Calculate Baud rate generator "scaler" number */
    scaler = (((uart->freq_hz * 10)/(baud * 8)) - 5) / 10;

    /* Set new baud rate by setting scaler */
    uart->regs->scaler = scaler;
  }

  return 0;
}

/* AMBA PP find routine. Extract AMBA PnP information into data structure. */
int find_matching_apbuart(struct ambapp_dev *dev, int index, void *arg)
{
  struct ambapp_apb_info *apb = (struct ambapp_apb_info *)dev->devinfo;

  /* Extract needed information of one APBUART */
  apbuarts[uarts].regs = (ambapp_apb_uart *)apb->start;
  apbuarts[uarts].irq = apb->irq;
  /* Get APBUART core frequency, it is assumed that it is the same
   * as Bus frequency where the UART is situated
   */
  apbuarts[uarts].freq_hz = ambapp_freq_get(&ambapp_plb, dev);
  uarts++;

  if (uarts >= CONFIGURE_NUMBER_OF_TERMIOS_PORTS)
    return 1; /* Satisfied number of UARTs, stop search */
  else
    return 0; /* Continue searching for more UARTs */
}

/*
 *  Console Device Driver Entry Points
 *
 */

int scan_uarts(void)
{
  if (uarts == 0) {
    memset(apbuarts, 0, sizeof(apbuarts));

    /* Find APBUART cores */
    ambapp_for_each(&ambapp_plb, (OPTIONS_ALL|OPTIONS_APB_SLVS), VENDOR_GAISLER,
                    GAISLER_APBUART, find_matching_apbuart, NULL);
  }

  return uarts;
}

rtems_device_driver console_initialize(
  rtems_device_major_number  major,
  rtems_device_minor_number  minor,
  void                      *arg
)
{
  rtems_status_code status;
  int i, uart0;
  char console_name[16];

  rtems_termios_initialize();

  /* Find UARTs */
  scan_uarts();

  /* Let LEON3 CPU index select UART, this is useful in AMP
   * systems. LEON3_Cpu_Index is zero for CPU0, one for CPU1...
   * Note that looking at the RTEMS_MULTIPROCESSING macro
   * does not work when different operating systems are used
   * and RTEMS is configured non-MP.
   */
  uart0 = LEON3_Cpu_Index;

  /*  Register Device Names */
  if (uarts && (uart0 < uarts)) 
  {
    status = rtems_io_register_name( "/dev/console", major, 0 );
    if (status != RTEMS_SUCCESSFUL)
      rtems_fatal_error_occurred(status);

    strcpy(console_name,"/dev/console_a");
    for (i = uart0+1; i < uarts; i++) {
      console_name[13]++;
      status = rtems_io_register_name( console_name, major, i);
    }
  }

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver console_open(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  rtems_status_code sc;
  struct apbuart_priv *uart;
  rtems_libio_open_close_args_t *priv = arg;

#if CONSOLE_USE_INTERRUPTS
  /* Interrupt mode routines */
  static const rtems_termios_callbacks Callbacks = {
    NULL,                        /* firstOpen */
    NULL,                        /* lastClose */
    NULL,                        /* pollRead */
    console_write_interrupt,     /* write */
    console_set_attributes,      /* setAttributes */
    NULL,                        /* stopRemoteTx */
    NULL,                        /* startRemoteTx */
    1                            /* outputUsesInterrupts */
  };
#else
  /* Polling mode routines */
  static const rtems_termios_callbacks Callbacks = {
    NULL,                        /* firstOpen */
    NULL,                        /* lastClose */
    console_pollRead,            /* pollRead */
    console_write_polled,        /* write */
    console_set_attributes,      /* setAttributes */
    NULL,                        /* stopRemoteTx */
    NULL,                        /* startRemoteTx */
    0                            /* outputUsesInterrupts */
  };
#endif

  assert( minor < uarts );
  if ( minor >= uarts )
    return RTEMS_INVALID_NUMBER;

  sc = rtems_termios_open (major, minor, arg, &Callbacks);
  if (sc != RTEMS_SUCCESSFUL)
    return sc;

  if ( minor == 0 )
    minor = LEON3_Cpu_Index;
  uart = &apbuarts[minor];

  if ( priv && priv->iop )
    uart->cookie = priv->iop->data1;
  else
    uart->cookie = NULL;

#if CONSOLE_USE_INTERRUPTS
  /* Register Interrupt handler */
  sc = rtems_interrupt_handler_install(uart->irq, "console",
                                       RTEMS_INTERRUPT_SHARED, console_isr,
				       uart);
  if (sc != RTEMS_SUCCESSFUL)
    return sc;

  uart->sending = 0;
  /* Enable Receiver and transmitter and Turn on RX interrupts */
  uart->regs->ctrl |= LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE |
                      LEON_REG_UART_CTRL_RI;
#else
  /* Enable Receiver and transmitter */
  uart->regs->ctrl |= LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE;
#endif
  uart->regs->status = 0;

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver console_close(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
#if CONSOLE_USE_INTERRUPTS
  struct apbuart_priv *uart;

  if ( minor == 0)
    minor = LEON3_Cpu_Index;

  /* Turn off RX interrupts */
  uart = &apbuarts[minor];
  uart->regs->ctrl &= ~(LEON_REG_UART_CTRL_RI);

  /**** Flush device ****/
  while ( uart->sending ) {
    /* Wait until all data has been sent */
  }

  /* uninstall ISR */
  rtems_interrupt_handler_remove(uart->irq, console_isr, uart);

#endif
  return rtems_termios_close (arg);
}

rtems_device_driver console_read(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_read (arg);
}

rtems_device_driver console_write(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_write (arg);
}

rtems_device_driver console_control(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_ioctl (arg);
}

#endif
