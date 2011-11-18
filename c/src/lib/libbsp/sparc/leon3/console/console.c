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

/* First UART found in AMBA Plug & Play used */
#define LEON3_UART_INDEX 0

#if CONSOLE_USE_INTERRUPTS && defined(RTEMS_MULTIPROCESSING)
#error LEON3 console driver does not support interrupt mode in multi processor systems
#endif

/* Note that it is not possible to use the interrupt mode of the driver
 * together with the "old" APBUART and -u to GRMON. However the new
 * APBUART core (from 1.0.17-b2710) has the GRMON debug bit and can 
 * handle interrupts.
 */

struct apbuart_priv {
  volatile LEON3_UART_Regs_Map *regs;
  int irq;
  void *cookie;
#if CONSOLE_USE_INTERRUPTS
  volatile int sending;
  char *buf;
#endif
};
static struct apbuart_priv apbuarts[CONFIGURE_NUMBER_OF_TERMIOS_PORTS];
static int uarts = 0;
static int isinit = 0;

/*
 *  Should we use a polled or interrupt drived console?
 *
 *  NOTE: This is defined in the custom/leon.cfg file.
 */

/*
 *  console_outbyte_polled
 *
 *  This routine transmits a character using polling.
 */

void console_outbyte_polled(
  struct apbuart_priv *uart,
  unsigned char ch
)
{

send:
  while ( (uart->regs->status & LEON_REG_UART_STATUS_THE) == 0 );
  uart->regs->data = (unsigned int) ch;

  if ( ch == '\n' ){
    ch = '\r';
    goto send;
  }
}

/*
 *  console_inbyte_nonblocking
 *
 *  This routine polls for a character.
 */

int console_inbyte_nonblocking( struct apbuart_priv *uart )
{

  if (uart->regs->status & LEON_REG_UART_STATUS_ERR) {
    uart->regs->status = ~LEON_REG_UART_STATUS_ERR;
  }

  if ((uart->regs->status & LEON_REG_UART_STATUS_DR) == 0)
    return -1;

  return (int) uart->regs->data;
}

int console_pollRead( int minor )
{
  return console_inbyte_nonblocking(&apbuarts[minor+LEON3_Cpu_Index]);
}

#if CONSOLE_USE_INTERRUPTS

/* Handle UART interrupts */
void console_isr_handler(struct apbuart_priv *uart)
{
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
#if 0
    /* More to transmit? */
    if ( (left>0) ){
      uart->regs->data = uart->buf;
      uart->buf++;
      uart->left--;
    }
#endif
  }
}

rtems_isr console_isr(rtems_vector_number vector)
{
  int i;

  for(i=LEON3_Cpu_Index; i<uarts; i++){
    if ( apbuarts[i].irq+0x10 == vector ){
      console_isr_handler(&apbuarts[i]);
      return;
    }
  }
}

void console_initialize_interrupts(struct apbuart_priv *uart)
{
  /* Install interrupt handler */
  set_vector(console_isr,uart->irq+0x10,1);
}

int console_write_interrupt (int minor, const char *buf, int len)
{
  struct apbuart_priv *uart;
  unsigned int oldLevel;
  
  uart = &apbuarts[minor+LEON3_Cpu_Index];
  
  /* Remember what position in buffer */
#if 0
  uart->buf = buf+1;
  uart->left = len-1;
#endif

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

  minor += LEON3_Cpu_Index;
  
  if ( minor >= uarts )
    return -1;
  
  uart = &apbuarts[minor];

  while (nwrite < len) {
    console_outbyte_polled( uart, *buf++ );
    nwrite++;
  }
  return nwrite;
}

unsigned int console_get_sys_freq(void)
{
  unsigned int freq_hz;
  struct ambapp_apb_info gptimer;
  LEON3_Timer_Regs_Map *tregs;

  /* LEON3: find timer address via AMBA Plug&Play info */	
  if ( ambapp_find_apbslv(&ambapp_plb, VENDOR_GAISLER, GAISLER_GPTIMER, &gptimer) == 1 ){
    tregs = (LEON3_Timer_Regs_Map *)gptimer.start;
    freq_hz = (tregs->scaler_reload+1)*1000*1000;
  } else {
    freq_hz = 40000000; /* Default to 40MHz */
    printk("CONSOLE: Failed to detect system frequency\n\r");
  }

  return freq_hz;
}

int console_set_attributes(int minor, const struct termios *t)
{
  unsigned int core_clk_hz;
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

  minor += LEON3_Cpu_Index;

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
    /* Get APBUART core frequency, it is assumed that it is the same
     * as system frequency 
     */
    core_clk_hz = console_get_sys_freq();
    
    /* Calculate Baud rate generator "scaler" number */
    scaler = (((core_clk_hz*10)/(baud*8))-5)/10;
    
    /* Set new baud rate by setting scaler */
    uart->regs->scaler = scaler;
  }
  
  return 0;
}

/*
 *  Console Device Driver Entry Points
 *
 */

int scan_uarts(void)
{
  int i;
  struct ambapp_apb_info apbuart;
  int found;

  if ( isinit != 0 )
    return uarts;

  memset(apbuarts, 0, sizeof(apbuarts));
  uarts = 0;

  for (i=0; i<CONFIGURE_NUMBER_OF_TERMIOS_PORTS; i++){
    found = ambapp_find_apbslv_next(
      &ambapp_plb, VENDOR_GAISLER, GAISLER_APBUART, &apbuart,
      i + LEON3_UART_INDEX);
    if ( found != 1 ) {
      /* No more APBUART found */
      break;
    }

    uarts++;
    apbuarts[i].regs = (volatile LEON3_UART_Regs_Map *)apbuart.start;
    apbuarts[i].irq = apbuart.irq;
  }

  isinit = 1;

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

  /*
   *  Initialize Hardware if ONLY CPU or first CPU in MP system
   */

  #if defined(RTEMS_MULTIPROCESSING)
    if (rtems_configuration_get_user_multiprocessing_table()->node == 1)
  #endif
  {
    for (i = uart0; i < uarts; i++)
    {
      apbuarts[i].regs->ctrl |=
        LEON_REG_UART_CTRL_RE | LEON_REG_UART_CTRL_TE;
      apbuarts[i].regs->status = 0;
#if CONSOLE_USE_INTERRUPTS
      console_initialize_interrupts(&apbuarts[i]);
#endif
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

  uart = &apbuarts[minor+LEON3_Cpu_Index];
  if ( priv && priv->iop )
    uart->cookie = priv->iop->data1;
  else
    uart->cookie = NULL;

#if CONSOLE_USE_INTERRUPTS
  uart->sending = 0;
  /* Turn on RX interrupts */
  uart->regs->ctrl |= LEON_REG_UART_CTRL_RI;
#endif

  return sc;
}

rtems_device_driver console_close(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
#if CONSOLE_USE_INTERRUPTS
  struct apbuart_priv *uart;
  
  /* Turn off RX interrupts */
  uart = &apbuarts[minor+LEON3_Cpu_Index];
  uart->regs->ctrl &= ~(LEON_REG_UART_CTRL_RI);
  
  /**** Flush device ****/
  while ( uart->sending ){
    /* Wait until all data has been sent */
  }
  
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

/* putchar/getchar for printk */

static void bsp_out_char(char c)
{
  struct apbuart_priv *uart;
  
  if ( uarts == 0 ){
    return;
  }
  
  if ( uarts <= LEON3_Cpu_Index ) {
    uart = &apbuarts[0];
  }else{
    uart = &apbuarts[LEON3_Cpu_Index];
  }
  
  console_outbyte_polled(uart, c);
}

/*
 *  To support printk
 */

BSP_output_char_function_type BSP_output_char = bsp_out_char;

static char bsp_in_char(void)
{
  struct apbuart_priv *uart;
  int tmp;
  
  if ( uarts == 0 ){
    return 0;
  }
  
  if ( uarts <= LEON3_Cpu_Index ) {
    uart = &apbuarts[0];
  }else{
    uart = &apbuarts[LEON3_Cpu_Index];
  }

  while ((tmp = console_inbyte_nonblocking(uart)) < 0);
  return (char) tmp;
}

BSP_polling_getchar_function_type BSP_poll_char = bsp_in_char;

#endif
