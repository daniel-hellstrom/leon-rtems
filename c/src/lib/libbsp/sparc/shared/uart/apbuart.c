/* Unused driver at the moment since we can not have two drivers for
 * the same hardware...
 */
#if 0

/*  This file contains the driver for the GRLIB APBUART serial port.
 *  No console driver, only char driver.
 *
 *  COPYRIGHT (c) 2008.
 *  Aeroflex Gaisler.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  2008-12-03, Daniel Hellstrom <daniel@gaisler.com>
 *   Converted to support driver manager.
 * 
 *  2007-07-11, Daniel Hellstrom <daniel@gaisler.com>
 *    Added ioctl command APBUART_CLR_STATS
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

/*#define DEBUG 1  */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif

typedef struct {
	int		size;
	unsigned char	*buf,
			*tail,
			*head,
			*max;
	int		full;	/* no more place in fifo */
} apbuart_fifo;

struct apbuart_priv {
	struct rtems_drvmgr_dev_info	*dev;
	ambapp_apb_uart		*regs;
	int			irq;
	int			minor;
	int			scaler;
	unsigned int		baud;
	unsigned int		freq_hz; /* UART Core Frequency */

	int			txblk;	/* Make write block until at least 1 char has 
					 * been put into software send fifo
					 */
	int			tx_flush;	/* Set this to block until all data has 
						 * placed into the hardware send fifo 
						 */
	int			rxblk;	/* Make read block until at least 1 char has
					 * been received (or taken from software fifo).
					 */
	int			started;	/* Set to 1 when in running mode */
	
	int			ascii_mode;	/* Set to 1 to make \n be printed as \r\n */

	/* TX/RX software FIFO Buffers */
	apbuart_fifo		*txfifo;
	apbuart_fifo		*rxfifo;

	apbuart_stats		stats;

	rtems_id		dev_sem;
	rtems_id		rx_sem;
	rtems_id		tx_sem;
};

/* Driver prototypes */
int apbuart_register_io(rtems_device_major_number *m);
int apbuart_device_init(struct apbuart_priv *priv);

int apbuart_init2(struct rtems_drvmgr_dev_info *dev);
int apbuart_init3(struct rtems_drvmgr_dev_info *dev);

struct rtems_drvmgr_drv_ops apbuart_ops = 
{
	.init = {NULL, apbuart_init2, apbuart_init3, NULL},
	.remove = NULL,
	.info = NULL
};

struct amba_dev_id apbuart_ids[] = 
{
	{VENDOR_GAISLER, GAISLER_APBUART},
	{0, 0}		/* Mark end of table */
};

struct amba_drv_info apbuart_drv_info =
{
	{
		DRVMGR_OBJ_DRV,				/* Driver */
		NULL,					/* Next driver */
		NULL,					/* Device list */
		DRIVER_AMBAPP_GAISLER_APBUART_ID,	/* Driver ID */
		"APBUART_DRV",				/* Driver Name */
		DRVMGR_BUS_TYPE_AMBAPP,			/* Bus Type */
		&apbuart_ops,
		0,					/* No devices yet */
	},
	&apbuart_ids[0]
};

static int apbuart_driver_io_registered = 0;
static rtems_device_major_number apbuart_driver_io_major = 0;

void apbuart_register_drv (void)
{
	DBG("Registering APBUART driver\n");
	rtems_drvmgr_drv_register(&apbuart_drv_info.general);
}

int apbuart_init2(struct rtems_drvmgr_dev_info *dev)
{
	struct apbuart_priv *priv;

	DBG("APBUART[%d] on bus %s\n", dev->minor_drv, dev->parent->dev->name);
	if ( strcmp(dev->parent->dev->drv->name, "AMBAPP_GRLIB_DRV") == 0 ) {
		/* Let standard console driver take care of APBUART
		 * for CPU-local APBUART cores.
		 */
		dev->priv = NULL;
		DBG("-- SKIPPING APBUART 1 --\n");
		return DRVMGR_FAIL;
	}
	priv = dev->priv = malloc(sizeof(struct apbuart_priv));
	if ( !priv )
		return DRVMGR_NOMEM;
	memset(priv, 0, sizeof(*priv));
	priv->dev = dev;

	/* This core will not find other cores, so we wait for init2() */

	return DRVMGR_OK;
}

int apbuart_init3(struct rtems_drvmgr_dev_info *dev)
{
	struct apbuart_priv *priv;
	char prefix[16];
	char devName[32];
	rtems_status_code status;

	priv = dev->priv;

	/* Do initialization */

	if ( apbuart_driver_io_registered == 0) {
		/* Register the I/O driver only once for all cores */
		if ( apbuart_register_io(&apbuart_driver_io_major) ) {
			/* Failed to register I/O driver */
			dev->priv = NULL;
			return DRVMGR_FAIL;
		}

		apbuart_driver_io_registered = 1;
	}

	/* I/O system registered and initialized 
	 * Now we take care of device initialization.
	 */

	/* Get frequency */
	if ( rtems_drvmgr_freq_get(dev, DEV_APB_SLV, &priv->freq_hz) ) {
		return DRVMGR_FAIL;
	}

	if ( apbuart_device_init(priv) ) {
		return DRVMGR_FAIL;
	}

	/* Get Filesystem name prefix */
	prefix[0] = '\0';
	if ( rtems_drvmgr_get_dev_prefix(dev, prefix) ) {
		/* Failed to get prefix, make sure of a unique FS name
		 * by using the driver minor.
		 */
		sprintf(devName, "/dev/apbuart%d", dev->minor_drv);
	} else {
		/* Got special prefix, this means we have a bus prefix
		 * And we should use our "bus minor"
		 */
		sprintf(devName, "/dev/%sapbuart%d", prefix, dev->minor_bus);
	}

	/* Register Device */
	status = rtems_io_register_name(devName, apbuart_driver_io_major, dev->minor_drv);
	if (status != RTEMS_SUCCESSFUL) {
		return DRVMGR_FAIL;
	}

	return DRVMGR_OK;
}

/******************* Driver Implementation ***********************/

#ifndef DEFAULT_TXBUF_SIZE
 #define DEFAULT_TXBUF_SIZE 32
#endif
#ifndef DEFAULT_RXBUF_SIZE
 #define DEFAULT_RXBUF_SIZE 32
#endif

/* Uncomment for debug output */
/* #define DEBUG 1 
 #define FUNCDEBUG 1 */

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) 
#endif
#ifdef FUNCDEBUG
#define FUNCDBG(x...) printk(x)
#else
#define FUNCDBG(x...) 
#endif

#ifndef READ_REG
	#define READ_REG(address) _APBUART_READ_REG((unsigned int)(address))
  static __inline__ unsigned int _APBUART_READ_REG(unsigned int addr) {
        unsigned int tmp;
        asm(" lda [%1]1, %0 "
            : "=r"(tmp)
            : "r"(addr)
           );
        return tmp;
	}
#endif

static apbuart_fifo *apbuart_fifo_create(int size);
static void apbuart_fifo_free(apbuart_fifo *fifo);
static inline int apbuart_fifo_isFull(apbuart_fifo *fifo);
static inline int apbuart_fifo_isEmpty(apbuart_fifo *fifo);
static int apbuart_fifo_put(apbuart_fifo *fifo, unsigned char c);
static int apbuart_fifo_get(apbuart_fifo *fifo, unsigned char *c);
static int inline apbuart_fifo_peek(apbuart_fifo *fifo, unsigned char **c);
static void inline apbuart_fifo_skip(apbuart_fifo *fifo);

static rtems_device_driver apbuart_initialize(rtems_device_major_number  major, rtems_device_minor_number  minor,  void *arg);
static rtems_device_driver apbuart_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver apbuart_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver apbuart_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver apbuart_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);
static rtems_device_driver apbuart_control(rtems_device_major_number major, rtems_device_minor_number minor, void *arg);

static void apbuart_interrupt(int irq, void *arg);

#define APBUART_DRIVER_TABLE_ENTRY { apbuart_initialize, apbuart_open, apbuart_close, apbuart_read, apbuart_write, apbuart_control }
static rtems_driver_address_table apbuart_driver = APBUART_DRIVER_TABLE_ENTRY;

int apbuart_register_io(rtems_device_major_number *m)
{
	rtems_status_code r;

	if ((r = rtems_io_register_driver(0, &apbuart_driver, m)) == RTEMS_SUCCESSFUL) {
		DBG("APBUART driver successfully registered, major: %d\n", *m);
	} else {
		switch(r) {
		case RTEMS_TOO_MANY:
			printk("APBUART rtems_io_register_driver failed: RTEMS_TOO_MANY\n");
			return -1;
		case RTEMS_INVALID_NUMBER:  
			printk("APBUART rtems_io_register_driver failed: RTEMS_INVALID_NUMBER\n");
			return -1;
		case RTEMS_RESOURCE_IN_USE:
			printk("APBUART rtems_io_register_driver failed: RTEMS_RESOURCE_IN_USE\n");
			return -1;
		default:
			printk("APBUART rtems_io_register_driver failed\n");
			return -1;
		}
	}
	return 0;
}

int apbuart_device_init(struct apbuart_priv *priv)
{
	struct amba_dev_info *ambadev;
	struct ambapp_core *pnpinfo;
	int minor = priv->dev->minor_drv;
	int rxFifoLen, txFifoLen;
	union rtems_drvmgr_key_value *value;

	/* Get device information from AMBA PnP information */
	ambadev = (struct amba_dev_info *)priv->dev->businfo;
	if ( ambadev == NULL ) {
		return -1;
	}
	pnpinfo = &ambadev->info;
	priv->irq = pnpinfo->irq;
	priv->regs = (ambapp_apb_uart *)pnpinfo->apb_slv->start;

	/* Clear HW regs */
	priv->regs->status = 0;
	priv->regs->ctrl = 0;

	/* Get Configuration from Bus resources (Let user override defaults) */
	rxFifoLen = DEFAULT_RXBUF_SIZE;
	txFifoLen = DEFAULT_TXBUF_SIZE;

	value = rtems_drvmgr_dev_key_get(priv->dev, "rxFifoLen", KEY_TYPE_INT);
	if ( value )
		rxFifoLen = value->i;

	value = rtems_drvmgr_dev_key_get(priv->dev, "txFifoLen", KEY_TYPE_INT);
	if ( value )
		txFifoLen = value->i;

	/* Allocate default software buffers */
	priv->txfifo = apbuart_fifo_create(txFifoLen);
	priv->rxfifo = apbuart_fifo_create(rxFifoLen);
	if ( !priv->txfifo || !priv->rxfifo )
		return -1;

	/* Setup interrupt handler */
	if ( rtems_drvmgr_interrupt_register(priv->dev, 0, apbuart_interrupt, priv) ) {
		return -1;
	}

	/* Device A Semaphore created with count = 1 */
	if ( rtems_semaphore_create(rtems_build_name('A', 'U', 'D', '0'+minor),
	     1,
	     RTEMS_FIFO|RTEMS_SIMPLE_BINARY_SEMAPHORE|RTEMS_NO_INHERIT_PRIORITY|RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING, 
	     0,
	     &priv->dev_sem) != RTEMS_SUCCESSFUL ){
		return -1;
	}

	if ( rtems_semaphore_create(rtems_build_name('A', 'U', 'T', '0'+minor),
	     1,
	     RTEMS_FIFO|RTEMS_SIMPLE_BINARY_SEMAPHORE|RTEMS_NO_INHERIT_PRIORITY|RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING, 
	     0,
	     &priv->tx_sem) != RTEMS_SUCCESSFUL ) {
		return -1;
	}

	if ( rtems_semaphore_create(rtems_build_name('A', 'U', 'R', '0'+minor),
	     1,
	     RTEMS_FIFO|RTEMS_SIMPLE_BINARY_SEMAPHORE|RTEMS_NO_INHERIT_PRIORITY|RTEMS_LOCAL|RTEMS_NO_PRIORITY_CEILING, 
	     0,
	     &priv->rx_sem) != RTEMS_SUCCESSFUL ) {
		return -1;
	}

	return 0;
}

/******************* I/O driver implementation ***********************/

static void apbuart_hw_open(struct apbuart_priv *uart)
{
	unsigned int scaler;
	
	/* Calculate Baudrate */
	if ( uart->scaler > 0 ) {
		scaler = uart->scaler;
	} else {
		scaler = (((uart->freq_hz*10)/(uart->baud*8))-5)/10;
		uart->scaler = scaler;
	}
	
	/* Set new baud rate */
	uart->regs->scaler = scaler;
	
	/* Enable receiver & Transmitter */
	uart->regs->ctrl = APBUART_CTRL_RE | APBUART_CTRL_RF | APBUART_CTRL_RI | APBUART_CTRL_TI;
}

static void apbuart_hw_close(struct apbuart_priv *uart)
{
	/* disable receiver & transmitter & all IRQs */
	uart->regs->ctrl = 0;
}

static rtems_device_driver apbuart_initialize(rtems_device_major_number  major, rtems_device_minor_number  minor,  void *arg)
{
	/* Initialize common data structures, for example common semaphores... */

	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver apbuart_open(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{  
	struct apbuart_priv *uart;
	struct rtems_drvmgr_dev_info *dev;

	if ( rtems_drvmgr_get_dev(&apbuart_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NAME;
	}
	uart = (struct apbuart_priv *)dev->priv;

	FUNCDBG("apbuart_open: major %d, minor %d\n", major, minor);

	if (rtems_semaphore_obtain(uart->dev_sem, RTEMS_NO_WAIT, RTEMS_NO_TIMEOUT) != RTEMS_SUCCESSFUL) {
		DBG("apbuart_open: resource in use\n");
		return RTEMS_RESOURCE_IN_USE;
	}

	/* Clear HW regs */
	uart->regs->status = 0;
	uart->regs->ctrl = 0;

	/* Set Defaults */

	/* 38400 baudrate */
	uart->scaler = 0; /* use uart->baud */
	uart->baud = 38400;

	/* Default to Blocking mode */
	uart->txblk = 1;
	uart->rxblk = 1;

	/* Default to no flush mode */
	uart->tx_flush = 0;

	/* non-ascii mode */
	uart->ascii_mode = 0;

	/* not started */
	uart->started = 0;

	if ( !uart->txfifo || (uart->txfifo->size!=DEFAULT_TXBUF_SIZE) ){
		apbuart_fifo_free(uart->txfifo);
		uart->txfifo = apbuart_fifo_create(DEFAULT_TXBUF_SIZE);
	}

	if ( !uart->rxfifo || (uart->rxfifo->size!=DEFAULT_RXBUF_SIZE) ){
		apbuart_fifo_free(uart->rxfifo);
		uart->rxfifo = apbuart_fifo_create(DEFAULT_RXBUF_SIZE);
	}

	if ( !uart->rxfifo || !uart->txfifo ){
		/* Failed to get memory */
		return RTEMS_NO_MEMORY;
	}

	/* Now user must call ioctl(START,0) to begin */
	
	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver apbuart_close(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	struct apbuart_priv *uart;
	struct rtems_drvmgr_dev_info *dev;

	if ( rtems_drvmgr_get_dev(&apbuart_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NAME;
	}
	uart = (struct apbuart_priv *)dev->priv;

	FUNCDBG("apbuart_close[%d]:\n",minor);
	
	apbuart_hw_close(uart);

	/* Software state will be set when open is called again */
	rtems_semaphore_release(uart->rx_sem);
	rtems_semaphore_release(uart->tx_sem);
	uart->started = 0;
	
	rtems_semaphore_release(uart->dev_sem);
	
	return RTEMS_SUCCESSFUL;
}
 
static rtems_device_driver apbuart_read(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	rtems_libio_rw_args_t *rw_args;
	unsigned int count = 0, oldLevel;
	unsigned char *buf;
	struct apbuart_priv *uart;
	struct rtems_drvmgr_dev_info *dev;

	if ( rtems_drvmgr_get_dev(&apbuart_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NAME;
	}
	uart = (struct apbuart_priv *)dev->priv;
	
	rw_args = (rtems_libio_rw_args_t *) arg;

	FUNCDBG("apbuart_read\n");
	
	buf = (unsigned char *)rw_args->buffer;
	if ( (rw_args->count < 1) || !buf )
		return RTEMS_INVALID_NAME; /* EINVAL */

	rtems_interrupt_disable(oldLevel);
	do {
		if ( (unsigned int)uart < 0x40000000 ) {
			printk("UART %x is screwed\n",uart);
		}
		/* Read from SW fifo */
		if ( apbuart_fifo_get(uart->rxfifo,&buf[count]) != 0 ){			
			/* non blocking or read at least 1 byte */
			if ( (count > 0) || (!uart->rxblk) )
				break; /* Return */
			
			rtems_interrupt_enable(oldLevel);

			/* Block thread until a char is received */
			rtems_semaphore_obtain(uart->rx_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
      
			rtems_interrupt_disable(oldLevel);
			continue;
		}
		
		/* Got char from SW FIFO */
		count++;
		
	} while (count < rw_args->count );
	
	rtems_interrupt_enable(oldLevel);

	rw_args->bytes_moved = count;
	
	if (count == 0)
		return RTEMS_TIMEOUT; /* ETIMEDOUT should be EAGAIN/EWOULDBLOCK */

	return RTEMS_SUCCESSFUL;
}
 
static rtems_device_driver apbuart_write(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	rtems_libio_rw_args_t *rw_args;
	unsigned int count, oldLevel, ctrl;
	char *buf;
	int direct=0;
	struct apbuart_priv *uart;
	struct rtems_drvmgr_dev_info *dev;

	if ( rtems_drvmgr_get_dev(&apbuart_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NAME;
	}
	uart = (struct apbuart_priv *)dev->priv;
	
	rw_args = (rtems_libio_rw_args_t *) arg;
  
	FUNCDBG("apbuart_write\n");
	
	buf = rw_args->buffer;
	
	if ( rw_args->count < 1 || !buf )
		return RTEMS_INVALID_NAME; /* EINVAL */ 
	
	count = 0;
	rtems_interrupt_disable(oldLevel);
	/* Do we need to start to send first char direct via HW
	 * to get IRQ going.
	 */
	
	ctrl = READ_REG(&uart->regs->ctrl);
	if ( (ctrl & APBUART_CTRL_TF) == 0 ){
		/* TX interrupt is disabled ==> 
		 * SW FIFO is empty and,
		 * HW FIFO empty
		 */
		uart->regs->ctrl = ctrl | APBUART_CTRL_TF;
		if ( uart->ascii_mode && (buf[0] == '\n') ){
			uart->regs->data = '\r';
		}else{
			uart->regs->data = buf[0];
			count++;
		}
		uart->regs->ctrl = ctrl | APBUART_CTRL_TE | APBUART_CTRL_TF;
		direct = 1;
	}
	
	while( count < rw_args->count ) {
		/* write to HW FIFO direct skipping SW FIFO */
		if ( direct && ((READ_REG(&uart->regs->status) & APBUART_STATUS_TF) == 0) ){
			uart->regs->data = buf[count];
		}
		/* write to SW FIFO */
		else if ( apbuart_fifo_put(uart->txfifo,buf[count]) ){
			direct = 0;
			DBG("APBUART[%d]: write: SW FIFO Full\n\r",minor);
			
			/* is full, block? */
			if ( ((count < 1) && uart->txblk) || uart->tx_flush ){
			
				rtems_interrupt_enable(oldLevel);
				
				rtems_semaphore_obtain(uart->tx_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
				
				rtems_interrupt_disable(oldLevel);
			
				/* Do we need to start to send first char direct via HW
				 * to get IRQ going.
				 */
	
				ctrl = READ_REG(&uart->regs->ctrl);
				if ( (ctrl & APBUART_CTRL_TF) == 0 ){
					/* TX interrupt is disabled ==> 
					 * SW FIFO is empty and,
					 * HW FIFO empty
					 */
					uart->regs->ctrl = ctrl | APBUART_CTRL_TF;
					if ( uart->ascii_mode && (buf[count] == '\n') ){
						uart->regs->data = '\r';
					}else{
						uart->regs->data = buf[count];
						count++;
					}
					uart->regs->ctrl = ctrl | APBUART_CTRL_TF | APBUART_CTRL_TE;
					direct = 1;
				}
				
				continue;
			}
			/* don't block, return current status */
			break;
		}else{
			direct = 0;
		}
		
		count++;
		
	}

	rtems_interrupt_enable(oldLevel);
	
	rw_args->bytes_moved = count;
	
	if (count == 0)
		return RTEMS_TIMEOUT; /* ETIMEDOUT should be EAGAIN/EWOULDBLOCK */
 
	return RTEMS_SUCCESSFUL;
}

static rtems_device_driver apbuart_control(rtems_device_major_number major, rtems_device_minor_number minor, void *arg)
{
	rtems_libio_ioctl_args_t *ioarg = (rtems_libio_ioctl_args_t *)arg;
	unsigned int *data = ioarg->buffer;
	int size;
	unsigned int baudrate, blocking;
	apbuart_stats *stats;
	struct apbuart_priv *uart;
	struct rtems_drvmgr_dev_info *dev;

	if ( rtems_drvmgr_get_dev(&apbuart_drv_info.general, minor, &dev) ) {
		DBG("Wrong minor %d\n", minor);
		return RTEMS_INVALID_NAME;
	}
	uart = (struct apbuart_priv *)dev->priv;

	FUNCDBG("apbuart_control [%i,%i]\n",major, minor);
  
	if (!ioarg)
		return RTEMS_INVALID_NAME;

	ioarg->ioctl_return = 0;
	switch(ioarg->command) {
	
	/* Enable Receiver & transmitter */
	case APBUART_START:
		if ( uart->started )
			return RTEMS_INVALID_NAME;
		apbuart_hw_open(uart);
		uart->started = 1;
		rtems_drvmgr_interrupt_enable(dev, 0, apbuart_interrupt, uart);
		break;
	
	/* Close Receiver & transmitter */
	case APBUART_STOP:
		if ( !uart->started )
			return RTEMS_INVALID_NAME;
		rtems_drvmgr_interrupt_disable(dev, 0, apbuart_interrupt, uart);
		apbuart_hw_close(uart);
		uart->started = 0;
		break;
	
	/* Set RX FIFO Software buffer length 
	 * It is only possible to change buffer size in
	 * non-running mode.
	 */
	case APBUART_SET_RXFIFO_LEN:
		if ( uart->started )
			return RTEMS_RESOURCE_IN_USE; /* EBUSY */
	
		size = (int)ioarg->buffer;
		if ( size < 1 ) 
			return RTEMS_INVALID_NAME; /* EINVAL */
		
		/* Free old buffer */
		apbuart_fifo_free(uart->rxfifo);
		
		/* Allocate new buffer & init it */
		uart->rxfifo = apbuart_fifo_create(size);
		if ( !uart->rxfifo )
			return RTEMS_NO_MEMORY;
		break;

	/* Set TX FIFO Software buffer length 
	 * It is only possible to change buffer size 
	 * while in non-running mode.
	 */
	case APBUART_SET_TXFIFO_LEN:
		if ( uart->started )
			return RTEMS_RESOURCE_IN_USE; /* EBUSY */
		
		size = (int)ioarg->buffer;
		if ( size < 1 ) 
			return RTEMS_INVALID_NAME; /* EINVAL */
		
		/* Free old buffer */
		apbuart_fifo_free(uart->txfifo);
		
		/* Allocate new buffer & init it */
		uart->txfifo = apbuart_fifo_create(size);
		if ( !uart->txfifo )
			return RTEMS_NO_MEMORY;
		break;
	
	case APBUART_SET_BAUDRATE:
		/* Set baud rate of */
		baudrate = (int)ioarg->buffer;
		if ( (baudrate < 1) || (baudrate > 115200) ){
			return RTEMS_INVALID_NAME;
		}
		uart->scaler = 0; /* use uart->baud */
		uart->baud = baudrate;
		break;
	
	case APBUART_SET_SCALER:
		/* use uart->scaler not uart->baud */
		uart->scaler = data[0];
		break;
	
	case APBUART_SET_BLOCKING:
		blocking = (unsigned int)ioarg->buffer;
		uart->rxblk = ( blocking & APBUART_BLK_RX );
		uart->txblk = ( blocking & APBUART_BLK_TX );
		uart->tx_flush = ( blocking & APBUART_BLK_FLUSH );
		break;
	
	case APBUART_GET_STATS:
		stats = (void *)ioarg->buffer;
		if ( !stats )
			return RTEMS_INVALID_NAME;
		
		/* Copy Stats */
		*stats = uart->stats;
		break;
  
	case APBUART_CLR_STATS:
		/* Clear/reset Stats */
		memset(&uart->stats,0,sizeof(uart->stats));
		break;
    
	case APBUART_SET_ASCII_MODE:
		uart->ascii_mode = (int)ioarg->buffer;
		break;
   
	default:
		return RTEMS_NOT_DEFINED;
	}
	return RTEMS_SUCCESSFUL;
}

/* The interrupt handler, taking care of the 
 * APBUART hardware
 */
static void apbuart_interrupt(int irq, void *arg)
{
	struct apbuart_priv *uart = (struct apbuart_priv *)arg;
	unsigned int status;
	int empty;
	unsigned char c, *next_char = NULL;
	int signal;
	
	/* Clear & record any error */
	status = READ_REG(&uart->regs->status);
	if ( status & (APBUART_STATUS_OV|APBUART_STATUS_PE|APBUART_STATUS_FE) ){
		/* Data overrun */
		if ( status & APBUART_STATUS_OV ){
			uart->stats.hw_dovr++;
		}
		/* Parity error */
		if ( status & APBUART_STATUS_PE ){
			uart->stats.hw_parity++;
		}
		/* Framing error */
		if ( status & APBUART_STATUS_FE ){
			uart->stats.hw_frame++;
		}
		uart->regs->status = status & ~(APBUART_STATUS_OV|APBUART_STATUS_PE|APBUART_STATUS_FE);
	}
	
	/* Empty RX fifo into software fifo */
	signal = 0;
	while ( (status=READ_REG(&uart->regs->status)) & APBUART_STATUS_DR ){
		c = READ_REG(&uart->regs->data);
		if ( apbuart_fifo_isFull(uart->rxfifo) ){
			uart->stats.sw_dovr++;
			DBG("]");
			break;
		}
		/* put into fifo */
		apbuart_fifo_put(uart->rxfifo,c);
    
		/* bump RX counter */
		uart->stats.rx_cnt++;
    
		signal = 1;
	}
	
	/* Wake RX thread if any */
	if ( signal )
		rtems_semaphore_release(uart->rx_sem);
	
	/* If room in HW fifo and we got more chars to be sent */
	if ( !(status & APBUART_STATUS_TF) ){
		
		if ( apbuart_fifo_isEmpty(uart->txfifo) ){
			/* Turn off TX interrupt when no data is to be sent */
			if ( status & APBUART_STATUS_TE ){
				uart->regs->ctrl = READ_REG(&uart->regs->ctrl) & ~APBUART_CTRL_TF;
				DBG("?");
			}
			return;
		}
		
		/* signal when there will be more room in SW fifo */
		if ( apbuart_fifo_isFull(uart->txfifo) )
			signal = 1;
		
		do{
			/* Put data into HW TX fifo */
			apbuart_fifo_peek(uart->txfifo,&next_char);
			c = *next_char;
			if ( uart->ascii_mode && ( c == '\n') ){
				uart->regs->data = '\n';
				*next_char = '\r'; /* avoid sending mutiple '\n' or '\r' */
			}else{
				uart->regs->data = c;
				apbuart_fifo_skip(uart->txfifo); /* remove sent char from fifo */
			}
			uart->regs->ctrl = READ_REG(&uart->regs->ctrl) | APBUART_CTRL_TE | APBUART_CTRL_TF;
			DBG("!");
		}while(!(empty=apbuart_fifo_isEmpty(uart->txfifo)) && 
		       !((status=READ_REG(&uart->regs->status))&APBUART_STATUS_TF) );
		
		/* Wake userspace thread, on empty or full fifo
		 * This makes tx_flush and block work.
		 */
		if ( signal || empty ){
			rtems_semaphore_release(uart->tx_sem);
		}
	}
}

/******************* APBUART FIFO implementation ***********************/

static apbuart_fifo *apbuart_fifo_create(int size)
{
	apbuart_fifo *fifo;
	fifo = (apbuart_fifo *) malloc(size + sizeof(apbuart_fifo));
	if ( fifo ) {
		/* Init fifo */
		fifo->size = size;
		fifo->buf = (unsigned char *)(fifo+1);
		fifo->tail = fifo->buf;
		fifo->head = fifo->buf;
		fifo->max = &fifo->buf[size-1];
		fifo->full=0;
	}
	return fifo;
}

static void apbuart_fifo_free(apbuart_fifo *fifo)
{
	if ( fifo )
		free(fifo);
}

static inline int apbuart_fifo_isFull(apbuart_fifo *fifo)
{
	return fifo->full;
}

static inline int apbuart_fifo_isEmpty(apbuart_fifo *fifo)
{
	if ( (fifo->head == fifo->tail) && !fifo->full )
		return -1;
	return 0;
}

static int apbuart_fifo_put(apbuart_fifo *fifo, unsigned char c)
{
	if ( !fifo->full ){
		*fifo->head = c;
		fifo->head = (fifo->head >= fifo->max ) ? fifo->buf : fifo->head+1;
		if ( fifo->head == fifo->tail )
			fifo->full = -1;
		return 0;
	}
	return -1;
}

static int apbuart_fifo_get(apbuart_fifo *fifo, unsigned char *c)
{
	if ( apbuart_fifo_isEmpty(fifo) )
		return -1;
	if ( c )
		*c = *fifo->tail;
	fifo->tail = (fifo->tail >= fifo->max ) ? fifo->buf : fifo->tail+1;
	fifo->full = 0;
	return 0;
}

static int inline apbuart_fifo_peek(apbuart_fifo *fifo, unsigned char **c)
{
	if ( apbuart_fifo_isEmpty(fifo) )
		return -1;
	if ( c )
		*c = fifo->tail;
	return 0;
}

static void inline apbuart_fifo_skip(apbuart_fifo *fifo)
{
	if ( !apbuart_fifo_isEmpty(fifo) ){
		fifo->tail = (fifo->tail >= fifo->max ) ? fifo->buf : fifo->tail+1;
		fifo->full = 0;
	}
}
#endif
