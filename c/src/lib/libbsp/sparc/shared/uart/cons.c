/*  This file contains the TTY driver for the serial ports. The driver
 *  is layered so that different UART hardware can be used. It is implemented
 *  using the Driver Manager.
 *
 *  This driver uses the termios pseudo driver.
 *
 *  COPYRIGHT (c) 2010.
 *  Aeroflex Gaisler
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 */

#include <bsp.h>
#include <stdlib.h>
#include <rtems/libio.h>
#include <rtems/bspIo.h>
#include <cons.h>

#ifdef RTEMS_DRVMGR_STARTUP

/* Note that it is not possible to use the interrupt mode of the driver
 * together with the "old" APBUART and -u to GRMON. However the new
 * APBUART core (from 1.0.17-b2710) has the GRMON debug bit and can 
 * handle interrupts.
 */

int console_initialized = 0;
rtems_device_major_number console_major = 0;
struct console_dev *dbgcon = NULL;

/* Before UART driver has registered calls to printk that gets to
 * console_dbg_out_char() will be filling data into the pre_printk_dbgbuf[]
 * buffer, hopefully the buffer can help in debugging the cause. At least
 * the last printk() will be caught.
 */
char pre_printk_dbgbuf[32] = {0};
int pre_printk_pos = 0;

#define FLAG_SYSCON 0x01
#define FLAG_DBGCON 0x02
struct console_priv {
	unsigned char flags; /* 0x1=SystemConsole, 0x2=DBG-Console */
	unsigned char minor;
	struct console_dev *dev;
};

#define CONSOLE_MAX CONFIGURE_NUMBER_OF_TERMIOS_PORTS
struct console_priv cons[CONSOLE_MAX] = {{0,0},};

static void console_dbg_init(void)
{
	if ( dbgcon && dbgcon->dbgops->init )
		dbgcon->dbgops->init(dbgcon);
}

static void console_dbg_out_char(char c)
{
	if ( !dbgcon || !dbgcon->dbgops->out_char ) {
		/* Local debug buffer when UART driver has not registered */
		pre_printk_dbgbuf[pre_printk_pos++] = c;
		pre_printk_pos = pre_printk_pos & (sizeof(pre_printk_dbgbuf)-1);
		return;
	}

	dbgcon->dbgops->out_char(dbgcon, c);
}

static char console_dbg_in_char(void)
{
	if ( !dbgcon || !dbgcon->dbgops->in_char )
		return 0;
	return dbgcon->dbgops->in_char(dbgcon);
}

/* Register Console to TERMIOS layer and initialize it */
void console_dev_init(struct console_priv *con, int minor)
{
	char name[32];
	char *namestr, *fsname;
	rtems_status_code status;

	if ( !con->dev->fsname ) {
		if ( minor == 0 ) {
			/* Special console name and MINOR for SYSTEM CONSOLE */
			namestr = "/dev/console";
		} else {
			namestr = "/dev/console_a";
		}
		strcpy(name, namestr);
		name[13] += minor; /* when minor=0, this has no effect... */
		fsname = name;
	} else {
		fsname = con->dev->fsname;
	}
	status = rtems_io_register_name(fsname, console_major, minor);
	if ( (minor == 0) && (status != RTEMS_SUCCESSFUL) ) {
		rtems_fatal_error_occurred(status);
	}
}

void console_dev_register(struct console_dev *dev)
{
	int i, minor;
	struct console_priv *con;

	if ( (dev->flags & CONSOLE_FLAG_SYSCON) && !cons[0].dev ) {
		con = &cons[0];
		con->flags = FLAG_SYSCON;
		minor = 0;
	} else {
		for (i=1; i<CONSOLE_MAX; i++) {
			if ( !cons[i].dev ) {
				con = &cons[i];
				con->flags = 0;
				minor = i;
				break;
			}
		}
	}
	if ( con == NULL ) {
		/* Not enough console structures */
		return;
	}

	/* Assign Console */
	con->dev = dev;
	con->minor = minor;

	/* Console layer is already initialized, that means that we can
	 * register termios interface directly.
	 */
	if ( console_initialized ) {
		if ( (dev->flags & CONSOLE_FLAG_DBGCON) && !dbgcon) {
			dbgcon = dev;
			con->flags |= FLAG_DBGCON;
			console_dbg_init();
		}
		console_dev_init(con, minor);
	} else {
		if ( dev->flags & CONSOLE_FLAG_DBGCON ) {
			/* Assign Debug Console */
			dbgcon = dev;
		}
	}
}

#if 0
void console_dev_unregister(struct console_dev *dev)
{

}
#endif

rtems_device_driver console_initialize(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	int i;

	console_major = major;

	rtems_termios_initialize();

	/* Initialize Debug Console */
	/*con->flags |= FLAG_DBGCON;*/
	console_dbg_init();

	/* Register all Console a file system device node */
	for (i=0; i<CONSOLE_MAX; i++) {
		if ( cons[i].dev ) {
			console_dev_init(&cons[i], i);
		}
	}

	console_initialized = 1;

	return RTEMS_SUCCESSFUL;
}

rtems_device_driver console_open(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	if ( (minor >= CONSOLE_MAX) || !cons[minor].dev )
		return RTEMS_INVALID_NUMBER;

	return rtems_termios_open(
			major,
			(int)cons[minor].dev,
			arg,
			cons[minor].dev->callbacks);
}

rtems_device_driver console_close(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	return rtems_termios_close(arg);
}

rtems_device_driver console_read(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	return rtems_termios_read(arg);
}

rtems_device_driver console_write(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	return rtems_termios_write(arg);
}

rtems_device_driver console_control(
	rtems_device_major_number	major,
	rtems_device_minor_number	minor,
	void				*arg)
{
	return rtems_termios_ioctl(arg);
}

/* printk() support functions, will call DBG-console device functions */
BSP_output_char_function_type BSP_output_char = console_dbg_out_char;
BSP_polling_getchar_function_type BSP_poll_char = console_dbg_in_char;

#endif
