/*
 *
 * - First console device that has System Console flag set will be
 *   system console.
 * - Last console device that has Debug Console flag set will be
 *   debug console.
 * - If none of the registered console devices has system console set,
 *   the first is registered device is used, unless it has 
 */

#ifndef __CONS_H__
#define __CONS_H__

struct console_dev;

#define CONSOLE_FLAG_SYSCON      0x01
#define CONSOLE_FLAG_DBGCON      0x02

struct console_dbg_ops {
	void (*init)(struct console_dev *);
	char (*in_char)(struct console_dev *);
	void (*out_char)(struct console_dev *, char c);
};

struct console_dev {
	/* Set to non-zero if this UART should be system console and/or
	 * debug console.
	 */
	int flags;
	struct console_dbg_ops *dbgops;
	char *fsname; /* File system prefix */
	const struct rtems_termios_callbacks *callbacks; /* TERMIOS Callbacks */
};

extern void console_dev_register(struct console_dev *dev);
#if 0
extern void console_dev_unregister(struct console_dev *dev);
#endif

#endif
