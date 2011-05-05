/*
 *  DRVMGR Command Implmentation
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include <drvmgr/drvmgr.h>

#include <rtems.h>
#include <rtems/shell.h>
#include "internal.h"

/* General info, root bus, number of devices etc. */
void show_drvmgr_info(void)
{

}

int shell_drvmgr_topo(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_short(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_info(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_remove(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_parent(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_drv(int argc, char *argv[])
{
  return 0;
}

int shell_drvmgr_mem(int argc, char *argv[])
{
  rtems_drvmgr_print_mem();
  return 0;
}

const char drvmgr_usage_str[] =
 " usage:\n"
 "  drvmgr topo              Show bus topology with all devices\n"
 "  drvmgr short ID          Short info about all devices/buses or one\n"
 "                           device/bus\n"
 "  drvmgr info ID [arg]     List general and driver specfic information\n"
 "                           about a device or bus\n"
 "  drvmgr remove ID         Remove a device or a bus\n"
 "  drvmgr parent ID         Short info about parent bus of a device\n"
 "  drvmgr drv [ID]          Information about one or all drivers\n"
 "  drvmgr mem               Dynamically memory usage\n"
 "  drvmgr --help\n";

int shell_drvmgr_usage(int argc, char *argv[])
{
  return 0;
}

struct shell_drvmgr_modifier {
  char *name;
  int (*func)(int argc, char *argv[]);
};

#define MODIFIER_NUM 8
static struct shell_drvmgr_modifier shell_drvmgr_modifiers[MODIFIER_NUM] =
{
  {"topo", shell_drvmgr_topo},
  {"short", shell_drvmgr_short},
  {"info", shell_drvmgr_info},
  {"remove", shell_drvmgr_remove},
  {"parent", shell_drvmgr_parent},
  {"drv", shell_drvmgr_drv},
  {"mem", shell_drvmgr_mem},
  {"--help", shell_drvmgr_usage},
};

struct shell_drvmgr_modifier *shell_drvmgr_find_modifier(char *name)
{
  struct shell_drvmgr_modifier *mod;
  int i;

  for (i=0, mod=&shell_drvmgr_modifiers[0]; i<MODIFIER_NUM; i++, mod++) {
    if (strcmp(name, mod->name) == 0)
      return mod;
  }

  return NULL;
}

int rtems_shell_main_drvmgr(
  int   argc,
  char *argv[]
)
{
  struct shell_drvmgr_modifier *mod;
  int rc;

  if (argc < 2) {
    show_drvmgr_info();
    rc = 0;
  } else if ((mod=shell_drvmgr_find_modifier(argv[1])) != NULL) {
    rc = mod->func(argc, argv);
  } else {
    rc = -1;
  }

  if (rc != 0) {
    printf("invalid argument\n");
    shell_drvmgr_usage(0,0);
  }

  return rc;
}

rtems_shell_cmd_t rtems_shell_DRVMGR_Command = {
  "drvmgr",                      /* name */
  drvmgr_usage_str,              /* usage */
  "system",                      /* topic */
  rtems_shell_main_drvmgr,       /* command */
  NULL,                          /* alias */
  NULL                           /* next */
};
