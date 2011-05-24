/*
 *  LIBPCI Command Implmentation
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pci.h>
#include <pci/cfg.h>
#include <pci/access.h>

#include <rtems.h>
#include <rtems/shell.h>
#include "internal.h"

static void usage(void);

static unsigned long get_pciid_from_string(char *arg)
{
  unsigned long pciid;
  char *bus_str, *dev_str, *fun_str;
  unsigned long busno, devno, funno;

  dev_str = strstr(arg, ":");
  if (dev_str == NULL) {
    /* PCIID */
    pciid = strtoul(arg, NULL, 16);
    if (pciid == ULONG_MAX)
      return ~0;
  } else {
    /* bus:dev:fun */
    bus_str = arg;
    *dev_str = '\0';
    dev_str++;
    fun_str = strstr(dev_str, ":");
    if (fun_str == NULL)
      return ~0;
    *fun_str = '\0';
    fun_str++;

    busno = strtoul(bus_str, NULL, 16);
    if (busno == ULONG_MAX)
      return ~0;
    devno = strtoul(dev_str, NULL, 16);
    if (devno == ULONG_MAX)
      return ~0;
    funno = strtoul(fun_str, NULL, 16);
    if (funno == ULONG_MAX)
      return ~0;
    pciid = PCI_DEV(busno, devno, funno);
  }

  return pciid;
}

/* General info, root bus, number of devices etc. */
void show_drvmgr_info(void)
{
  rtems_drvmgr_summary();
}

int shell_drvmgr_topo(int argc, char *argv[])
{
  rtems_drvmgr_print_topo();
  return 0;
}

int shell_drvmgr_short(int argc, char *argv[])
{
  puts(" Not implemented");
  return 0;
}

int shell_drvmgr_info(int argc, char *argv[])
{
  void *obj;

  /* Get ID from string */
  if (argc < 3)
    return -2;
  obj = get_obj_adr(argv[2]);
  if (!obj)
    return -3;

  rtems_drvmgr_info(obj);

  return 0;
}

int shell_drvmgr_remove(int argc, char *argv[])
{
  puts(" Not implemented");
  return 0;
}

int shell_drvmgr_parent(int argc, char *argv[])
{
  void *obj;
  int obj_type;
  struct rtems_drvmgr_dev_info *dev;
  struct rtems_drvmgr_bus_info *bus;

  /* Get ID from string */
  if (argc < 3)
    return -2;
  obj = get_obj_adr(argv[2]);
  if (!obj)
    return -3;

  obj_type = *(int *)obj;
  if (obj_type == DRVMGR_OBJ_BUS) {
    bus = obj;
    if (!bus->dev) {
      puts(" bus has no bridge device");
    } else if(!bus->dev->parent) {
      puts(" bridge device has no parent");
    } else {
      dev = bus->dev;
      printf(" BUSID=%p\n", dev->parent);
    }
  } else if (obj_type == DRVMGR_OBJ_DEV) {
    dev = obj;
    if (!dev->parent) {
      puts(" device has no parent bus");
    } else {
      printf(" BUSID=%p\n", dev->parent);
    }
  } else {
    puts(" ID is not a device or bus");
    return 1;
  }

  return 0;
}

int shell_drvmgr_drv(int argc, char *argv[])
{
  puts(" Not implemented");
  return 0;
}

int shell_pci_ls(int argc, char *argv[])
{
  unsigned long pciid;

  if (argc == 2) {
    /* List all devices */
    pci_print();
  } else if (argc > 3) {
    return -1;
  } else {
    pciid = get_pciid_from_string(argv[2]);
    if (pciid == ~0)
      return -1;

    pci_print_dev((pci_dev_t)pciid);
  }
  return 0;
}

int shell_pci_r8(int argc, char *argv[])
{
  unsigned long pciid, offset;
  uint8_t data;
  int result;

  if (argc != 4) {
    return -1;
  }

  pciid = get_pciid_from_string(argv[2]);
  if (pciid == ~0)
    return -1;

  offset = strtoul(dev_str, NULL, 0);
  if (offset == ULONG_MAX)
    return -1;

  result = pci_cfg_r8(pciid, offset, &data);
  switch (result) {
    default:
    case 
    
  }

  return 0;
}

const char pci_usage_str[] =
 " usage:\n"
 "  pci ls [PCIID]                     List one or all devices\n"
 "  pci r{8|16|32} PCIID OFS           Configuration space read\n"
 "                                     access by PCIID\n"
 "  pci config                         Print current PCI config for\n"
 "                                     static configuration library\n"
 "  pci --help\n";

#if 0
 "  pci ls [bus:dev:fun|PCIID]         List one or all devices\n"
 "  pci {r|w}{8|16|32} bus:dev:fun OFS Configuration space access\n"
#endif
 
static void usage(void)
{
  puts(pci_usage_str);
}

int shell_pci_usage(int argc, char *argv[])
{
  usage();
  return 0;
}

struct shell_pci_modifier {
  char *name;
  int (*func)(int argc, char *argv[], pci_dev_t dev, int ofs);
  int option;
};

#define GET_PCIID 1
#define GET_PCIOFS 2
#define MODIFIER_NUM 8
static struct shell_pci_modifier shell_pci_modifiers[MODIFIER_NUM] =
{
  {"ls", shell_pci_ls, 0},
  {"r8", shell_pci_r8, GET_PCIID|GET_PCIOFS},
  {"r16", shell_pci_r16, GET_PCIID|GET_PCIOFS},
  {"r32", shell_pci_r32, GET_PCIID|GET_PCIOFS},
  {"w8", shell_pci_w8, GET_PCIID|GET_PCIOFS},
  {"w16", shell_pci_w16, GET_PCIID|GET_PCIOFS},
  {"w32", shell_pci_w32, GET_PCIID|GET_PCIOFS},
  {"config", shell_pci_config, 0},
  {"--help", shell_pci_usage},
};

struct shell_pci_modifier *shell_pci_find_modifier(char *name)
{
  struct shell_drvmgr_modifier *mod;
  int i;

  if (name == NULL)
    return NULL;

  for (i=0, mod=&shell_drvmgr_modifiers[0]; i<MODIFIER_NUM; i++, mod++) {
    if (strcmp(name, mod->name) == 0)
      return mod;
  }

  return NULL;
}

int rtems_shell_main_pci(
  int   argc,
  char *argv[]
)
{
  struct shell_drvmgr_modifier *mod;
  int rc;

  if (argc < 2) {
    usage();
    rc = 0;
  } else if ((mod=shell_drvmgr_find_modifier(argv[1])) != NULL) {
    argi = 2;
    if (mod->option & GET_PCIID) {
      if (argc <= argi) {
        rc = -1;
      } else {
        get_pciid_from_string
        argi++;
      }
    }
    if ((rc == 0) && (mod->option & GET_PCIOFS)) {
      
    }
  
    rc = mod->func(argc, argv);
  } else {
    rc = -1;
  }

  if (rc < 0) {
    printf(" invalid argument\n");
    usage();
  }

  return rc;
}

rtems_shell_cmd_t rtems_shell_PCI_Command = {
  "pci",                         /* name */
  pci_usage_str,                 /* usage */
  "system",                      /* topic */
  rtems_shell_main_pci,          /* command */
  NULL,                          /* alias */
  NULL                           /* next */
};
