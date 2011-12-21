##
## $Id$
##

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am

include_bspdir = $(includedir)/bsp

dist_project_lib_DATA = bsp_specs

include_HEADERS = include/bsp.h
include_HEADERS += include/tm27.h
include_HEADERS += ../../sparc/shared/include/debug_defs.h

nodist_include_HEADERS = include/bspopts.h
nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h
DISTCLEANFILES = include/bspopts.h

noinst_PROGRAMS =

# wrapup is the one that actually builds and installs the library
# from the individual .rel files built in other directories
SUBDIRS = . tools

include_HEADERS += include/leon.h
include_HEADERS += ../../shared/include/coverhd.h

noinst_LIBRARIES = libbspstart.a
libbspstart_a_SOURCES = ../../sparc/shared/start.S
project_lib_DATA = start.$(OBJEXT)

dist_project_lib_DATA += \
    startup/linkcmds \
    startup/linkcmds_ngmp \
    ../shared/startup/linkcmds.base

noinst_LIBRARIES += libbsp.a
libbsp_a_SOURCES =

# startup
libbsp_a_SOURCES += ../../shared/bspclean.c ../../shared/bsplibc.c \
    ../../shared/bsppost.c ../../shared/bootcard.c startup/bspstart.c \
    ../../sparc/shared/bsppretaskinghook.c startup/bsppredriver.c \
    ../../sparc/shared/bspgetworkarea.c ../../shared/sbrk.c startup/setvec.c \
    ../../sparc/shared/startup/early_malloc.c \
    startup/spurious.c startup/bspidle.S
# gnatsupp
libbsp_a_SOURCES += gnatsupp/gnatsupp.c ../../sparc/shared/gnatcommon.c
# AMBA bus
include_HEADERS += include/amba.h
include_HEADERS += ../../sparc/shared/include/ambapp.h
include_HEADERS += ../../sparc/shared/include/ambapp_ids.h
include_HEADERS += ../../sparc/shared/include/grlib.h
libbsp_a_SOURCES += amba/amba.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_alloc.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_count.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_depth.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_find_by_idx.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_freq.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_parent.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_old.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_names.c
libbsp_a_SOURCES += ../../sparc/shared/amba/ambapp_show.c

# Clock Driver and Timer Library
include_HEADERS += ../../sparc/shared/include/tlib.h
libbsp_a_SOURCES += ../../sparc/shared/timer/gptimer.c
libbsp_a_SOURCES += ../../sparc/shared/timer/tlib.c
libbsp_a_SOURCES += ../../sparc/shared/timer/tlib_ckinit.c
# non-Driver Manager Clock Implementation
libbsp_a_SOURCES += clock/ckinit.c ../../../shared/clockdrv_shell.h

# console
libbsp_a_SOURCES += console/console.c
libbsp_a_SOURCES += ../../sparc/shared/uart/cons.c
libbsp_a_SOURCES += ../../sparc/shared/uart/apbuart_cons.c
include_HEADERS += ../../sparc/shared/include/cons.h
# debugio
libbsp_a_SOURCES += console/debugputs.c

# IRQ
include_bsp_HEADERS = \
    ../../shared/include/irq-generic.h \
    ../../shared/include/irq-info.h \
    include/bsp/irq.h
libbsp_a_SOURCES += \
    startup/eirq.c \
    ../../sparc/shared/irq/irq-shared.c \
    ../../shared/src/irq-generic.c \
    ../../shared/src/irq-legacy.c \
    ../../shared/src/irq-info.c \
    ../../shared/src/irq-shell.c \
    ../../shared/src/irq-server.c

# PCI
include_HEADERS += ../../sparc/shared/include/grpci2.h
libbsp_a_SOURCES += ../../sparc/shared/pci/grpci.c
libbsp_a_SOURCES += ../../sparc/shared/pci/grpci2.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/pcif.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/pci_memreg_sparc_le.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/pci_memreg_sparc_be.c

# PCI target devices
include_HEADERS += ../../sparc/shared/include/gr_701.h
include_HEADERS += ../../sparc/shared/include/gr_rasta_adcdac.h
include_HEADERS += ../../sparc/shared/include/gr_rasta_io.h
include_HEADERS += ../../sparc/shared/include/gr_rasta_tmtc.h
include_HEADERS += ../../sparc/shared/include/gr_tmtc_1553.h
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_701.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_rasta_adcdac.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_rasta_io.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_rasta_spw_router.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_rasta_tmtc.c
libbsp_a_SOURCES +=  ../../sparc/shared/pci/gr_tmtc_1553.c

# B1553BRM
include_HEADERS += ../../sparc/shared/include/b1553brm.h
libbsp_a_SOURCES += ../../sparc/shared/1553/b1553brm.c

# CAN
include_HEADERS += ../../sparc/shared/include/occan.h \
    ../../sparc/shared/include/grcan.h
libbsp_a_SOURCES += ../../sparc/shared/can/occan.c \
    ../../sparc/shared/can/grcan.c

# SpaceWire
include_HEADERS += ../../sparc/shared/include/grspw.h
libbsp_a_SOURCES += ../../sparc/shared/spw/grspw.c

# UART
include_HEADERS += ../../sparc/shared/include/apbuart.h
libbsp_a_SOURCES += ../../sparc/shared/uart/apbuart.c

# I2CMST
include_HEADERS += ../../sparc/shared/include/i2cmst.h
libbsp_a_SOURCES += ../../sparc/shared/i2c/i2cmst.c

# timer
libbsp_a_SOURCES += timer/timer.c
libbsp_a_SOURCES += timer/watchdog.c

# Driver Manager 
include_drvmgrdir = $(includedir)/drvmgr
include_drvmgr_HEADERS = ../../sparc/shared/include/drvmgr/ambapp_bus_grlib.h
include_drvmgr_HEADERS += ../../sparc/shared/include/drvmgr/ambapp_bus.h
libbsp_a_SOURCES += ../../sparc/shared/drvmgr/ambapp_bus.c
libbsp_a_SOURCES += ../../sparc/shared/drvmgr/ambapp_bus_grlib.c

if HAS_NETWORKING
noinst_PROGRAMS += leon_smc91111.rel
leon_smc91111_rel_SOURCES = leon_smc91111/leon_smc91111.c
leon_smc91111_rel_CPPFLAGS = $(AM_CPPFLAGS)
leon_smc91111_rel_CPPFLAGS += -D__INSIDE_RTEMS_BSD_TCPIP_STACK__
leon_smc91111_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
endif

if HAS_NETWORKING
noinst_PROGRAMS += leon_open_eth.rel
leon_open_eth_rel_SOURCES = leon_open_eth/leon_open_eth.c
leon_open_eth_rel_CPPFLAGS = $(AM_CPPFLAGS)
leon_open_eth_rel_CPPFLAGS += -D__INSIDE_RTEMS_BSD_TCPIP_STACK__
leon_open_eth_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
endif

if HAS_NETWORKING
noinst_PROGRAMS += leon_greth.rel
leon_greth_rel_SOURCES = leon_greth/leon_greth.c
leon_greth_rel_CPPFLAGS = $(AM_CPPFLAGS)
leon_greth_rel_CPPFLAGS += -D__INSIDE_RTEMS_BSD_TCPIP_STACK__
leon_greth_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)
endif

# BSP Network configuration
if HAS_NETWORKING
include_HEADERS += ../../sparc/shared/include/network_interface_add.h
libbsp_a_SOURCES += ../../sparc/shared/net/network_interface_add.c
endif

EXTRA_DIST = shmsupp/README
if HAS_MP
# shmsupp
libbsp_a_SOURCES += shmsupp/addrconv.c shmsupp/getcfg.c shmsupp/lock.c \
    shmsupp/mpisr.c
endif

libbsp_a_LIBADD = \
    ../../../libcpu/@RTEMS_CPU@/access.rel \
    ../../../libcpu/@RTEMS_CPU@/cache.rel \
    ../../../libcpu/@RTEMS_CPU@/reg_win.rel \
    ../../../libcpu/@RTEMS_CPU@/syscall.rel

if HAS_NETWORKING
libbsp_a_LIBADD += leon_open_eth.rel
libbsp_a_LIBADD += leon_greth.rel
libbsp_a_LIBADD += leon_smc91111.rel
endif

EXTRA_DIST += times

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am
