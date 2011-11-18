include_HEADERS += ../../sparc/shared/include/grtm.h
noinst_PROGRAMS += tm.rel
tm_rel_SOURCES = ../../sparc/shared/tmtc/grtm.c
tm_rel_SOURCES += ../../sparc/shared/tmtc/grtm_rmap.c
tm_rel_CPPFLAGS = $(AM_CPPFLAGS)
tm_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

libbsp_a_LIBADD += tm.rel
