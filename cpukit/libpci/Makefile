### Debug Makefile for just building LIBPCI

CC=/opt/rtems-4.10/bin/sparc-rtems-gcc
CFLAGS=-Wall -I. -I/opt/rtems-4.10/sparc-rtems/leon3/lib/include -g3 -O3 -c

.PHONY: drvmgr pci

drvmgr:
	$(CC) $(CFLAGS) pci_bus.c

pci:
	$(CC) $(CFLAGS) pci_access.c
	$(CC) $(CFLAGS) pci_access_func.c
	$(CC) $(CFLAGS) pci_access_io.c
	$(CC) $(CFLAGS) pci_access_mem_be.c
	$(CC) $(CFLAGS) pci_access_mem.c
	$(CC) $(CFLAGS) pci_access_mem_le.c
	$(CC) $(CFLAGS) pci_bus.c
	$(CC) $(CFLAGS) pci_cfg_auto.c
	$(CC) $(CFLAGS) pci_cfg.c
	$(CC) $(CFLAGS) pci_cfg_peripheral.c
	$(CC) $(CFLAGS) pci_cfg_print_code.c
	$(CC) $(CFLAGS) pci_cfg_read.c
	$(CC) $(CFLAGS) pci_cfg_static.c
	$(CC) $(CFLAGS) pci_find.c
	$(CC) $(CFLAGS) pci_find_dev.c
	$(CC) $(CFLAGS) pci_for_each.c
	$(CC) $(CFLAGS) pci_for_each_child.c
	$(CC) $(CFLAGS) pci_for_each_dev.c
	$(CC) $(CFLAGS) pci_get_dev.c
	$(CC) $(CFLAGS) pci_irq.c
	$(CC) $(CFLAGS) pci_print.c
