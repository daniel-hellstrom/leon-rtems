$(PROJECT_INCLUDE)/grtm.h: ../../sparc/shared/include/grtm.h $(PROJECT_INCLUDE)/$(dirstamp)
	$(INSTALL_DATA) $< $(PROJECT_INCLUDE)/grtm.h
PREINSTALL_FILES += $(PROJECT_INCLUDE)/grtm.h
