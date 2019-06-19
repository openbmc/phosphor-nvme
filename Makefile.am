ACLOCAL_AMFLAGS = -I m4
AM_DEFAULT_SOURCE_EXT = .cpp
bin_PROGRAMS = nvme_main

nvme_main_SOURCES = \
                nvme_main.cpp  \
                nvme_manager.cpp \
                smbus.cpp \
                nvmes.cpp


nvme_main_LDFLAGS = \
                    -lstdc++fs \
                    $(SDBUSPLUS_LIBS) \
                    $(PHOSPHOR_DBUS_INTERFACES_LIBS) \
                    $(SDEVENTPLUS_LIBS)

nvme_main_CXX_FLAGS =  \
                $(PHOSPHOR_DBUS_INTERFACES_CFLAGS) \
	            $(SDBUSPLUS_CFLAGS) \
	            $(PHOSPHOR_LOGGING_CFLAGS)

nvmedir = $(sysconfdir)/nvme
nvme_DATA = nvme_config.json