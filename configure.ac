# Initialization
AC_PREREQ([2.69])
AC_INIT([phosphor-nvme], [0.1])
AC_LANG([C++])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE([subdir-objects -Wall foreign dist-xz])
AM_SILENT_RULES([yes])

# Checks for programs.
AC_PROG_CXX
AM_PROG_AR
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Checks compiler characteristics.
AX_CXX_COMPILE_STDCXX_17([noext])
AX_APPEND_COMPILE_FLAGS([-Wall -Wno-unused-variable], [CXXFLAGS])

# Checks for modules
PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],, [AC_MSG_ERROR([Could not find sdbusplus...openbmc/sdbusplus package required])])
PKG_CHECK_MODULES([PHOSPHOR_LOGGING], [phosphor-logging],, [AC_MSG_ERROR([Could not find phosphor-logging...openbmc/phosphor-logging package required])])
PKG_CHECK_MODULES([PHOSPHOR_DBUS_INTERFACES], [phosphor-dbus-interfaces],, [AC_MSG_ERROR([Could not find phosphor-dbus-interfaces...openbmc/phosphor-dbus-interfaces package required])])
PKG_CHECK_MODULES([SDEVENTPLUS], [sdeventplus], [], [AC_MSG_ERROR(["sdeventplus required and not found."])])

# Dbus service name
AC_ARG_VAR(NVME_REQUEST_NAME, [The Dbus busname to own])
AS_IF([test "x$NVME_REQUEST_NAME" == "x"], [NVME_REQUEST_NAME="xyz.openbmc_project.nvme.manager"])
AC_DEFINE_UNQUOTED([NVME_REQUEST_NAME], ["$NVME_REQUEST_NAME"], [The Dbus busname to own])

# Service dbus root
AC_ARG_VAR(NVME_OBJ_PATH_ROOT, [The nvme Dbus root])
AS_IF([test "x$NVME_OBJ_PATH_ROOT" == "x"], [NVME_OBJ_PATH_ROOT="/xyz/openbmc_project/sensors/temperature"])
AC_DEFINE_UNQUOTED([NVME_OBJ_PATH_ROOT], ["$NVME_OBJ_PATH_ROOT"], [The nvme Dbus root])

# Service dbus objpath
AC_ARG_VAR(NVME_OBJ_PATH, [The nvme object path])
AS_IF([test "x$NVME_OBJ_PATH" == "x"], [NVME_OBJ_PATH="/xyz/openbmc_project/sensors/temperature/nvme"])
AC_DEFINE_UNQUOTED([NVME_OBJ_PATH], ["$NVME_OBJ_PATH"], [The nvme Dbus root])

# Service dbus interface
AC_ARG_VAR(DBUS_PROPERTY_IFACE, [The properies interface])
AS_IF([test "x$DBUS_PROPERTY_IFACE" == "x"], [DBUS_PROPERTY_IFACE="org.freedesktop.DBus.Properties"])
AC_DEFINE_UNQUOTED([DBUS_PROPERTY_IFACE], ["$DBUS_PROPERTY_IFACE"], [The nvme Dbus root])

LT_INIT # Required for systemd linking

# Create configured output
AC_CONFIG_FILES([Makefile])
AC_OUTPUT







