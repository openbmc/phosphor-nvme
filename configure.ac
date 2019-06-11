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

# LED group service dbus busname
AC_ARG_VAR(LED_GROUP_BUSNAME, [The LED group manager dbus name])
AS_IF([test "x$LED_GROUP_BUSNAME" == "x"], [LED_GROUP_BUSNAME="xyz.openbmc_project.LED.GroupManager"])
AC_DEFINE_UNQUOTED([LED_GROUP_BUSNAME], ["$LED_GROUP_BUSNAME"], [The LED group manager dbus name])

# LED group interface
AC_ARG_VAR(LED_GROUP_IFACE, [The LED group manager interface])
AS_IF([test "x$LED_GROUP_IFACE" == "x"], [LED_GROUP_IFACE="xyz.openbmc_project.Led.Group"])
AC_DEFINE_UNQUOTED([LED_GROUP_IFACE], ["$LED_GROUP_IFACE"], [The LED group manager interface])

# LED controller interface
AC_ARG_VAR(LED_CONTROLLER_IFACE, [The LED controller interface])
AS_IF([test "x$LED_CONTROLLER_IFACE" == "x"], [LED_CONTROLLER_IFACE="xyz.openbmc_project.Led.Physical"])
AC_DEFINE_UNQUOTED([LED_CONTROLLER_IFACE], ["$LED_CONTROLLER_IFACE"], [The LED controller interface])

# item interface
AC_ARG_VAR(ITEM_IFACE, [The item interface])
AS_IF([test "x$ITEM_IFACE" == "x"], [ITEM_IFACE="xyz.openbmc_project.Inventory.Item"])
AC_DEFINE_UNQUOTED([ITEM_IFACE], ["$ITEM_IFACE"], [The item interface])

# NVMe status interface
AC_ARG_VAR(NVME_STATUS_IFACE, [The NVMe status interface])
AS_IF([test "x$NVME_STATUS_IFACE" == "x"], [NVME_STATUS_IFACE="xyz.openbmc_project.Nvme.Status"])
AC_DEFINE_UNQUOTED([NVME_STATUS_IFACE], ["$NVME_STATUS_IFACE"], [The NVMe status interface])

# Asset interface
AC_ARG_VAR(ASSET_IFACE, [The Asset interface])
AS_IF([test "x$ASSET_IFACE" == "x"], [ASSET_IFACE="xyz.openbmc_project.Inventory.Decorator.Asset"])
AC_DEFINE_UNQUOTED([ASSET_IFACE], ["$ASSET_IFACE"], [The Asset interface])

# Inventory dbus busname
AC_ARG_VAR(INVENTORY_BUSNAME, [The inventory busname])
AS_IF([test "x$INVENTORY_BUSNAME" == "x"], [INVENTORY_BUSNAME="xyz.openbmc_project.Inventory.Manager"])
AC_DEFINE_UNQUOTED([INVENTORY_BUSNAME], ["$INVENTORY_BUSNAME"], [The inventory busname])

# Inventory path
AC_ARG_VAR(INVENTORY_PATH, [The inventory path])
AS_IF([test "x$INVENTORY_PATH" == "x"], [INVENTORY_PATH="/xyz/openbmc_project/inventory/system/chassis/motherboard/nvme"])
AC_DEFINE_UNQUOTED([INVENTORY_PATH], ["$INVENTORY_PATH"], [The inventory path])

LT_INIT # Required for systemd linking

# Create configured output
AC_CONFIG_FILES([Makefile])
AC_OUTPUT







