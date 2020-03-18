project(
    'phosphor-nvme',
    'cpp',
    version: '1.0',
    default_options: [
        'cpp_std=c++17',
    ],
)

# Set build-type according to platform.
platform = get_option('platform')

platform_src = []
platform_json = []
if platform == 'default'
    platform_src += 'nvme_manager.cpp'
    platform_json += 'nvme_config.json'
elif platform == 'mihawk'
    platform_src += 'mihawk_nvme_manager.cpp'
    platform_json += 'mihawk_config.json'
endif

executable(
    'nvme_main',
    [
        'nvme_main.cpp',
        platform_src,
        'smbus.cpp',
        'nvmes.cpp',
    ],
    dependencies: [
        dependency('phosphor-logging'),
        dependency('sdbusplus'),
        dependency('phosphor-dbus-interfaces'),
        dependency('sdeventplus'),
    ],
    install: true,
    install_dir: get_option('bindir')
)

install_data(sources : platform_json, install_dir : '/etc/nvme')

conf_data = configuration_data()
conf_data.set('NVME_REQUEST_NAME', '"xyz.openbmc_project.nvme.manager"')
conf_data.set('NVME_OBJ_PATH_ROOT', '"/xyz/openbmc_project/sensors/temperature"')
conf_data.set('NVME_OBJ_PATH', '"/xyz/openbmc_project/sensors/temperature/nvme"')
conf_data.set('DBUS_PROPERTY_IFACE', '"org.freedesktop.DBus.Properties"')
conf_data.set('LED_GROUP_BUSNAME', '"xyz.openbmc_project.LED.GroupManager"')
conf_data.set('LED_GROUP_IFACE', '"xyz.openbmc_project.Led.Group"')
conf_data.set('LED_CONTROLLER_IFACE', '"xyz.openbmc_project.Led.Physical"')
conf_data.set('ITEM_IFACE', '"xyz.openbmc_project.Inventory.Item"')
conf_data.set('NVME_STATUS_IFACE', '"xyz.openbmc_project.Nvme.Status"')
conf_data.set('ASSET_IFACE', '"xyz.openbmc_project.Inventory.Decorator.Asset"')
conf_data.set('INVENTORY_BUSNAME', '"xyz.openbmc_project.Inventory.Manager"')
conf_data.set('NVME_INVENTORY_PATH', '"/xyz/openbmc_project/inventory/system/chassis/motherboard/nvme"')
conf_data.set('INVENTORY_NAMESPACE', '"/xyz/openbmc_project/inventory"')
conf_data.set('INVENTORY_MANAGER_IFACE', '"xyz.openbmc_project.Inventory.Manager"')

configure_file(output : 'config.h',
               configuration : conf_data)
