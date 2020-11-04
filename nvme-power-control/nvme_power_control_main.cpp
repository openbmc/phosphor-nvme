#include "config.hpp"
#include "nvme_power_control.hpp"

int main(void)
{
    std::cerr << "Start NVMe power control service...\n";

    auto gpioConfigs =
        nvme_power_control::loadGpiosConfig(NVME_POWER_GPIO_CONFIG_FILE);

    boost::asio::io_context ioc;

    auto conn = std::make_shared<sdbusplus::asio::connection>(ioc);
    auto objServer = std::make_unique<sdbusplus::asio::object_server>(conn);

    // Request dbus name
    conn->request_name(nvmePowerService);

    nvme_power_control::NvmePowerCtrl ctrl(ioc, *conn, *objServer, gpioConfigs);

    ioc.run();

    return 0;
}
