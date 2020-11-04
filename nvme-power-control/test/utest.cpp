#include "nvme_power_control.hpp"

#include <fstream>

#include <gtest/gtest.h>

namespace nvme_power_control
{
TEST(TestNvmePowerCtrl, TestCreateObject)
{
    boost::asio::io_context ioc;

    auto conn = std::make_shared<sdbusplus::asio::connection>(ioc);
    auto objServer = std::make_unique<sdbusplus::asio::object_server>(conn);

    auto j2 = R"(
      [{
        "LineName": "pwr_u2_0_en",
        "DefaultGPIOValue": 1
      }]
    )"_json;

    std::ofstream file("gpio_example.json");
    file << std::setw(4) << j2 << std::endl;

    auto gpioConfigs = loadGpiosConfig("gpio_example.json");

    NvmePowerCtrl testCtrl(ioc, *conn, *objServer, gpioConfigs);

    std::remove("gpio_example.json");
}
} // namespace nvme_power_control
