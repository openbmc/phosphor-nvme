#include "nvme_power_control.hpp"

#include "config.hpp"

namespace nvme_power_control
{
bool NvmePowerCtrl::setGPIOOutput(const std::string& name, const int value,
                                  gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line.\n";
        throw std::runtime_error("Failed to find the line name");
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request(
            {__FUNCTION__, gpiod::line_request::DIRECTION_OUTPUT, 0}, value);
    }
    catch (std::exception&)
    {
        std::cerr << "Failed to request " << name << " output\n";
        return false;
    }

    std::cerr << name << " set to " << std::to_string(value) << "\n";
    return true;
}

void NvmePowerCtrl::createNvmePowerCtrl(std::vector<struct GpioConfig> configs)
{
    std::string path;
    gpiod::line gpioLine;
    int i = 0;

    for (const auto& config : configs)
    {
        // Set dbus path
        path = nvmePowerPath + config.linename;

        interfaces.push_back(objServer_.add_interface(path, nvmePowerIface));

        setGPIOOutput(config.linename, config.defaultGpioValue, gpioLine);

        interfaces[i]->register_property(
            "Asserted", static_cast<bool>(config.defaultGpioValue),
            // custom set
            [this, config](const bool req, bool& rsp) {
                int value;
                value = static_cast<int>(req);
                gpiod::line line;
                setGPIOOutput(config.linename, value, line);
                line.reset();

                rsp = req;
                return 1;
            });

        interfaces[i++]->initialize();
    }

    // Release gpioLine
    gpioLine.reset();
}
} // namespace nvme_power_control

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
