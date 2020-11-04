#include "nvme_power_control.hpp"

namespace nvme_power_control
{
void from_json(const nlohmann::json& jsonData, GpioConfig& configItem)
{
    jsonData.at("LineName").get_to(configItem.linename);
    jsonData.at("DefaultGPIOValue").get_to(configItem.defaultGpioValue);
}

std::vector<struct GpioConfig> loadGpiosConfig(const std::string& path)
{
    std::ifstream configFile(path);
    if (!configFile.is_open())
    {
        std::cerr << "loadGpiosConfig : Incorrect File Path or empty file\n ";
        throw std::runtime_error("Incorrect File Path or empty file");
    }

    auto data = nlohmann::json::parse(configFile, nullptr);

    if (data.is_discarded())
{
        std::cerr << "Failed to parse config file";
        throw std::runtime_error("Failed to parse config file");
    }

    std::vector<struct GpioConfig> configs;

    for (nlohmann::json::iterator it = data.begin(); it != data.end(); ++it)
    {
        auto gpios = it.value();
        configs.push_back(gpios.get<struct GpioConfig>());
    }

    return std::move(configs);
}

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
