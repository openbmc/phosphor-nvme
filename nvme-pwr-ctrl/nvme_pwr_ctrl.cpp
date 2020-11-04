#include <ctype.h>
#include <string.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <fstream>
#include <gpiod.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <variant>

namespace nvme_pwr_ctrl
{
static boost::asio::io_service io;
static std::shared_ptr<sdbusplus::asio::connection> conn;
static std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>>
    pwrDisIface;
static std::vector<std::shared_ptr<sdbusplus::asio::dbus_interface>> pwrEnIface;

static constexpr const char* nvmePowerService =
    "xyz.openbmc_project.Control.Nvme.Power";
static constexpr const char* nvmePowerIface =
    "xyz.openbmc_project.Control.Nvme.Power";
static std::string nvmePowerPath = "/xyz/openbmc_project/control/nvme/";
static std::vector<std::string> pwrDisLineName;
static std::vector<std::string> pwrEnLineName;
static std::vector<std::string> pwrDisPath;
static std::vector<std::string> pwrEnPath;
static int max_ssd_nums = 0;

static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line.\n";
        return false;
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_OUTPUT},
                         value);
    }
    catch (std::exception&)
    {
        std::cerr << "Failed to request " << name << " output\n";
        return false;
    }

    std::cerr << name << " set to " << std::to_string(value) << "\n";
    return true;
}

static void PowerStateMonitor()
{
    static auto match = sdbusplus::bus::match::match(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0namespace=xyz.openbmc_project.control.nvme.power",
        [](sdbusplus::message::message& m) {
            std::string intfName;
            boost::container::flat_map<std::string,
                                       std::variant<bool, std::string>>
                propertiesChanged;

            m.read(intfName, propertiesChanged);
            std::string obj_path;
            obj_path = m.get_path();

            std::string line_name;
            size_t pos = 0;
            std::string token;
            std::string delimiter = "/";
            while ((pos = obj_path.find(delimiter)) != std::string::npos)
            {
                token = obj_path.substr(0, pos);
                std::cout << token << std::endl;
                obj_path.erase(0, pos + delimiter.length());
            }
            line_name.assign(obj_path);
            std::cerr << "line_name: " << line_name << "\n";

            try
            {
                bool value = std::get<bool>(propertiesChanged.begin()->second);
                std::cerr << "value: " << value << "\n";
                gpiod::line line;
                setGPIOOutput(line_name, value, line);
                // Release line
                line.reset();
            }
            catch (std::exception& e)
            {
                std::cerr << "Unable to read property\n";
                return;
            }
        });
}

static int loadGpiosConfig()
{
    const std::string configFilePath = "/etc/nvme/nvme_power_gpio.json";

    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        std::cerr << "loadGpiosConfig : Cannot open nvme power gpio path\n ";
        return -1;
    }

    auto data = nlohmann::json::parse(configFile, nullptr);

    if (data.is_discarded())
    {
        std::cerr << "nvme power gpio JSON parser failure";
        return -1;
    }
    for (nlohmann::json::iterator it = data.begin(); it != data.end(); ++it)
    {
        auto tmp = it.value();
        pwrDisLineName.push_back(tmp.at("NVMeDrivePwrDis"));
        pwrEnLineName.push_back(tmp.at("NVMeDrivePwrEn"));
        max_ssd_nums++;
    }

    return 0;
}
} // namespace nvme_pwr_ctrl

int main(int argc, char* argv[])
{
    std::cerr << "Start NVMe power control service...\n";
    nvme_pwr_ctrl::conn =
        std::make_shared<sdbusplus::asio::connection>(nvme_pwr_ctrl::io);

    // Load Power GPIO's through json config file
    if (nvme_pwr_ctrl::loadGpiosConfig() == -1)
    {
        std::cerr << "nvme power control: Error in Parsing...\n";
        return -1;
    }

    // Request dbus name
    nvme_pwr_ctrl::conn->request_name(nvme_pwr_ctrl::nvmePowerService);
    sdbusplus::asio::object_server server =
        sdbusplus::asio::object_server(nvme_pwr_ctrl::conn);

    int i;

    for (i = 0; i < nvme_pwr_ctrl::max_ssd_nums; i++)
    {
        // Set PwrDis dbus path
        nvme_pwr_ctrl::pwrDisPath.push_back(nvme_pwr_ctrl::nvmePowerPath +
                                            nvme_pwr_ctrl::pwrDisLineName[i]);

        // Add PwrDis interface and propety
        nvme_pwr_ctrl::pwrDisIface.push_back(server.add_interface(
            nvme_pwr_ctrl::pwrDisPath[i], nvme_pwr_ctrl::nvmePowerIface));
        nvme_pwr_ctrl::pwrDisIface[i]->register_property(
            "Asserted", false, sdbusplus::asio::PropertyPermission::readWrite);
        nvme_pwr_ctrl::pwrDisIface[i]->initialize();

        // Set PwrEn dbus path
        nvme_pwr_ctrl::pwrEnPath.push_back(nvme_pwr_ctrl::nvmePowerPath +
                                           nvme_pwr_ctrl::pwrEnLineName[i]);

        // Add PwrEn interface and propety
        nvme_pwr_ctrl::pwrEnIface.push_back(server.add_interface(
            nvme_pwr_ctrl::pwrEnPath[i], nvme_pwr_ctrl::nvmePowerIface));
        nvme_pwr_ctrl::pwrEnIface[i]->register_property(
            "Asserted", true, sdbusplus::asio::PropertyPermission::readWrite);
        nvme_pwr_ctrl::pwrEnIface[i]->initialize();
    }

    // Initialize PwrDis and PwrEn GPIOs
    gpiod::line gpioLine;

    for (i = 0; i < nvme_pwr_ctrl::max_ssd_nums; i++)
    {
        if (!nvme_pwr_ctrl::setGPIOOutput(nvme_pwr_ctrl::pwrDisLineName[i], 0,
                                          gpioLine))
        {
            return -1;
        }

        if (!nvme_pwr_ctrl::setGPIOOutput(nvme_pwr_ctrl::pwrEnLineName[i], 1,
                                          gpioLine))
        {
            return -1;
        }
    }

    // Release gpioLine
    gpioLine.reset();

    nvme_pwr_ctrl::PowerStateMonitor();

    nvme_pwr_ctrl::io.run();

    return 0;
}