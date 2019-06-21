#include "nvme_manager.hpp"

#include "smbus.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/message.hpp>
#include <sstream>
#include <string>
#include <xyz/openbmc_project/Led/Physical/server.hpp>

#include "i2c-dev.h"

#define MONITOR_INTERVAL_SECONDS 1
#define NVME_SSD_SLAVE_ADDRESS 0x6a
#define GPIO_BASE_PATH "/sys/class/gpio/gpio"
#define IS_PRESENT "0"
#define POWERGD "1"
#define NOWARNING_STRING "ff"

static constexpr auto configFile = "/etc/nvme/nvme_config.json";
auto retries = 3;
static constexpr auto delay = std::chrono::milliseconds{100};
using Json = nlohmann::json;

static constexpr const uint8_t COMMAND_CODE_0 = 0;
static constexpr const uint8_t COMMAND_CODE_8 = 8;

static constexpr int CapacityFaultMask = 1;
static constexpr int temperatureFaultMask = 1 << 1;
static constexpr int DegradesFaultMask = 1 << 2;
static constexpr int MediaFaultMask = 1 << 3;
static constexpr int BackupDeviceFaultMask = 1 << 4;
static constexpr int NOWARNING = 255;

static constexpr int SERIALNUMBER_START_INDEX = 3;
static constexpr int SERIALNUMBER_END_INDEX = 23;

static constexpr const u_int64_t TEMPERATURE_SENSOR_FAILURE = 129;

namespace fs = std::experimental::filesystem;

namespace phosphor
{
namespace nvme
{

using namespace std;
using namespace phosphor::logging;

void Nvme::setNvmeInventoryProperties(
    const bool& present, const phosphor::nvme::Nvme::NVMeData& nvmeData,
    const std::string& inventoryPath)
{
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 ITEM_IFACE, "Present", present);
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 ASSET_IFACE, "Manufacturer", nvmeData.vendor);
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 ASSET_IFACE, "SerialNumber",
                                 nvmeData.serialNumber);
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "SmartWarnings",
                                 nvmeData.smartWarnings);
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "StatusFlags",
                                 nvmeData.statusFlags);
    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "DriveLifeUsed",
                                 nvmeData.driveLifeUsed);

    auto smartWarning = (!nvmeData.smartWarnings.empty())
                            ? std::stoi(nvmeData.smartWarnings, 0, 16)
                            : NOWARNING;

    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "CapacityFault",
                                 !(smartWarning & CapacityFaultMask));

    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "TemperatureFault",
                                 !(smartWarning & temperatureFaultMask));

    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "DegradesFault",
                                 !(smartWarning & DegradesFaultMask));

    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "MediaFault",
                                 !(smartWarning & MediaFaultMask));

    util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                 NVME_STATUS_IFACE, "BackupDeviceFault",
                                 !(smartWarning & BackupDeviceFaultMask));
}

void Nvme::setFaultLED(const std::string& locateLedGroupPath,
                       const std::string& faultLedGroupPath,
                       const bool& request)
{
    if (locateLedGroupPath.empty() || faultLedGroupPath.empty())
    {
        return;
    }

    // Before toggle LED, check whether is Identify or not.
    if (!getLEDGroupState(locateLedGroupPath))
    {
        util::SDBusPlus::setProperty(bus, LED_GROUP_BUSNAME, faultLedGroupPath,
                                     LED_GROUP_IFACE, "Asserted", request);
    }
}

void Nvme::setLocateLED(const std::string& locateLedGroupPath,
                        const std::string& locateLedBusName,
                        const std::string& locateLedPath, const bool& isPresent)
{
    if (locateLedGroupPath.empty() || locateLedBusName.empty() ||
        locateLedPath.empty())
    {
        return;
    }

    namespace server = sdbusplus::xyz::openbmc_project::Led::server;

    if (!getLEDGroupState(locateLedGroupPath))
    {
        if (isPresent)
            util::SDBusPlus::setProperty(
                bus, locateLedBusName, locateLedPath, LED_CONTROLLER_IFACE,
                "State",
                server::convertForMessage(server::Physical::Action::On));
        else
            util::SDBusPlus::setProperty(
                bus, locateLedBusName, locateLedPath, LED_CONTROLLER_IFACE,
                "State",
                server::convertForMessage(server::Physical::Action::Off));
    }
}

bool Nvme::getLEDGroupState(const std::string& ledPath)
{
    auto asserted = util::SDBusPlus::getProperty<bool>(
        bus, LED_GROUP_BUSNAME, ledPath, LED_GROUP_IFACE, "Asserted");

    return asserted;
}

void Nvme::setLEDsStatus(const phosphor::nvme::Nvme::NVMeConfig& config,
                         bool success,
                         const phosphor::nvme::Nvme::NVMeData& nvmeData)
{
    if (success)
    {
        if (!nvmeData.smartWarnings.empty())
        {
            auto request =
                (strcmp(nvmeData.smartWarnings.c_str(), NOWARNING_STRING) == 0)
                    ? false
                    : true;

            setFaultLED(config.locateLedGroupPath, config.faultLedGroupPath,
                        request);
            setLocateLED(config.locateLedGroupPath,
                         config.locateLedControllerBusName,
                         config.locateLedControllerPath, !request);
        }
    }
    else
    {
        // Drive is present but can not get data, turn on fault LED.
        log<level::ERR>(
            "Drive status is good but can not get data.",
            entry("objPath = %s", std::to_string(config.index).c_str()));
        setFaultLED(config.locateLedGroupPath, config.faultLedGroupPath, true);
        setLocateLED(config.locateLedGroupPath,
                     config.locateLedControllerBusName,
                     config.locateLedControllerPath, false);
    }
}

std::string intToHex(int input)
{
    std::stringstream tmp;
    tmp << std::hex << input;

    return tmp.str();
}

/** @brief Get NVMe info over smbus  */
bool getNVMeInfobyBusID(int busID, phosphor::nvme::Nvme::NVMeData& nvmeData)
{
    nvmeData.present = true;
    nvmeData.vendor = "";
    nvmeData.serialNumber = "";
    nvmeData.smartWarnings = "";
    nvmeData.statusFlags = "";
    nvmeData.driveLifeUsed = "";
    nvmeData.sensorValue = TEMPERATURE_SENSOR_FAILURE;

    phosphor::smbus::Smbus smbus;

    unsigned char rsp_data_command_0[I2C_DATA_MAX] = {0};
    unsigned char rsp_data_command_8[I2C_DATA_MAX] = {0};

    uint8_t tx_data = COMMAND_CODE_0;

    auto init = smbus.smbusInit(busID);

    if (init == -1)
    {
        log<level::ERR>("smbusInit fail!");

        nvmeData.present = false;

        return nvmeData.present;
    }

    auto res_int =
        smbus.SendSmbusRWBlockCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS, &tx_data,
                                     sizeof(tx_data), rsp_data_command_0);

    if (res_int < 0)
    {
        log<level::ERR>("Send command code 0 fail!");

        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    tx_data = COMMAND_CODE_8;

    res_int =
        smbus.SendSmbusRWBlockCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS, &tx_data,
                                     sizeof(tx_data), rsp_data_command_8);

    if (res_int < 0)
    {
        log<level::ERR>("Send command code 8 fail!");
        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    nvmeData.vendor =
        intToHex(rsp_data_command_8[1]) + " " + intToHex(rsp_data_command_8[2]);

    for (int offset = SERIALNUMBER_START_INDEX; offset < SERIALNUMBER_END_INDEX;
         offset++)
    {
        nvmeData.serialNumber += static_cast<char>(rsp_data_command_8[offset]);
    }

    nvmeData.statusFlags = intToHex(rsp_data_command_0[1]);
    nvmeData.smartWarnings = intToHex(rsp_data_command_0[2]);
    nvmeData.driveLifeUsed = intToHex(rsp_data_command_0[4]);
    nvmeData.sensorValue = (u_int64_t)rsp_data_command_0[3];

    smbus.smbusClose(busID);

    return nvmeData.present;
}

void Nvme::run()
{
    init();

    std::function<void()> callback(std::bind(&Nvme::read, this));
    try
    {
        u_int64_t interval = MONITOR_INTERVAL_SECONDS * 1000000;
        _timer.restart(std::chrono::microseconds(interval));
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("Error in polling loop. "),
            entry("ERROR = %s", e.what());
    }
}

/** @brief Parsing NVMe config JSON file  */
Json parseSensorConfig()
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        log<level::ERR>("NVMe config JSON file not found");
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("NVMe config readings JSON parser failure");
    }

    return data;
}

/** @brief Obtain the initial configuration value of NVMe  */
std::vector<phosphor::nvme::Nvme::NVMeConfig> getNvmeConfig()
{

    phosphor::nvme::Nvme::NVMeConfig nvmeConfig;
    std::vector<phosphor::nvme::Nvme::NVMeConfig> nvmeConfigs;
    uint64_t criticalHigh = 0;
    uint64_t criticalLow = 0;
    uint64_t maxValue = 0;
    uint64_t minValue = 0;
    uint64_t warningHigh = 0;
    uint64_t warningLow = 0;

    try
    {
        auto data = parseSensorConfig();
        static const std::vector<Json> empty{};
        std::vector<Json> readings = data.value("config", empty);
        std::vector<Json> thresholds = data.value("threshold", empty);
        if (!thresholds.empty())
        {
            for (const auto& instance : thresholds)
            {
                criticalHigh = instance.value("criticalHigh", 0);
                criticalLow = instance.value("criticalLow", 0);
                maxValue = instance.value("maxValue", 0);
                minValue = instance.value("minValue", 0);
                warningHigh = instance.value("warningHigh", 0);
                warningLow = instance.value("warningLow", 0);
            }
        }
        else
        {
            log<level::ERR>(
                "Invalid NVMe config file, thresholds dosen't exist");
        }

        if (!readings.empty())
        {
            for (const auto& instance : readings)
            {
                uint8_t index = instance.value("NVMeDriveIndex", 0);
                uint8_t busID = instance.value("NVMeDriveBusID", 0);
                std::string faultLedGroupPath =
                    instance.value("NVMeDriveFaultLEDGroupPath", "");
                std::string locateLedGroupPath =
                    instance.value("NVMeDriveLocateLEDGroupPath", "");
                uint8_t presentPin = instance.value("NVMeDrivePresentPin", 0);
                uint8_t pwrGoodPin = instance.value("NVMeDrivePwrGoodPin", 0);
                std::string locateLedControllerBusName =
                    instance.value("NVMeDriveLocateLEDControllerBusName", "");
                std::string locateLedControllerPath =
                    instance.value("NVMeDriveLocateLEDControllerPath", "");

                nvmeConfig.index = index;
                nvmeConfig.busID = busID;
                nvmeConfig.faultLedGroupPath = faultLedGroupPath;
                nvmeConfig.presentPin = presentPin;
                nvmeConfig.pwrGoodPin = pwrGoodPin;
                nvmeConfig.locateLedControllerBusName =
                    locateLedControllerBusName;
                nvmeConfig.locateLedControllerPath = locateLedControllerPath;
                nvmeConfig.locateLedGroupPath = locateLedGroupPath;
                nvmeConfig.criticalHigh = criticalHigh;
                nvmeConfig.criticalLow = criticalLow;
                nvmeConfig.warningHigh = warningHigh;
                nvmeConfig.warningLow = warningLow;
                nvmeConfig.maxValue = maxValue;
                nvmeConfig.minValue = minValue;
                nvmeConfigs.push_back(nvmeConfig);
            }
        }
        else
        {
            log<level::ERR>("Invalid NVMe config file, config dosen't exist");
        }
    }
    catch (const Json::exception& e)
    {
        log<level::ERR>("Json Exception caught."), entry("MSG: %s", e.what());
    }

    return nvmeConfigs;
}

std::string Nvme::getGPIOValueOfNvme(const std::string& fullPath)
{
    std::string val;
    std::ifstream ifs;

    while (true)
    {
        try
        {
            if (!ifs.is_open())
                ifs.open(fullPath);
            ifs.clear();
            ifs.seekg(0);
            ifs >> val;
        }
        catch (const std::exception& e)
        {
            --retries;
            std::this_thread::sleep_for(delay);
            log<level::ERR>("Can not open gpio path.",
                            entry("MSG: %s", e.what()));
            continue;
        }
        break;
    }

    ifs.close();
    return val;
}

void Nvme::createNVMeInventory()
{
    using Properties =
        std::map<std::string, sdbusplus::message::variant<std::string, bool>>;
    using Interfaces = std::map<std::string, Properties>;

    std::string inventoryPath;
    std::map<sdbusplus::message::object_path, Interfaces> obj;

    for (int i = 0; i < (int)(configs.size()); i++)
    {
        inventoryPath = "/system/chassis/motherboard/nvme" +
                        std::to_string(configs[i].index);

        obj = {{
            inventoryPath,
            {{ITEM_IFACE, {}}, {NVME_STATUS_IFACE, {}}, {ASSET_IFACE, {}}},
        }};
        util::SDBusPlus::CallMethod(bus, INVENTORY_BUSNAME, INVENTORY_NAMESPACE,
                                    INVENTORY_MANAGER_IFACE, "Notify", obj);
    }
}

void Nvme::init()
{
    // read json file
    configs = getNvmeConfig();
    createNVMeInventory();
}

/** @brief Monitor NVMe drives every one second  */
void Nvme::read()
{
    std::string devPresentPath;
    std::string devPwrGoodPath;
    std::string inventoryPath;

    for (int i = 0; i < (int)(configs.size()); i++)
    {
        NVMeData nvmeData;
        devPresentPath =
            GPIO_BASE_PATH + std::to_string(configs[i].presentPin) + "/value";

        devPwrGoodPath =
            GPIO_BASE_PATH + std::to_string(configs[i].pwrGoodPin) + "/value";

        inventoryPath = NVME_INVENTORY_PATH + std::to_string(configs[i].index);

        auto iter = nvmes.find(std::to_string(configs[i].index));

        if (getGPIOValueOfNvme(devPresentPath) == IS_PRESENT)
        {
            // Drive status is good, update value or create d-bus and update
            // value.
            if (getGPIOValueOfNvme(devPwrGoodPath) == POWERGD)
            {
                // get NVMe information through i2c by busID.
                auto success = getNVMeInfobyBusID(configs[i].busID, nvmeData);
                // can not find. create dbus
                if (iter == nvmes.end())
                {
                    log<level::INFO>(
                        "SSD plug.",
                        entry("index = %s",
                              std::to_string(configs[i].index).c_str()));

                    std::string objPath =
                        NVME_OBJ_PATH + std::to_string(configs[i].index);
                    auto nvmeSSD = std::make_shared<phosphor::nvme::NvmeSSD>(
                        bus, objPath.c_str());
                    nvmes.emplace(std::to_string(configs[i].index), nvmeSSD);

                    setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                    nvmeSSD->setSensorValueToDbus(nvmeData.sensorValue);
                    nvmeSSD->setSensorThreshold(
                        configs[i].criticalHigh, configs[i].criticalLow,
                        configs[i].maxValue, configs[i].minValue,
                        configs[i].warningHigh, configs[i].warningLow);

                    nvmeSSD->checkSensorThreshold();
                    setLEDsStatus(configs[i], success, nvmeData);
                }
                else
                {
                    setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                    iter->second->setSensorValueToDbus(nvmeData.sensorValue);
                    iter->second->checkSensorThreshold();
                    setLEDsStatus(configs[i], success, nvmeData);
                }
            }
            else
            {
                // Present pin is true but power good pin is false
                // remove nvme d-bus path, clean all properties in inventory
                // and turn on fault LED

                log<level::ERR>(
                    "Present pin is true but power good pin is false.",
                    entry("index = %s",
                          std::to_string(configs[i].index).c_str()));

                setFaultLED(configs[i].locateLedGroupPath,
                            configs[i].faultLedGroupPath, true);
                setLocateLED(configs[i].locateLedGroupPath,
                             configs[i].locateLedControllerBusName,
                             configs[i].locateLedControllerPath, false);

                nvmeData = NVMeData();
                setNvmeInventoryProperties(false, nvmeData, inventoryPath);
                nvmes.erase(std::to_string(configs[i].index));
                log<level::ERR>(
                    "Erase SSD from map and d-bus.",
                    entry("index = %s",
                          std::to_string(configs[i].index).c_str()));
            }
        }
        else
        {
            // Drive not present, remove nvme d-bus path ,
            // clean all properties in inventory
            // and turn off fault and locate LED

            setFaultLED(configs[i].locateLedGroupPath,
                        configs[i].faultLedGroupPath, false);
            setLocateLED(configs[i].locateLedGroupPath,
                         configs[i].locateLedControllerBusName,
                         configs[i].locateLedControllerPath, false);

            nvmeData = NVMeData();
            setNvmeInventoryProperties(false, nvmeData, inventoryPath);
            nvmes.erase(std::to_string(configs[i].index));
        }
    }
}
} // namespace nvme
} // namespace phosphor
