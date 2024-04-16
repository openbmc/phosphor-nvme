#include "nvme_manager.hpp"

#include "i2c.h"

#include "smbus.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/message.hpp>
#include <xyz/openbmc_project/Led/Physical/server.hpp>

#include <filesystem>
#include <map>
#include <sstream>
#include <string>
#define MONITOR_INTERVAL_SECONDS 1
#define MAX_SMBUS_ERROR_RETRY 0
#define NVME_SSD_SLAVE_ADDRESS 0x6a
#define NVME_SSD_VPD_SLAVE_ADDRESS 0x53
#define GPIO_BASE_PATH "/sys/class/gpio/gpio"
#define IS_PRESENT "0"
#define POWERGD "1"
#define NOWARNING_STRING "ff"

static constexpr auto configFile = "/etc/nvme/nvme_config.json";
static constexpr auto delay = std::chrono::milliseconds{100};
using Json = nlohmann::json;

static constexpr const uint8_t COMMAND_CODE_0 = 0;
static constexpr const uint8_t COMMAND_CODE_8 = 8;
static constexpr const uint8_t CODE_0_LENGTH = 8;
static constexpr const uint8_t CODE_8_LENGTH = 23;

static constexpr int CapacityFaultMask = 1;
static constexpr int temperatureFaultMask = 1 << 1;
static constexpr int DegradesFaultMask = 1 << 2;
static constexpr int MediaFaultMask = 1 << 3;
static constexpr int BackupDeviceFaultMask = 1 << 4;
static constexpr int DriveNotReadyMask = 1 << 6;
static constexpr int NOWARNING = 255;

static constexpr int SERIALNUMBER_START_INDEX = 3;
static constexpr int SERIALNUMBER_END_INDEX = 23;
static constexpr int MODELNUMBER_START_INDEX = 46;
static constexpr int MODELNUMBER_END_INDEX = 85;

static constexpr const int TEMPERATURE_SENSOR_FAILURE = 0x81;

static std::map<std::string, std::string> map_vendor = {
    {"80 86", "Intel"}, {"1e f", "Kioxia"}, {"14 4d", "Samsung"}};

namespace fs = std::filesystem;

namespace phosphor
{
namespace nvme
{

using namespace std;
using namespace phosphor::logging;

void Nvme::setNvmeInventoryProperties(
    NVMeConfig& config, bool present,
    const phosphor::nvme::Nvme::NVMeData& nvmeData,
    const std::string& inventoryPath)
{
    static std::unordered_map<int, std::string> preSerial;
    static std::unordered_map<int, std::string> preSmartWarning;
    static std::unordered_map<int, std::string> preStatusFlags;

    if (preSerial[config.busID].compare(nvmeData.serialNumber) != 0)
    {
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     ITEM_IFACE, "Present", present);
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     ASSET_IFACE, "Manufacturer",
                                     nvmeData.vendor);
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     ASSET_IFACE, "SerialNumber",
                                     nvmeData.serialNumber);
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     ASSET_IFACE, "Model",
                                     nvmeData.modelNumber);
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     NVME_STATUS_IFACE, "DriveLifeUsed",
                                     nvmeData.driveLifeUsed);
        preSerial[config.busID] = nvmeData.serialNumber;
    }

    if (preStatusFlags[config.busID].compare(nvmeData.statusFlags) != 0)
    {
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     NVME_STATUS_IFACE, "StatusFlags",
                                     nvmeData.statusFlags);
        preStatusFlags[config.busID] = nvmeData.statusFlags;
    }

    if (preSmartWarning[config.busID].compare(nvmeData.smartWarnings) != 0)
    {
        util::SDBusPlus::setProperty(bus, INVENTORY_BUSNAME, inventoryPath,
                                     NVME_STATUS_IFACE, "SmartWarnings",
                                     nvmeData.smartWarnings);
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
        preSmartWarning[config.busID] = nvmeData.smartWarnings;
    }
}

void Nvme::setFaultLED(const std::string& locateLedGroupPath,
                       const std::string& faultLedGroupPath, bool request)
{
    if (locateLedGroupPath.empty() || faultLedGroupPath.empty())
    {
        return;
    }

    // Before toggle LED, check whether is Identify or not.
    if (!getLEDGroupState(locateLedGroupPath))
    {
        if (getLEDGroupState(faultLedGroupPath) != request)
        {
            util::SDBusPlus::setProperty(bus, LED_GROUP_BUSNAME,
                                         faultLedGroupPath, LED_GROUP_IFACE,
                                         "Asserted", request);
        }
    }
}

void Nvme::setLocateLED(const std::string& locateLedGroupPath,
                        const std::string& locateLedBusName,
                        const std::string& locateLedPath, bool isPresent)
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
        isError[config.index] = false;
    }
    else
    {
        if (isError[config.index] != true)
        {
            // Drive is present but can not get data, turn on fault LED.
            log<level::ERR>("Drive status is good but can not get data.",
                            entry("OBJ_PATH=%s", config.index.c_str()));
            isError[config.index] = true;
        }

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
bool Nvme::getNVMeInfobyBusID(int busID,
                              phosphor::nvme::Nvme::NVMeData& nvmeData)
{
    nvmeData.present = true;
    nvmeData.vendor = "";
    nvmeData.serialNumber = "";
    nvmeData.modelNumber = "";
    nvmeData.smartWarnings = "";
    nvmeData.statusFlags = "";
    nvmeData.driveLifeUsed = "";
    nvmeData.sensorValue = static_cast<int8_t>(TEMPERATURE_SENSOR_FAILURE);
    nvmeData.wcTemp = 0;

    phosphor::smbus::Smbus smbus;

    unsigned char rsp_data_command_0[I2C_DATA_MAX] = {0};
    unsigned char rsp_data_command_8[I2C_DATA_MAX] = {0};

    uint8_t tx_data = COMMAND_CODE_0;

    auto init = smbus.smbusInit(busID);

    if (init == -1)
    {
        if (isErrorSmbus[busID] != true)
        {
            log<level::ERR>("smbusInit fail!");
            isErrorSmbus[busID] = true;
        }

        nvmeData.present = false;

        return nvmeData.present;
    }

    auto res_int = smbus.SendSmbusRWCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS,
                                           &tx_data, sizeof(tx_data),
                                           rsp_data_command_0, CODE_0_LENGTH);

    if (res_int < 0)
    {
        if (isErrorSmbus[busID] != true)
        {
            log<level::ERR>("Send command code 0 fail!");
            isErrorSmbus[busID] = true;
        }

        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    if (rsp_data_command_0[1] & DriveNotReadyMask)
    {
        if (isErrorSmbus[busID] != true)
        {
            log<level::ERR>("Drive not ready!");
            isErrorSmbus[busID] = true;
        }
        smbus.smbusClose(busID);
        return false;
    }

    nvmeData.statusFlags = intToHex(rsp_data_command_0[1]);
    nvmeData.smartWarnings = intToHex(rsp_data_command_0[2]);
    nvmeData.driveLifeUsed = intToHex(rsp_data_command_0[4]);
    nvmeData.sensorValue = static_cast<int8_t>(rsp_data_command_0[3]);
    nvmeData.wcTemp = static_cast<int8_t>(rsp_data_command_0[5]);

    tx_data = COMMAND_CODE_8;

    res_int = smbus.SendSmbusRWCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS, &tx_data,
                                      sizeof(tx_data), rsp_data_command_8,
                                      CODE_8_LENGTH);

    if (res_int < 0)
    {
        if (isErrorSmbus[busID] != true)
        {
            log<level::ERR>("Send command code 8 fail!");
            isErrorSmbus[busID] = true;
        }

        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    nvmeData.vendor = intToHex(rsp_data_command_8[1]) + " " +
                      intToHex(rsp_data_command_8[2]);

    for (auto iter = map_vendor.begin(); iter != map_vendor.end(); iter++)
    {
        if (iter->first == nvmeData.vendor)
        {
            nvmeData.vendor = iter->second;
            break;
        }
    }

    for (int offset = SERIALNUMBER_START_INDEX; offset < SERIALNUMBER_END_INDEX;
         offset++)
    {
        // Only accept digits/letters/punctuation characters.
        if (rsp_data_command_8[offset] >= '!' &&
            rsp_data_command_8[offset] <= '~')
            nvmeData.serialNumber +=
                static_cast<char>(rsp_data_command_8[offset]);
    }

    if ((nvmeData.vendor == "Samsung") || (nvmeData.vendor == "Kioxia"))
    {
        unsigned char rsp_data_vpd[I2C_DATA_MAX] = {0};
        const int rx_len = (MODELNUMBER_END_INDEX - MODELNUMBER_START_INDEX);
        tx_data = MODELNUMBER_START_INDEX;

        auto res_int =
            smbus.SendSmbusRWCmdRAW(busID, NVME_SSD_VPD_SLAVE_ADDRESS, &tx_data,
                                    sizeof(tx_data), rsp_data_vpd, rx_len);

        if (res_int < 0)
        {
            if (isErrorSmbus[busID] != true)
            {
                log<level::ERR>("Send command read VPD fail!");
                isErrorSmbus[busID] = true;
            }

            smbus.smbusClose(busID);
            nvmeData.present = false;
            return nvmeData.present;
        }

        for (int i = 0; i < rx_len; i++)
        {
            // Only accept digits/letters/punctuation characters.
            if ((rsp_data_vpd[i] >= '!' && rsp_data_vpd[i] <= '~'))
                nvmeData.modelNumber += static_cast<char>(rsp_data_vpd[i]);
        }

        if (nvmeData.modelNumber.substr(0, nvmeData.vendor.size()) == "SAMSUNG")
            nvmeData.modelNumber.erase(0, nvmeData.vendor.size());
    }

    smbus.smbusClose(busID);

    isErrorSmbus[busID] = false;

    return nvmeData.present;
}

void Nvme::run()
{
    init();

    std::function<void()> callback(std::bind(&Nvme::read, this));
    try
    {
        u_int64_t interval = monitorIntervalSec * 1000000;
        _timer.restart(std::chrono::microseconds(interval));
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("Error in polling loop. "), entry("ERROR=%s", e.what());
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
std::vector<phosphor::nvme::Nvme::NVMeConfig> Nvme::getNvmeConfig()
{
    phosphor::nvme::Nvme::NVMeConfig nvmeConfig;
    std::vector<phosphor::nvme::Nvme::NVMeConfig> nvmeConfigs;
    int8_t criticalHigh = 0;
    int8_t criticalLow = 0;
    int8_t maxValue = 0;
    int8_t minValue = 0;
    int8_t warningHigh = 0;
    int8_t warningLow = 0;

    try
    {
        auto data = parseSensorConfig();
        static const std::vector<Json> empty{};
        std::vector<Json> readings = data.value("config", empty);
        std::vector<Json> thresholds = data.value("threshold", empty);
        monitorIntervalSec = data.value("monitorIntervalSec",
                                        MONITOR_INTERVAL_SECONDS);
        maxSmbusErrorRetry = data.value("maxSmbusErrorRetry",
                                        MAX_SMBUS_ERROR_RETRY);

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
                uint16_t presentPin = instance.value("NVMeDrivePresentPin", 0);
                uint16_t pwrGoodPin = instance.value("NVMeDrivePwrGoodPin", 0);
                std::string locateLedControllerBusName =
                    instance.value("NVMeDriveLocateLEDControllerBusName", "");
                std::string locateLedControllerPath =
                    instance.value("NVMeDriveLocateLEDControllerPath", "");

                nvmeConfig.index = std::to_string(index);
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
        log<level::ERR>("Json Exception caught."), entry("MSG=%s", e.what());
    }

    return nvmeConfigs;
}

std::string Nvme::getGPIOValueOfNvme(const std::string& fullPath)
{
    std::string val;
    std::ifstream ifs;
    auto retries = 3;

    while (retries != 0)
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
                            entry("MSG=%s", e.what()));
            continue;
        }
        break;
    }

    ifs.close();
    return val;
}

void Nvme::createNVMeInventory()
{
    using Properties = std::map<std::string, std::variant<std::string, bool>>;
    using Interfaces = std::map<std::string, Properties>;

    std::string inventoryPath;
    std::map<sdbusplus::message::object_path, Interfaces> obj;

    for (const auto& config : configs)
    {
        inventoryPath = "/system/chassis/motherboard/nvme" + config.index;

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
    createNVMeInventory();
}

void Nvme::readNvmeData(NVMeConfig& config, bool isPwrGood)
{
    std::string inventoryPath = NVME_INVENTORY_PATH + config.index;
    NVMeData nvmeData;

    // get NVMe information through i2c by busID.
    bool success;

    // skip reading nvme data when power good is false
    if (isPwrGood)
    {
        success = getNVMeInfobyBusID(config.busID, nvmeData);
    }
    else
    {
        nvmeData.present = false;
        nvmeData.sensorValue = static_cast<int8_t>(TEMPERATURE_SENSOR_FAILURE);
        success = false;
        // Skip retry below when isPwrGood is false because smbus is going to
        // fail
        nvmeSmbusErrCnt[config.busID] = maxSmbusErrorRetry;
    }

    if (success)
    {
        nvmeSmbusErrCnt[config.busID] = 0;
    }
    else
    {
        if (nvmeSmbusErrCnt[config.busID] < maxSmbusErrorRetry)
        {
            // Return early so that we retry
            nvmeSmbusErrCnt[config.busID]++;
            log<level::INFO>("getNVMeInfobyBusID failed, retry...",
                             entry("INDEX=%s", config.index.c_str()),
                             entry("ERRCNT=%u", nvmeSmbusErrCnt[config.busID]));
            return;
        }
    }

    // find NvmeSSD object by index
    auto iter = nvmes.find(config.index);

    // can not find. create dbus
    if (iter == nvmes.end())
    {
        log<level::INFO>("SSD plug.", entry("INDEX=%s", config.index.c_str()));

        std::string objPath = NVME_OBJ_PATH + config.index;
        auto nvmeSSD =
            std::make_shared<phosphor::nvme::NvmeSSD>(bus, objPath.c_str());
        nvmes.emplace(config.index, nvmeSSD);

        setNvmeInventoryProperties(config, true, nvmeData, inventoryPath);
        nvmeSSD->setSensorValueToDbus(nvmeData.sensorValue);
        if (nvmeData.wcTemp != 0)
        {
            config.criticalHigh = nvmeData.wcTemp;
            config.warningHigh = nvmeData.wcTemp;
        }
        nvmeSSD->setSensorMaxMin(config.maxValue, config.minValue);
        nvmeSSD->setSensorThreshold(config.criticalHigh, config.criticalLow,
                                    config.warningHigh, config.warningLow);

        nvmeSSD->checkSensorThreshold();
        setLEDsStatus(config, success, nvmeData);
    }
    else
    {
        setNvmeInventoryProperties(config, true, nvmeData, inventoryPath);
        iter->second->setSensorValueToDbus(nvmeData.sensorValue);
        if (nvmeData.wcTemp != 0)
        {
            config.criticalHigh = nvmeData.wcTemp;
            config.warningHigh = nvmeData.wcTemp;

            iter->second->setSensorThreshold(
                config.criticalHigh, config.criticalLow, config.warningHigh,
                config.warningLow);
        }
        iter->second->checkSensorThreshold();
        setLEDsStatus(config, success, nvmeData);
    }
}

/** @brief Monitor NVMe drives every one second  */
void Nvme::read()
{
    std::string devPresentPath;
    std::string devPwrGoodPath;
    std::string inventoryPath;

    for (auto config : configs)
    {
        NVMeData nvmeData;

        inventoryPath = NVME_INVENTORY_PATH + config.index;
        devPresentPath = GPIO_BASE_PATH + std::to_string(config.presentPin) +
                         "/value";
        devPwrGoodPath = GPIO_BASE_PATH + std::to_string(config.pwrGoodPin) +
                         "/value";

        auto presentPinValStr = (config.presentPin)
                                    ? getGPIOValueOfNvme(devPresentPath)
                                    : IS_PRESENT;
        auto pwrGoodPinValStr =
            (config.pwrGoodPin) ? getGPIOValueOfNvme(devPwrGoodPath) : POWERGD;
        const bool isPwrGood = (pwrGoodPinValStr == POWERGD);

        if (presentPinValStr != IS_PRESENT)
        {
            // Drive not present, remove nvme d-bus path ,
            // clean all properties in inventory
            // and turn off fault and locate LED

            setFaultLED(config.locateLedGroupPath, config.faultLedGroupPath,
                        false);
            setLocateLED(config.locateLedGroupPath,
                         config.locateLedControllerBusName,
                         config.locateLedControllerPath, false);

            nvmeData = NVMeData();
            setNvmeInventoryProperties(config, false, nvmeData, inventoryPath);
            nvmes.erase(config.index);
            continue;
        }

        if (!isPwrGood)
        {
            // IFDET should be used to provide the final say
            // in SSD's presence - IFDET showing SSD is present
            // but the power is off (if the drive is plugged in)
            // is a valid state.

            setFaultLED(config.locateLedGroupPath, config.faultLedGroupPath,
                        true);
            setLocateLED(config.locateLedGroupPath,
                         config.locateLedControllerBusName,
                         config.locateLedControllerPath, false);

            nvmeData = NVMeData();
            setNvmeInventoryProperties(config, true, nvmeData, inventoryPath);

            if (isErrorPower[config.index] != true)
            {
                log<level::ERR>(
                    "Present pin is true but power good pin is false.",
                    entry("INDEX=%s", config.index.c_str()));
                log<level::ERR>("Erase SSD from map and d-bus.",
                                entry("INDEX=%s", config.index.c_str()));

                isErrorPower[config.index] = true;
            }
        }
        else
        {
            isErrorPower[config.index] = false;
        }

        // Keep reading to report the invalid temperature
        // (To make thermal loop know that the sensor reading
        //  is invalid).
        readNvmeData(config, isPwrGood);
        if (nvmes.find(config.index) != nvmes.end())
        {
            nvmes.find(config.index)->second->setSensorAvailability(isPwrGood);
        }
    }
}
} // namespace nvme
} // namespace phosphor
