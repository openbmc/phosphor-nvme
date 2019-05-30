#include "nvme_manager.hpp"
#include "nlohmann/json.hpp"
#include "smbus.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sstream>
#include <string>
#include "i2c-dev.h"
#include <xyz/openbmc_project/Led/Physical/server.hpp>

#define MAX_I2C_BUS 30
#define MONITOR_INTERVAL_SENCODS 1
#define NVME_SSD_SLAVE_ADDRESS 0x6a
#define GPIO_BASE_PATH "/sys/class/gpio/gpio"
#define IS_PRESENT "0"
#define POWERGD "1"

static int fd[MAX_I2C_BUS] = {0};
static constexpr auto configFile = "/etc/nvme/nvme_config.json";
auto retries = 3;
static constexpr auto delay = std::chrono::milliseconds{100};
using Json = nlohmann::json;
std::vector<phosphor::nvme::Nvme::NVMeConfig> configs;
namespace fs = std::experimental::filesystem;

namespace phosphor
{
namespace nvme
{

using namespace std;
using namespace phosphor::logging;

void Nvme::assertFaultLog(int smartWarning, std::string inventoryPath)
{
    bool bit;

    for (int i = 0; i < 5; i++)
    {
        if (((smartWarning >> i) & 1) == 0)
            bit = true;
        else
            bit = false;

        switch (i)
        {
            case 0:
                setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "CapacityFault", bit);
                break;
            case 1:
                setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "TemperatureFault", bit);
                break;
            case 2:
                setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "DegradesFault", bit);
                break;
            case 3:
                setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "MediaFault", bit);
                break;
            case 4:
                setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "BackupDeviceFault", bit);
                break;
        }
    }
}

void Nvme::setNvmeInventoryProperties(bool present, phosphor::nvme::Nvme::NVMeData nvmeData, std::string inventoryPath)
{
    setInventoryParm(inventoryPath, ITEM_IFACE, "Present", present);
    setInventoryParm(inventoryPath, ASSET_IFACE, "Manufacturer", nvmeData.vendor);
    setInventoryParm(inventoryPath, ASSET_IFACE, "SerialNumber", nvmeData.serialNumber);
    setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "SmartWarnings", nvmeData.smartWarnings);
    setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "StatusFlags", nvmeData.statusFlags);
    setInventoryParm(inventoryPath, NVME_STATUS_IFACE, "DriveLifeUsed", nvmeData.driveLifeUsed);
}

template <typename T>
void Nvme::setInventoryParm(std::string &objPath, const std::string &interface, const std::string &property, const T &value)
{

    sdbusplus::message::variant<T> data = value;

    try{
        auto methodCall = bus.new_method_call(INVENTORY_BUSNAME, objPath.c_str(),
                                            DBUS_PROPERTY_IFACE, "Set");

        methodCall.append(interface.c_str());
        methodCall.append(property);
        methodCall.append(data);

        auto reply = bus.call(methodCall);

    }catch (const std::exception &e)
    {
        std::cerr << "Call method fail: set inventory properties. ERROR = " << e.what() << std::endl;
        std::cerr << "objPath = " << objPath << std::endl;
        return ;
    }
}

void Nvme::checkAssertFaultLED(std::string &locateLedGroupPath, std::string &faultLedGroupPath, bool request)
{
    if(locateLedGroupPath.empty() || faultLedGroupPath.empty())
    {
        return;
    }

    if( !getLEDGroupState(locateLedGroupPath)) // Before asserted LED, check whether is Identify or not.
    {
        setFaultLED("Asserted", request, faultLedGroupPath);
    }
}

void Nvme::checkAssertLocateLED(std::string &locateLedGroupPath, std::string &locateLedBusName, std::string &locateLedPath , bool isPresent)
{
    if(locateLedGroupPath.empty() || locateLedBusName.empty() || locateLedPath.empty())
    {
        return;
    }

    namespace server = sdbusplus::xyz::openbmc_project::Led::server;

    if( !getLEDGroupState(locateLedGroupPath))
    {
        if(isPresent)
            setLocateLED("State", server::convertForMessage(server::Physical::Action::On), locateLedBusName, locateLedPath);
        else
            setLocateLED("State", server::convertForMessage(server::Physical::Action::Off), locateLedBusName, locateLedPath);
    }

}

bool Nvme::getLEDGroupState(std::string &ledPath)
{
    std::string obj_path;
    obj_path = ledPath;
    bool asserted = false;

    try{
        auto method = bus.new_method_call(LED_GROUP_BUSNAME, obj_path.c_str(),
                                      DBUS_PROPERTY_IFACE, "GetAll");

        method.append(LED_GROUP_IFACE);
        auto reply = bus.call(method);

        std::map<std::string, variant<bool>> properties;
        reply.read(properties);

        asserted = get<bool>(properties.at("Asserted"));

    }catch (const sdbusplus::exception::SdBusError& ex)
    {
        std::cerr << "Call method fail: Error in get LED group status. ERROR = " << ex.what() << std::endl;
        std::cerr << "path = " << ledPath << std::endl;
    }
    return asserted;
}

template <typename T>
void Nvme::setFaultLED(const std::string &property, const T &value, std::string &ledPath)
{
    if(ledPath.empty())
        return;

    sdbusplus::message::variant<bool> data = value;
    std::string obj_path;
    obj_path = ledPath;

    try{
        auto methodCall = bus.new_method_call(LED_GROUP_BUSNAME, obj_path.c_str(),
                                            DBUS_PROPERTY_IFACE, "Set");

        methodCall.append(LED_GROUP_IFACE);
        methodCall.append(property);
        methodCall.append(data);

        auto reply = bus.call(methodCall);

    }catch (const sdbusplus::exception::SdBusError& ex)
    {
        std::cerr << "Call method fail: set fault LED Aasserted. ERROR = " << ex.what() << std::endl;
        std::cerr << "path = " << ledPath << std::endl;
        return ;
    }
}

template <typename T>
void Nvme::setLocateLED(const std::string& property, const T& value, std::string &locateLedBusName, std::string &locateLedPath)
{
    sdbusplus::message::variant<T> data = value;
    std::string obj_path;
    obj_path = locateLedPath;

    try{
        auto methodCall = bus.new_method_call(locateLedBusName.c_str(), obj_path.c_str(),
                                                DBUS_PROPERTY_IFACE, "Set");

        methodCall.append(LED_CONTROLLER_IFACE);
        methodCall.append(property);
        methodCall.append(data);

        auto reply = bus.call(methodCall);

    }catch (const sdbusplus::exception::SdBusError& ex)
    {
        std::cerr << "Call method fail: set locate LED state. ERROR = " << ex.what() << std::endl;
        std::cerr << "path = " << locateLedPath << std::endl;
        return ;
    }
}

std::string intToHex(int input)
{
    std::stringstream tmp;
    tmp << std::hex << input;

    return tmp.str();
}

bool getNVMeInfobyBusID(int busID, phosphor::nvme::Nvme::NVMeData &nvmeData)
{
    nvmeData.present = true;
    nvmeData.vendor = "";
    nvmeData.serialNumber = "";
    nvmeData.smartWarnings = "";
    nvmeData.statusFlags = "";
    nvmeData.driveLifeUsed = "";
    nvmeData.sensorValue = 129;

    phosphor::smbus::Smbus smbus;

    unsigned char rsp_data_command_0[8] = {0};
    unsigned char rsp_data_command_8[24] = {0};

    uint8_t in_data = 0; // command code

    auto init = smbus.smbusInit(busID);

    if (init == -1)
    {
        std::cerr << "smbusInit fail!" << std::endl;

        nvmeData.present = false;

        return nvmeData.present;
    }

    auto res_int = smbus.SendSmbusRWBlockCmdRAW(
        busID, NVME_SSD_SLAVE_ADDRESS, &in_data, 1, rsp_data_command_0);

    if (res_int < 0)
    {
        std::cerr << "Send command code 0 fail!" << std::endl;

        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    in_data = 8; // command code

    res_int = smbus.SendSmbusRWBlockCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS,
                                           &in_data, 1, rsp_data_command_8);

    if (res_int < 0)
    {
        std::cerr << "Send command code 8 fail!" << std::endl;
        smbus.smbusClose(busID);
        nvmeData.present = false;
        return nvmeData.present;
    }

    nvmeData.vendor =
        intToHex(rsp_data_command_8[1]) + " " + intToHex(rsp_data_command_8[2]);
    for (int i = 3; i < 23; i++) // i: serialNumber position
    {
        nvmeData.serialNumber += static_cast<char>(rsp_data_command_8[i]);
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
        u_int64_t interval = MONITOR_INTERVAL_SENCODS * 1000000;
        _timer.restart(std::chrono::microseconds(interval));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in polling loop. ERROR = " << e.what() << std::endl;
    }
}

Json parseSensorConfig()
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        std::cerr << "NVMe config JSON file not found" << std::endl;
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        std::cerr << "NVMe config readings JSON parser failure" << std::endl;
    }

    return data;
}

std::vector<phosphor::nvme::Nvme::NVMeConfig> getNvmeConfig()
{

    phosphor::nvme::Nvme::NVMeConfig nvmeConfig;
    std::vector<phosphor::nvme::Nvme::NVMeConfig> nvmeConfigs;
    uint64_t criticalHigh = 0;
    uint64_t criticalLow = 0;
    uint64_t maxValue = 0;
    uint64_t minValue = 0;

    try
    {
        auto data = parseSensorConfig();
        static const std::vector<Json> empty{};
        std::vector<Json> readings = data.value("config", empty);
        std::vector<Json> thresholds = data.value("threshold", empty);
        if (!thresholds.empty())
        {
            for (const auto &instance : thresholds)
            {
                criticalHigh = instance.value("criticalHigh", 0);
                criticalLow = instance.value("criticalLow", 0);
                maxValue = instance.value("maxValue", 0);
                minValue = instance.value("minValue", 0);
            }
        }
        else
        {
            std::cerr << "Invalid NVMe config file, thresholds dosen't exist" << std::endl;
        }

        if (!readings.empty())
        {
            for (const auto &instance : readings)
            {
                uint8_t index = instance.value("NvmeDriveIndex", 0);
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
                nvmeConfig.maxValue = maxValue;
                nvmeConfig.minValue = minValue;
                nvmeConfigs.push_back(nvmeConfig);
            }
        }
        else
        {
            std::cerr << "Invalid NVMe config file, config dosen't exist" << std::endl;
        }
    }
    catch (const Json::exception &e)
    {
        std::cerr << "Json Exception caught. MSG: " << e.what() << std::endl;
    }

    return nvmeConfigs;
}

std::string Nvme::getValue(std::string fullPath)
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
        catch (const std::exception &e)
        {
            --retries;
            std::this_thread::sleep_for(delay);
            std::cerr << "Can not open gpio path MSG: " << e.what() << std::endl;
            continue;
        }
        break;
    }

    ifs.close();
    return val;
}

void Nvme::init()
{
    // read json file
    configs = getNvmeConfig();
}

void Nvme::setSSDLEDStatus(std::shared_ptr<phosphor::nvme::NvmeSSD> nvmeSSD,
                           phosphor::nvme::Nvme::NVMeConfig config,
                           bool success,
                           phosphor::nvme::Nvme::NVMeData nvmeData)
{
    if (success)
    {
        if (!nvmeData.smartWarnings.empty())
        {
            std::string inventoryPath = INVENTORY_PATH + std::to_string(config.index);
            assertFaultLog(std::stoi(nvmeData.smartWarnings, 0, 16), inventoryPath);
            auto request = (strcmp(nvmeData.smartWarnings.c_str(), "ff") == 0)
                               ? false
                               : true;
            checkAssertFaultLED(config.locateLedGroupPath,
                                    config.faultLedGroupPath, request);
            checkAssertLocateLED(config.locateLedGroupPath,
                                     config.locateLedControllerBusName,
                                     config.locateLedControllerPath, !request);
        }
    }
    else
    {
        // Drive is present but can not get data, turn on fault LED.
        std::cerr << "Drive status is good but can not get data. index = " << std::to_string(config.index) << std::endl;
        checkAssertFaultLED(config.locateLedGroupPath,
                                config.faultLedGroupPath, true);
        checkAssertLocateLED(config.locateLedGroupPath,
                                 config.locateLedControllerBusName,
                                 config.locateLedControllerPath, false);
    }
}



void Nvme::read()
{
    std::string devPresentPath;
    std::string devPwrGoodPath;
    std::string inventoryPath;

        for (int i = 0; i < configs.size(); i++)
        {
            NVMeData nvmeData;
            devPresentPath =
                GPIO_BASE_PATH + std::to_string(configs[i].presentPin) + "/value";

            devPwrGoodPath =
                GPIO_BASE_PATH + std::to_string(configs[i].pwrGoodPin) + "/value";

            inventoryPath = INVENTORY_PATH + std::to_string(configs[i].index);

            auto iter = nvmes.find(std::to_string(configs[i].index));

            if (getValue(devPresentPath) == IS_PRESENT)
            {
                // Drive status is good, update value or create d-bus and update
                // value.
                if (getValue(devPwrGoodPath) == POWERGD)
                {
                    // get NVMe information through i2c by busID.
                    auto success = getNVMeInfobyBusID(configs[i].busID, nvmeData);
                    // can not find. create dbus
                    if (iter == nvmes.end())
                    {
                        std::cerr << "SSD plug. index = "<< std::to_string(configs[i].index) << std::endl;

                        std::string objPath =
                            NVME_OBJ_PATH + std::to_string(configs[i].index);
                        auto nvmeSSD = std::make_shared<phosphor::nvme::NvmeSSD>(
                            bus, objPath.c_str());
                        nvmes.emplace(std::to_string(configs[i].index), nvmeSSD);

                        setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                        nvmeSSD->setSensorValueToDbus(nvmeData.sensorValue);
                        nvmeSSD->setSensorThreshold(
                            configs[i].criticalHigh, configs[i].criticalLow,
                            configs[i].maxValue, configs[i].minValue);

                        nvmeSSD->checkSensorThreshold();
                        setSSDLEDStatus(nvmeSSD, configs[i], success, nvmeData);
                    }
                    else
                    {
                        setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                        iter->second->setSensorValueToDbus(nvmeData.sensorValue);

                        iter->second->checkSensorThreshold();
                        setSSDLEDStatus(iter->second, configs[i], success,
                                        nvmeData);
                    }
                }
                else
                {
                    // Present pin is true but power good pin is false
                    // remove nvme d-bus path, clean all properties in inventory
                    // and turn on fault LED
                    std::cerr << "Present pin is true but power good pin is false. index = "<< std::to_string(configs[i].index) << std::endl;

                    checkAssertFaultLED(
                        configs[i].locateLedGroupPath,
                        configs[i].faultLedGroupPath, true);
                    checkAssertLocateLED(
                        configs[i].locateLedGroupPath,
                        configs[i].locateLedControllerBusName,
                        configs[i].locateLedControllerPath, false);

                    nvmeData = NVMeData();
                    setNvmeInventoryProperties(false, nvmeData, inventoryPath);
                    nvmes.erase(std::to_string(configs[i].index));
                    std::cerr << "Erase SSD from map and d-bus. index = "<< std::to_string(configs[i].index) << std::endl;
                }
            }
            else
            {
                // Drive not present, remove nvme d-bus path ,
                // clean all properties in inventory
                // and turn off fault and locate LED

                checkAssertFaultLED(configs[i].locateLedGroupPath,
                                            configs[i].faultLedGroupPath,
                                            false);
                checkAssertLocateLED(
                    configs[i].locateLedGroupPath,
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
