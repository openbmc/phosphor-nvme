#include "nvme_manager.hpp"
#include "smbus.hpp"

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/message.hpp>
#include <sstream>
#include <string>

#include "i2c.h"
#define MONITOR_INTERVAL_SECONDS 1
#define NVME_SSD_SLAVE_ADDRESS 0x6a
#define PRESENT 1

static constexpr auto configFile = "/etc/nvme/mihawk_config.json";
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

static constexpr const int TEMPERATURE_SENSOR_FAILURE = 0x81;

namespace fs = std::filesystem;

namespace phosphor
{
namespace nvme
{

using namespace std;
using namespace phosphor::logging;

void Nvme::setNvmeInventoryProperties(
    bool present, const phosphor::nvme::Nvme::NVMeData& nvmeData,
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

std::string intToHex(int input)
{
    std::stringstream tmp;
    tmp << std::hex << input;

    return tmp.str();
}

/** @brief Get NVMe info over smbus  */
bool getNVMeInfobyBusID(int busID, int address, int channel,
                        phosphor::nvme::Nvme::NVMeData& nvmeData)
{
    nvmeData.present = true;
    nvmeData.vendor = "";
    nvmeData.serialNumber = "";
    nvmeData.smartWarnings = "";
    nvmeData.statusFlags = "";
    nvmeData.driveLifeUsed = "";
    nvmeData.sensorValue = (int8_t)TEMPERATURE_SENSOR_FAILURE;

    phosphor::smbus::Smbus smbus;

    unsigned char rsp_data_command_0[I2C_DATA_MAX] = {0};
    unsigned char rsp_data_command_8[I2C_DATA_MAX] = {0};

    uint8_t tx_data = COMMAND_CODE_0;

    auto init = smbus.smbusInit(busID);

    static std::unordered_map<int, bool> isErrorSmbus;

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
    smbus.SetSmbusCmdByte(busID, address, 0, channel);
    auto res_int =
        smbus.SendSmbusRWBlockCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS, &tx_data,
                                     sizeof(tx_data), rsp_data_command_0);

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

    tx_data = COMMAND_CODE_8;

    res_int =
        smbus.SendSmbusRWBlockCmdRAW(busID, NVME_SSD_SLAVE_ADDRESS, &tx_data,
                                     sizeof(tx_data), rsp_data_command_8);

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
    nvmeData.sensorValue = (int8_t)rsp_data_command_0[3];

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
                uint8_t address = instance.value("NVMeAddress", 0);
                uint8_t channel = instance.value("NVMeChannel", 0);

                nvmeConfig.index = std::to_string(index);
                nvmeConfig.busID = busID;
                nvmeConfig.address = address;
                nvmeConfig.channel = channel;
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
                            entry("MSG: %s", e.what()));
            continue;
        }
        break;
    }

    ifs.close();
    return val;
}

#define MAX_SUPPORT_NVME 24
#define NVME_PER_MUX 8
#define MUX_BUS 12
#define MUX_SWITCH_COMMAND 0x00
#define MUX_SWITCH_ADDR 0x70
#define MUX_ADDR 0x40
#define CPLD_PRESENT_COMMAND 0x07
#define CPLD_IFDET_COMMAND 0x09

/*This function intends to test existence of NVMe driver via I2C.
We can tell what kind of device is connected by reading driverPresent and
driverIfdet.

drivePresent  | driveIfdet  | Type
-------------------------------------------
      0       |      0      | SAS/SATA HDD
-------------------------------------------
      1       |      0      | NVMe SSD
-------------------------------------------
      1       |      1      | NC
*/
int getNVMePresent(const unsigned int nvmeDriverIndex)
{
    int val = 0;
    unsigned char present = 0;
    unsigned char ifdet = 0;
    unsigned char drivePresent = 0;
    unsigned char driveIfdet = 0;
    unsigned char muxNum = nvmeDriverIndex / NVME_PER_MUX;
    unsigned char muxChannel = nvmeDriverIndex % NVME_PER_MUX;
    char filename[20];
    auto bus = phosphor::smbus::Smbus();

    if (nvmeDriverIndex > MAX_SUPPORT_NVME - 1)
    {
        std::cerr << "nvmeDriverIndex over restriction\n";
        return 0;
    }

    int fd = bus.openI2cDev(MUX_BUS, filename, sizeof(filename), 0);
    if (fd < 0)
    {
        std::cerr << "Failed to open CPLD\n";
        return 0;
    }

    if (ioctl(fd, I2C_SLAVE_FORCE, MUX_SWITCH_ADDR) < 0)
    {
        std::cerr << "Failed to switch mux\n";
        goto exit_getNvmePresent;
    }
    i2c_smbus_write_byte_data(fd, MUX_SWITCH_COMMAND, (0x01 << muxNum));

    if (ioctl(fd, I2C_SLAVE_FORCE, MUX_ADDR) < 0)
    {
        std::cerr << "Failed to set CPLD addr\n";
        goto exit_getNvmePresent;
    }

    present = i2c_smbus_read_byte_data(fd, CPLD_PRESENT_COMMAND);
    ifdet = i2c_smbus_read_byte_data(fd, CPLD_IFDET_COMMAND);

    drivePresent = (present >> muxChannel) & 0x01;
    driveIfdet = (ifdet >> muxChannel) & 0x01;

    /*NVMe present*/
    if ((drivePresent == 1) && (driveIfdet == 0))
    {
        val = 1;
    }

exit_getNvmePresent:
    close(fd);
    return val;
}

void Nvme::createNVMeInventory()
{
    using Properties =
        std::map<std::string, sdbusplus::message::variant<std::string, bool>>;
    using Interfaces = std::map<std::string, Properties>;

    std::string inventoryPath;
    std::map<sdbusplus::message::object_path, Interfaces> obj;

    for (const auto config : configs)
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

/** @brief Monitor NVMe drives every one second  */
void Nvme::read()
{
    std::string inventoryPath;

    for (auto config : configs)
    {
        NVMeData nvmeData;

        inventoryPath = NVME_INVENTORY_PATH + config.index;

        auto iter = nvmes.find(config.index);

        int a = std::stoi(config.index);

        if (getNVMePresent(a) == PRESENT)
        {
            // Drive status is good, update value or create d-bus and update
            // value.

            // get NVMe information through i2c by busID.
            auto success = getNVMeInfobyBusID(config.busID, config.address,
                                              config.channel, nvmeData);
            // can not find. create dbus
            if (iter == nvmes.end())
            {
                log<level::INFO>("SSD plug.",
                                 entry("index = %s", config.index.c_str()));
                std::string objPath = NVME_OBJ_PATH + config.index;
                auto nvmeSSD = std::make_shared<phosphor::nvme::NvmeSSD>(
                    bus, objPath.c_str());
                nvmes.emplace(config.index, nvmeSSD);

                setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                nvmeSSD->setSensorValueToDbus(nvmeData.sensorValue);
                nvmeSSD->setSensorThreshold(
                    config.criticalHigh, config.criticalLow, config.maxValue,
                    config.minValue, config.warningHigh, config.warningLow);

                nvmeSSD->checkSensorThreshold();
            }
            else
            {
                setNvmeInventoryProperties(true, nvmeData, inventoryPath);
                iter->second->setSensorValueToDbus(nvmeData.sensorValue);
                iter->second->checkSensorThreshold();
            }
        }
        else
        {
            // Drive not present, remove nvme d-bus path ,
            // clean all properties in inventory
            // and turn off fault and locate LED

            nvmeData = NVMeData();
            setNvmeInventoryProperties(false, nvmeData, inventoryPath);
            nvmes.erase(config.index);
        }
    }
}
} // namespace nvme
} // namespace phosphor
