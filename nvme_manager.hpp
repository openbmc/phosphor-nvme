#pragma once

#include "config.h"

#include "nvmes.hpp"
#include "sdbusplus.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server.hpp>
#include <sdbusplus/server/object.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>

#include <fstream>

namespace phosphor
{
namespace nvme
{

/** @class Nvme
 *  @brief Nvme manager implementation.
 */
class Nvme
{
  public:
    Nvme() = delete;
    Nvme(const Nvme&) = delete;
    Nvme& operator=(const Nvme&) = delete;
    Nvme(Nvme&&) = delete;
    Nvme& operator=(Nvme&&) = delete;

    /** @brief Constructs Nvme
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] objPath - The dbus path of nvme
     */
    Nvme(sdbusplus::bus_t& bus) :
        bus(bus), _event(sdeventplus::Event::get_default()),
        _timer(_event, std::bind(&Nvme::read, this))
    {
        // read json file
        configs = getNvmeConfig();
    }

    /**
     * Structure for keeping nvme configure data required by nvme monitoring
     */
    struct NVMeConfig
    {
        std::string index;
        uint8_t busID;
        std::string faultLedGroupPath;
        uint16_t presentPin;
        uint16_t pwrGoodPin;
        std::string locateLedControllerBusName;
        std::string locateLedControllerPath;
        std::string locateLedGroupPath;
        int8_t criticalHigh;
        int8_t criticalLow;
        int8_t maxValue;
        int8_t minValue;
        int8_t warningHigh;
        int8_t warningLow;
    };

    /**
     * Structure for keeping nvme data required by nvme monitoring
     */
    struct NVMeData
    {
        bool present;              /* Whether or not the nvme is present  */
        std::string vendor;        /* The nvme manufacturer  */
        std::string serialNumber;  /* The nvme serial number  */
        std::string modelNumber;   /* The nvme model number   */
        std::string smartWarnings; /* Indicates smart warnings for the state  */
        std::string statusFlags;   /* Indicates the status of the drives  */
        std::string
            driveLifeUsed;  /* A vendor specific estimate of the percentage  */
        int8_t sensorValue; /* Sensor value, if sensor value didn't be
                                  update, means sensor failure, default set to
                                  129(0x81) according to NVMe-MI SPEC*/
        int8_t wcTemp;      /* Indicates over temperature warning threshold.
                               This is intended to initially match the temperature
                               reported in the WCTEMP field in the NVMe Identify
                               Controller data structure */
    };

    /** @brief Setup polling timer in a sd event loop and attach to D-Bus
     *         event loop.
     */
    void run();

    /** @brief Get GPIO value of nvme by sysfs */
    std::string getGPIOValueOfNvme(const std::string& fullPath);
    /** @brief Map of the object NvmeSSD */
    std::unordered_map<std::string, std::shared_ptr<phosphor::nvme::NvmeSSD>>
        nvmes;

    /** @brief Set locate and fault LED status of SSD
     *
     * @param[in] config - Nvme configure data
     * @param[in] success - Success or not that get NVMe Info by SMbus
     * @param[in] nvmeData - Nvme information
     */
    void setLEDsStatus(const phosphor::nvme::Nvme::NVMeConfig& config,
                       bool success,
                       const phosphor::nvme::Nvme::NVMeData& nvmeData);

    /** @brief Set SSD fault LED status */
    void setFaultLED(const std::string& locateLedGroupPath,
                     const std::string& ledPath, bool request);
    /** @brief Set SSD locate LED status */
    void setLocateLED(const std::string& ledPath,
                      const std::string& locateLedBusName,
                      const std::string& locateLedPath, bool ispresent);
    /** @brief Get Identify State*/
    bool getLEDGroupState(const std::string& ledPath);

    /** @brief Set inventory properties of nvme */
    void setNvmeInventoryProperties(
        NVMeConfig& config, bool present,
        const phosphor::nvme::Nvme::NVMeData& nvmeData,
        const std::string& inventoryPath);

    void createNVMeInventory();

    /** @brief read NVMe information via Smbus */
    bool getNVMeInfobyBusID(int busID,
                            phosphor::nvme::Nvme::NVMeData& nvmeData);

    /** @brief read and update NVME data to dbus */
    void readNvmeData(NVMeConfig& config, bool isPwrGood);

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus_t& bus;
    /** @brief the Event Loop structure */
    sdeventplus::Event _event;
    /** @brief Read Timer */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    std::vector<phosphor::nvme::Nvme::NVMeConfig> configs;

    /** @brief error status for Smbus */
    std::unordered_map<int, bool> isErrorSmbus;
    /** @brief error status for LED */
    std::unordered_map<std::string, bool> isError;
    /** @brief error status for NVMe power*/
    std::unordered_map<std::string, bool> isErrorPower;

    /** @brief Set up initial configuration value of NVMe */
    void init();
    /** @brief Monitor NVMe drives every one second  */
    void read();

    std::vector<phosphor::nvme::Nvme::NVMeConfig> getNvmeConfig();

    /** @brief Monitor interval in second  */
    size_t monitorIntervalSec;
    /** @brief Maximum smbus error retry  */
    uint16_t maxSmbusErrorRetry;
    /** @brief Map of each NVMe smbus error count */
    std::unordered_map<int, uint16_t> nvmeSmbusErrCnt;
};
} // namespace nvme
} // namespace phosphor
