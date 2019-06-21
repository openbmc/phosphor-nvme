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
     * @param[in] objPath - The Dbus path of nvme
     */
    Nvme(sdbusplus::bus::bus& bus) :
        bus(bus), _event(sdeventplus::Event::get_default()),
        _timer(_event, std::bind(&Nvme::read, this))
    {
    }

    /**
     * Structure for keeping nvme configure data required by nvme monitoring
     */
    struct NVMeConfig
    {
        uint8_t index;
        uint8_t busID;
        std::string faultLedGroupPath;
        uint8_t presentPin;
        uint8_t pwrGoodPin;
        std::string locateLedControllerBusName;
        std::string locateLedControllerPath;
        std::string locateLedGroupPath;
        uint64_t criticalHigh;
        uint64_t criticalLow;
        uint64_t maxValue;
        uint64_t minValue;
        uint64_t warningHigh;
        uint64_t warningLow;
    };

    /**
     * Structure for keeping nvme data required by nvme monitoring
     */
    struct NVMeData
    {
        bool present;              /* Whether or not the nvme is present  */
        std::string vendor;        /* The nvme manufacturer  */
        std::string serialNumber;  /* The nvme serial number  */
        std::string smartWarnings; /* Indicates smart warnings for the state  */
        std::string statusFlags;   /* Indicates the status of the drives  */
        std::string
            driveLifeUsed; /* A vendor specific estimate of the percentage  */
        u_int64_t sensorValue; /* Sensor value, if sensor value didn't be
                                  update, means sensor failure, default set to
                                  129(0x81) accroding to NVMe-MI SPEC*/
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
                     const std::string& ledPath, const bool& request);
    /** @brief Set SSD locate LED status */
    void setLocateLED(const std::string& ledPath,
                      const std::string& locateLedBusName,
                      const std::string& locateLedPath, const bool& ispresent);
    /** @brief Get Identify State*/
    bool getLEDGroupState(const std::string& ledPath);

    /** @brief Set inventory properties of nvme */
    void setNvmeInventoryProperties(
        const bool& present, const phosphor::nvme::Nvme::NVMeData& nvmeData,
        const std::string& inventoryPath);

    void createNVMeInventory();

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief the Event Loop structure */
    sdeventplus::Event _event;
    /** @brief Read Timer */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    std::vector<phosphor::nvme::Nvme::NVMeConfig> configs;

    /** @brief Set up initial configuration value of NVMe */
    void init();
    /** @brief Monitor NVMe drives every one second  */
    void read();
};
} // namespace nvme
} // namespace phosphor