#include "config.h"

#include "nvmes.hpp"

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
        uint8_t presentPin;
        uint8_t pwrGoodPin;
        uint64_t criticalHigh;
        uint64_t criticalLow;
        uint64_t maxValue;
        uint64_t minValue;
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
    std::string getGPIOValueOfNvme(std::string);
    /** @brief Map of the object NvmeSSD */
    std::unordered_map<std::string, std::shared_ptr<phosphor::nvme::NvmeSSD>>
        nvmes;

  private:
    /** @brief sdbusplus bus client connection. */
    sdbusplus::bus::bus& bus;
    /** @brief the Event Loop structure */
    sdeventplus::Event _event;
    /** @brief Read Timer */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    /** @brief Set up initial configuration value of NVMe */
    void init();
    /** @brief Monitor NVMe drives every one second  */
    void read();
};
} // namespace nvme
} // namespace phosphor