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

class Nvme
{
  public:
    Nvme() = delete;
    Nvme(const Nvme&) = delete;
    Nvme& operator=(const Nvme&) = delete;
    Nvme(Nvme&&) = delete;
    Nvme& operator=(Nvme&&) = delete;

    Nvme(sdbusplus::bus::bus& bus, const char* objPath) :
        bus(bus), _event(sdeventplus::Event::get_default()),
        _timer(_event, std::bind(&Nvme::read, this)), _objPath(objPath)
    {
    }

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

    void run();
    const std::string _objPath;

    std::string getValue(std::string);
    std::unordered_map<std::string, std::shared_ptr<phosphor::nvme::NvmeSSD>>
        nvmes;

  private:
    sdbusplus::bus::bus& bus;

    sdeventplus::Event _event;
    /** @brief Read Timer */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    void init();
    void read();
};
} // namespace nvme
} // namespace phosphor