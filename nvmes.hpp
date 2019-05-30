#include "config.h"
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>
#include <xyz/openbmc_project/Inventory/Decorator/Asset/server.hpp>
#include <xyz/openbmc_project/Inventory/Item/server.hpp>
#include <xyz/openbmc_project/Nvme/Status/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

namespace phosphor
{
namespace nvme
{

using ValueIface = sdbusplus::xyz::openbmc_project::Sensor::server::Value;

using CriticalInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Critical;

using WarningInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Warning;

using NvmeIfaces =
    sdbusplus::server::object::object<ValueIface, CriticalInterface, WarningInterface>;

class NvmeSSD : public NvmeIfaces
{
  public:
    NvmeSSD() = delete;
    NvmeSSD(const NvmeSSD &) = delete;
    NvmeSSD &operator=(const NvmeSSD &) = delete;
    NvmeSSD(NvmeSSD &&) = delete;
    NvmeSSD &operator=(NvmeSSD &&) = delete;
    virtual ~NvmeSSD() = default;

    NvmeSSD(sdbusplus::bus::bus &bus, const char *objPath) :
        NvmeIfaces(bus, objPath), bus(bus)
    {
    }

    void setSensorValueToDbus(const u_int64_t value);
    void checkSensorThreshold();
    void setSensorThreshold(uint64_t criticalHigh, uint64_t criticalLow,
                            uint64_t maxValue, uint64_t minValue);

  private:
    sdbusplus::bus::bus &bus;
};
} // namespace nvme
} // namespace phosphor