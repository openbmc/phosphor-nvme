#pragma once

#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/server.hpp>

namespace phosphor
{
namespace nvme
{

using ValueIface = sdbusplus::xyz::openbmc_project::Sensor::server::Value;

using CriticalInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Critical;

using WarningInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Warning;

using AvailabilityInterface =
    sdbusplus::xyz::openbmc_project::State::Decorator::server::Availability;

using NvmeIfaces = sdbusplus::server::object_t<ValueIface, CriticalInterface,
                                               WarningInterface, AvailabilityInterface>;

class NvmeSSD : public NvmeIfaces
{
  public:
    NvmeSSD() = delete;
    NvmeSSD(const NvmeSSD&) = delete;
    NvmeSSD& operator=(const NvmeSSD&) = delete;
    NvmeSSD(NvmeSSD&&) = delete;
    NvmeSSD& operator=(NvmeSSD&&) = delete;
    virtual ~NvmeSSD() = default;

    /** @brief Constructs NvmeSSD
     *
     * @param[in] bus     - Handle to system dbus
     * @param[in] objPath - The Dbus path of nvme
     */
    NvmeSSD(sdbusplus::bus_t& bus, const char* objPath) :
        NvmeIfaces(bus, objPath), bus(bus)
    {
        ValueIface::unit(Unit::DegreesC);
    }

    /** @brief Set sensor value temperature to nvme D-bus  */
    void setSensorValueToDbus(const int8_t value);
    /** @brief Check if sensor value higher or lower threshold */
    void checkSensorThreshold();
    /** @brief Set Sensor Threshold to D-bus at beginning */
    void setSensorThreshold(int8_t criticalHigh, int8_t criticalLow,
                            int8_t warningHigh, int8_t warningLow);
    /** @brief Set Sensor Max/Min value to D-bus at beginning */
    void setSensorMaxMin(int8_t maxValue, int8_t minValue);
    /** @brief Set Sensor Availability to D-bus */
    void setSensorAvailability(bool avail);

  private:
    sdbusplus::bus_t& bus;
};
} // namespace nvme
} // namespace phosphor
