#include "nvmes.hpp"

namespace phosphor
{
namespace nvme
{

void NvmeSSD::checkSensorThreshold()
{
    uint64_t value = ValueIface::value();
    uint64_t criticalHigh = CriticalInterface::criticalHigh();
    uint64_t criticalLow = CriticalInterface::criticalLow();
    uint64_t warningHigh = WarningInterface::warningHigh();
    uint64_t warningLow = WarningInterface::warningLow();

    CriticalInterface::criticalAlarmHigh(value > criticalHigh);

    CriticalInterface::criticalAlarmLow(value < criticalLow);

    WarningInterface::warningAlarmHigh(value > warningHigh);

    WarningInterface::warningAlarmLow(value < warningLow);
}

void NvmeSSD::setSensorThreshold(uint64_t criticalHigh, uint64_t criticalLow,
                                 uint64_t maxValue, uint64_t minValue,
                                 uint64_t warningHigh, uint64_t warningLow)
{

    CriticalInterface::criticalHigh(criticalHigh);
    CriticalInterface::criticalLow(criticalLow);

    WarningInterface::warningHigh(warningHigh);
    WarningInterface::warningLow(warningLow);

    ValueIface::maxValue(maxValue);
    ValueIface::minValue(minValue);
}

void NvmeSSD::setSensorValueToDbus(const u_int64_t value)
{
    ValueIface::value(value);
}

} // namespace nvme
} // namespace phosphor