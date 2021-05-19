#include "nvmes.hpp"

namespace phosphor
{
namespace nvme
{

void NvmeSSD::checkSensorThreshold()
{
    int8_t value = ValueIface::value();
    int8_t criticalHigh = CriticalInterface::criticalHigh();
    int8_t criticalLow = CriticalInterface::criticalLow();
    int8_t warningHigh = WarningInterface::warningHigh();
    int8_t warningLow = WarningInterface::warningLow();

    CriticalInterface::criticalAlarmHigh(value > criticalHigh);

    CriticalInterface::criticalAlarmLow(value < criticalLow);

    WarningInterface::warningAlarmHigh(value > warningHigh);

    WarningInterface::warningAlarmLow(value < warningLow);
}

void NvmeSSD::setSensorThreshold(int8_t criticalHigh, int8_t criticalLow,
                                 int8_t warningHigh, int8_t warningLow)
{
    auto criticalHi = CriticalInterface::criticalHigh();
    auto criticalLo = CriticalInterface::criticalLow();
    auto warningHi = WarningInterface::warningHigh();
    auto warningLo = WarningInterface::warningLow();

    if (criticalHi != criticalHigh)
    {
        CriticalInterface::criticalHigh(criticalHigh);
    }
    if (criticalLo != criticalLow)
    {
        CriticalInterface::criticalLow(criticalLow);
    }

    if (warningHi != warningHigh)
    {
        WarningInterface::warningHigh(warningHigh);
    }
    if (warningLo != warningLow)
    {
        WarningInterface::warningLow(warningLow);
    }
}

void NvmeSSD::setSensorMaxMin(int8_t maxValue, int8_t minValue)
{
    ValueIface::maxValue(maxValue);
    ValueIface::minValue(minValue);
}

void NvmeSSD::setSensorValueToDbus(const int8_t value)
{
    ValueIface::value(value);
}

} // namespace nvme
} // namespace phosphor