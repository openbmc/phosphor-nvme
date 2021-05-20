#include "nvmes.hpp"

#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>

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
    auto criticalAlarmLowState = CriticalInterface::criticalAlarmLow();
    auto criticalAlarmHighState = CriticalInterface::criticalAlarmHigh();
    auto warningAlarmLowState = WarningInterface::warningAlarmLow();
    auto warningAlarmHighState = WarningInterface::warningAlarmHigh();

    CriticalInterface::criticalAlarmHigh(value > criticalHigh);

    CriticalInterface::criticalAlarmLow(value < criticalLow);

    WarningInterface::warningAlarmHigh(value > warningHigh);

    WarningInterface::warningAlarmLow(value < warningLow);

    if (criticalAlarmHighState != (value >= criticalHigh))
    {
        if (value >= criticalHigh)
        {
            CriticalInterface::criticalHighAlarmAsserted(value);
        }
        else
        {
            CriticalInterface::criticalHighAlarmDeasserted(value);
        }
    }
    if (criticalAlarmLowState != (value <= criticalLow))
    {
        if (value <= criticalLow)
        {
            CriticalInterface::criticalLowAlarmAsserted(value);
        }
        else
        {
            CriticalInterface::criticalLowAlarmDeasserted(value);
        }
    }
    if (warningAlarmHighState != (value >= warningHigh))
    {
        if (value >= criticalHigh)
        {
            WarningInterface::warningHighAlarmAsserted(value);
        }
        else
        {
            WarningInterface::warningHighAlarmDeasserted(value);
        }
    }
    if (warningAlarmLowState != (value <= warningLow))
    {
        if (value <= warningLow)
        {
            WarningInterface::warningLowAlarmAsserted(value);
        }
        else
        {
            WarningInterface::warningLowAlarmDeasserted(value);
        }
    }
}

void NvmeSSD::setSensorThreshold(int8_t criticalHigh, int8_t criticalLow,
                                 int8_t warningHigh, int8_t warningLow)
{
    CriticalInterface::criticalHigh(criticalHigh);
    CriticalInterface::criticalLow(criticalLow);

    WarningInterface::warningHigh(warningHigh);
    WarningInterface::warningLow(warningLow);
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
