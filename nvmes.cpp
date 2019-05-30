#include "nvme_manager.hpp"
#include <iostream>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sstream>
#include <string>


namespace phosphor
{
namespace nvme
{
using namespace std;
using namespace phosphor::logging;

void NvmeSSD::checkSensorThreshold()
{
    uint64_t value = ValueIface::value();
    uint64_t criticalHigh = CriticalInterface::criticalHigh();
    uint64_t criticalLow = CriticalInterface::criticalLow();

    if (value > criticalHigh)
        CriticalInterface::criticalAlarmHigh(true);
    else
        CriticalInterface::criticalAlarmHigh(false);

    if (value < criticalLow)
        CriticalInterface::criticalAlarmLow(true);
    else
        CriticalInterface::criticalAlarmLow(false);
}

void NvmeSSD::setSensorThreshold(uint64_t criticalHigh, uint64_t criticalLow,
                                 uint64_t maxValue, uint64_t minValue)
{

    CriticalInterface::criticalHigh(criticalHigh);
    CriticalInterface::criticalLow(criticalLow);
    ValueIface::maxValue(maxValue);
    ValueIface::minValue(minValue);
}

void NvmeSSD::setSensorValueToDbus(const u_int64_t value)
{
    ValueIface::value(value);
}

} // namespace nvme
} // namespace phosphor