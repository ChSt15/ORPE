#include "hw_hal_watchdog.h"

#include "rodos-debug.h"

namespace RODOS {

constexpr uint32_t HW_HAL_WATCHDOG::convertMillisecondsToCycles(int32_t milliseconds) {
    if(static_cast<uint32_t>(milliseconds) >= MAX_TIMEOUT_MILLISECONDS) {
        return UINT32_MAX;
    }
    return globalClockSetup.getAPB2Frequency() * (static_cast<uint32_t>(milliseconds));
}

void HW_HAL_WATCHDOG::hwInitWatchdog(int32_t intervalMilliseconds) {

    if(intervalMilliseconds <= 0) {
        return;
    }

    // Reset and Enable Watchdog
    sysconfigPeripheralBitband->WATCHDOG_RESET.write(0U);
    sysconfigPeripheralBitband->WATCHDOG_RESET.write(1U);
    sysconfigPeripheralBitband->WATCHDOG_ENABLE.write(1U);

    // Enable Reset and Interrupts
    watchdog->WDOGCONTROL.write(
      WD_WDOGCONTROL::RESEN(1U),
      WD_WDOGCONTROL::INTEN(1U));
    watchdog->WDOGLOAD.write(convertMillisecondsToCycles(intervalMilliseconds));
}

void HW_HAL_WATCHDOG::hwTriggerWatchdog() {
    watchdog->WDOGLOAD.write(watchdog->WDOGLOAD.read());
}


} // namespace RODOS
