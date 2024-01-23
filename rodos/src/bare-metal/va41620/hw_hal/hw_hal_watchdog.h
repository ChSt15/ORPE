#pragma once

#include "peripheral_ctrl/peripheral_defs.h"
#include "clkgen_config.h"

#include <cstdint>

namespace RODOS {

class HW_HAL_WATCHDOG {
  public:
    static void hwInitWatchdog(int32_t intervalMilliseconds);
    static void hwTriggerWatchdog();

  private:
    static constexpr uint64_t MAX_TIMEOUT_MILLISECONDS = (UINT32_MAX / globalClockSetup.getAPB2Frequency());

    static constexpr uint32_t convertMillisecondsToCycles(int32_t milliseconds);
};

} // namespace RODOS
