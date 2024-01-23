#pragma once
#include "rcc_config.h"
#include <cstdint>

//Note: currently there is no support for the SMPS voltage ranges
//See sections 5.1.8 and 6.2.9 in the STM32L496 reference Manual

namespace RODOS {

class VCORERangeManager {
public:
    static constexpr uint32_t voltageRange = VCORERangeConfig::voltageRange;
    static_assert(voltageRange==1 || voltageRange==2);

    static constexpr uint32_t maxMSIFreq = (voltageRange==1) ? 48'000'000 : 24'000'000;
    static constexpr uint32_t maxHSEFreq = (voltageRange==1) ? 48'000'000 : 26'000'000;
    static constexpr uint32_t maxPLLFreq = (voltageRange==1) ? 80'000'000 : 26'000'000;
    static constexpr uint32_t maxPLLVCOFreq = (voltageRange==1) ? 344'000'000 : 128'000'000;

    static void init();
};


}
