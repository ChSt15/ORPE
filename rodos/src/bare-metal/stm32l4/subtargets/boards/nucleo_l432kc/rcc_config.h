#pragma once
#include <stdint.h>
#include "peripheral_ctrl/register_maps/rcc_registers.h"

//Note: currently there is no support for the SMPS voltage ranges
//See sections 5.1.8 and 6.2.9 in the STM32L496 reference Manual

namespace RODOS {

class VCORERangeConfig {
public:
    static constexpr uint32_t voltageRange = 1;
};


class LSEConfig {
public:
    static constexpr bool enable = false;
};

class MSIConfig {
public:
    static constexpr bool enable = true;
    static constexpr uint8_t range = 6;
    static constexpr bool usePLLMode = false;
};

class HSI16Config {
public:
    static constexpr bool enable = false;
};

class HSEConfig {
public:
    static constexpr bool enable = false;
    static constexpr uint32_t freq = 8'000'000;
};

class PLLConfig{
public:
    static constexpr bool enable = true;
    static constexpr PLL_SOURCE source = PLL_SOURCE::MSI;
    static constexpr uint32_t PLLM = 1;
    static constexpr uint32_t PLLN = 40;

    static constexpr bool enablePLLSAI3CLK = false;
    static constexpr uint32_t PLLPRegValue = 0;
    static constexpr uint32_t PLLPDIV = 0;

    static constexpr bool enablePLL48M1CLK = false;
    static constexpr uint32_t PLLQRegValue = 0;
    
    static constexpr bool enablePLLCLK = true;
    static constexpr uint32_t PLLRRegValue = 0;
};

class LSIConfig {
public:
    static constexpr bool enable = true;
};


class SYSCLKConfig {
public:
    static constexpr SYSCLK_SOURCE source = SYSCLK_SOURCE::PLL;
};


class BusClockConfig {
public:
    static constexpr uint32_t HPRE = 0;
    static constexpr uint32_t PPRE1 = 0;
    static constexpr uint32_t PPRE2 = 0;
};


}
