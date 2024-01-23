#pragma once

#include "cortex_m/register_access/reserved_bytes.h"
#include "cortex_m/hw_periph_bitband.h"

namespace RODOS {

constexpr uint32_t DISTANCE_BETWEEN_REGS = 0x20;
constexpr uint32_t REG_OFFSET = (DISTANCE_BETWEEN_REGS * 8 * sizeof(uint32_t));

constexpr uint32_t RCC_BASE_ADDR = 0x4002'1000;
constexpr uint32_t RCC_AHB1RSTR_BASE = RCC_BASE_ADDR + 0x28;
constexpr uint32_t RCC_AHB2RSTR_BASE = RCC_BASE_ADDR + 0x2C;
constexpr uint32_t RCC_AHB3RSTR_BASE = RCC_BASE_ADDR + 0x30;

constexpr uint32_t RCC_APB1RSTR1_BASE = RCC_BASE_ADDR + 0x38;
constexpr uint32_t RCC_APB1RSTR2_BASE = RCC_BASE_ADDR + 0x3C;
constexpr uint32_t RCC_APB2RSTR_BASE = RCC_BASE_ADDR + 0x40;

/**
 * Handles the RCC reset/enable bits for a peripheral.
 * See RCC_AHB1RSTR to RCC_APB2ENR registers descriptions in the reference manual
 * Uses the cortex-M4 bit-banding feature in order to make all actions atomic
 */
class HWRCCPeriph {
    volatile uint32_t resetRegisterBB;
    ReservedWord RESERVED0[REG_OFFSET/sizeof(uint32_t) - 1];
    volatile uint32_t enableRegisterBB;
    ReservedWord RESERVED1[REG_OFFSET/sizeof(uint32_t) - 1];
    volatile uint32_t sleepStopRegisterBB;

public:
    HWRCCPeriph() = delete;
    ~HWRCCPeriph() = delete;

    inline uint32_t isEnabled()
    {
        return enableRegisterBB;
    }
    inline void enableClock()
    {
        enableRegisterBB = 1;
        isEnabled(); //dummy read to ensure a small wait
    }
    inline void disableClock()
    {
        enableRegisterBB = 0;
    }
    inline uint32_t isResetActive()
    {
        return resetRegisterBB;
    }
    inline void reset()
    {
        resetRegisterBB = 1;
    }
    inline void releaseReset()
    {
        resetRegisterBB = 0;
    }

    inline void enableInSleepStop()
    {
        sleepStopRegisterBB = 1;
    }
    inline void disableInSleepStop()
    {
        sleepStopRegisterBB = 0;
    }
};
static_assert(sizeof(HWRCCPeriph) == 2 * REG_OFFSET + sizeof(uint32_t));


template <uint32_t baseAddr, uint32_t offset>
using HWRCCBitBand = HWPeriphBitBand<baseAddr, offset, HWRCCPeriph>;


}
