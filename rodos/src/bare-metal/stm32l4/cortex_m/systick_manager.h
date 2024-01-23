#pragma once
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "platform-parameter.h"

namespace RODOS {

class SysTickManager{
private:
    static constexpr uint32_t freq = HCLK::freq/8;

    static constinit uint32_t ticks;

    static inline void setReloadValue(uint32_t ticks);
    static inline void resetCurrentVal();
    static inline void enable();
public:
    static void start();
    static void setInterval(const int64_t microseconds);
    static void stop();

    static constexpr uint32_t maxTicks = 0x00FF'FFFF;
    static constexpr int64_t calculateTicks(int64_t microseconds){
        return (freq * microseconds) / 1'000'000;
    }

};


}
