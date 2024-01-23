#include "hw_specific.h"
#include "timemodel.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "cortex_m/nvic.h"
#include "rodos-debug.h"

#include <atomic>

namespace RODOS {

class HW_Time {
private:
    static constexpr int32_t clockFreq {globalClockSetup.getAPB1Frequency()};

    static constexpr uint32_t reloadValue {0xFFFF'FFFF};

    static constexpr int32_t clockPeriodNanosec {1*SECONDS/clockFreq};

    static constexpr float minTimeDrift {-1};
    static constexpr float maxTimeDrift {1};
    static_assert(minTimeDrift < maxTimeDrift);
    static constinit std::atomic<float> timeDrift;

    static void initTiming(){
        using namespace TIM_CSD_CTRL;
        tim0to15[0].RST_VALUE.write(reloadValue);
        tim0to15[0].CNT_VALUE.write(reloadValue);

        tim0to15[1].RST_VALUE.write(reloadValue);
        tim0to15[1].CNT_VALUE.write(reloadValue);
        tim0to15[1].CSD_CTRL.write(CSDEN0(1));
        tim0to15[1].CASCADE[0].write(static_cast<uint32_t>(CascadeSource::fromTimer(0)));
    }

    static void start(){
        using namespace TIM_CTRL;
        tim0to15[0].CTRL.write(ENABLE(1));
        tim0to15[1].CTRL.write(ENABLE(1));
    }
public:
    static void init(){
        sysconfigPeripheralBitband->TIM_ENABLE[0].write(uint32_t(1));
        sysconfigPeripheralBitband->TIM_ENABLE[1].write(uint32_t(1));
        initTiming();
        start();
    }
    static int64_t getNanoTime(){
        uint32_t countLow { 0 };
        uint32_t countHigh { tim0to15[1].CNT_VALUE.read() };
        uint32_t prevCountHigh { 0 };
        do {
            prevCountHigh = countHigh;
            countLow = tim0to15[0].CNT_VALUE.read();
            countHigh = tim0to15[1].CNT_VALUE.read();
        } while(countHigh != prevCountHigh);
        int64_t totalCount = static_cast<int64_t>(reloadValue-countHigh)<<32 | (reloadValue-countLow);
        return totalCount * clockPeriodNanosec;
    }

    static void setTimeDrift(float drift){
        if(drift>minTimeDrift && drift<maxTimeDrift){
            timeDrift = drift;
        }
    }
};

constinit std::atomic<float> HW_Time::timeDrift{0};


int64_t hwGetNanoseconds() {
    return HW_Time::getNanoTime();
}


void hwInitTime()
{
    HW_Time::init();
}


int64_t hwGetAbsoluteNanoseconds()
{
    return hwGetNanoseconds();
}

void setTimeDrift(float drift){
    HW_Time::setTimeDrift(drift);
}

}
