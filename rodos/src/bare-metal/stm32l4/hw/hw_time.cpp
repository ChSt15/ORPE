#include "hw_specific.h"

#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"
#include "cortex_m/nvic.h"

#include <atomic>
#include <limits>

namespace RODOS {



class HW_Time {
private:
    static constexpr uint32_t interruptFreq {100};
    static constexpr uint32_t clockFreq {1'000'000};

    static_assert((PCLK::freq1%clockFreq)==0);
    static_assert(clockFreq%interruptFreq==0);

    static constexpr uint32_t reloadValue {clockFreq/interruptFreq};
    static constexpr uint32_t prescalar {PCLK::freq1/clockFreq};
    static constexpr uint16_t maxReloadValue {0xFFFF};
    static constexpr uint32_t maxPrescalar {0x10000};
    static_assert(reloadValue<maxReloadValue);
    static_assert(prescalar<maxPrescalar);

    static constexpr uint32_t prescalarRegValue {prescalar-1};

    static constexpr int32_t clockPeriodNanosec {1'000'000'000/clockFreq};
    static constexpr int32_t interruptPeriodNanosec {1'000'000'000/interruptFreq};

    constinit volatile static int64_t nanoTime;

    static constexpr float minTimeDrift {-1};
    static constexpr float maxTimeDrift {1};
    static_assert(minTimeDrift < maxTimeDrift);
    static constinit std::atomic<float> timeDrift;

    static inline BasicTimerStruct& tim6 = *TIM6Def::periphStruct;

    static void initInterrupts(){
        using namespace BasicTimer_DIER;
        NVIC::setIRQPriority(TIM6Def::IRQ_N, NVIC::highestPriority);
        NVIC::enableIRQ(TIM6Def::IRQ_N);
        tim6.DIER.write(UIE(1));
    }
    static void initTiming(){
        tim6.PSC.write(prescalarRegValue);
        tim6.ARR.write(reloadValue);
    }

    static void start(){
        using namespace BasicTimer_CR1;
        tim6.CR1.write(CEN(1));
    }

    static inline void resetInterrupt(){
        tim6.SR.write(0ul);
        NVIC::clearPendingIRQ(TIM6Def::IRQ_N);
    }
public:
    static void init(){
        TIM6Def::rccPeriph->enableClock();
        initInterrupts();
        initTiming();
        start();
    }

    static int64_t getNanoTime(){
        uint32_t count = 0;
        int64_t returnTime = 0;

        do {
            returnTime = nanoTime;
            count = tim6.CNT.read();
        } while(returnTime != nanoTime);

        constexpr int64_t maxPassedTime { reloadValue*clockPeriodNanosec };
        static_assert(maxPassedTime < std::numeric_limits<int32_t>::max(), "multiplication of count and clockPeriodNanosec may overflow.");
        const int32_t passedTime { static_cast<int32_t>(count)*clockPeriodNanosec };
        static_assert(static_cast<int64_t>(static_cast<float>(maxPassedTime)*minTimeDrift) > std::numeric_limits<int32_t>::min(), "conversion of time drift adjustment may underflow");
        static_assert(static_cast<int64_t>(static_cast<float>(maxPassedTime)*maxTimeDrift) < std::numeric_limits<int32_t>::max(), "conversion of time drift adjustment may overflow");
        return returnTime + passedTime + static_cast<int32_t>(static_cast<float>(passedTime)*timeDrift);
    }

    static inline void IRQHandler(){
        static_assert(static_cast<int64_t>(static_cast<float>(interruptPeriodNanosec)*minTimeDrift) > std::numeric_limits<int32_t>::min(), "conversion of time drift adjustment may underflow");
        static_assert(static_cast<int64_t>(static_cast<float>(interruptPeriodNanosec)*maxTimeDrift) < std::numeric_limits<int32_t>::max(), "conversion of time drift adjustment may overflow");
        nanoTime = nanoTime + interruptPeriodNanosec + static_cast<int32_t>(static_cast<float>(interruptPeriodNanosec)*timeDrift);
        resetInterrupt();
    }

    static void setTimeDrift(float drift){
        if(drift>minTimeDrift && drift<maxTimeDrift){
            timeDrift = drift;
        }
    }
};

constinit volatile int64_t HW_Time::nanoTime{0};
constinit std::atomic<float> HW_Time::timeDrift{0};

extern "C" void TIM6_DAC_IRQHandler()
{
    HW_Time::IRQHandler();
}



int64_t hwGetNanoseconds() {
    return HW_Time::getNanoTime();
}


void hwInitTime()
{
    HW_Time::init();
}


int64_t hwGetAbsoluteNanoseconds()
{
    return hwGetNanoseconds();// + timeAtStartup;
}

void setTimeDrift(float drift){
    HW_Time::setTimeDrift(drift);
}



}
