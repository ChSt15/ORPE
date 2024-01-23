#pragma once

#include "peripheral_ctrl/clock_management/clock_manager.h"
namespace RODOS {


class FlashManager {
private:
    static void setLatency(uint32_t latency);

    static constexpr auto getMinLatency = [](uint32_t freq){
        switch(VCORERangeManager::voltageRange){
        case 1:
            if(freq <= 16'000'000){
                return 0;
            }else if(freq <= 32'000'000){
                return 1;
            }else if(freq <= 48'000'000){
                return 2;
            }else if(freq <= 64'000'000){
                return 3;
            }else{
                return 4;
            }
        case 2:
            if(freq <= 6'000'000){
                return 0;
            }else if(freq <= 12'000'000){
                return 1;
            }else if(freq <= 18'000'000){
                return 2;
            }else{
                return 3;
            }
        default: __builtin_trap();
        }
    };

    static constexpr uint32_t minLatencyMSI = getMinLatency(MSI::freq);
    static constexpr uint32_t minLatencySYSCLK = getMinLatency(HCLK::freq);

public:
    static inline void initLatencyPreMSIInit(){
        if constexpr(minLatencyMSI>0){
            setLatency(minLatencyMSI);
        }
    }
    static inline void initLatencyPreSYSCLKswitch(){
        if constexpr(minLatencySYSCLK>minLatencyMSI){
            setLatency(minLatencySYSCLK);
        }
    }
    static void initLatencyPostSYSCLKswitch(){
        if constexpr(minLatencySYSCLK<minLatencyMSI){
            setLatency(minLatencySYSCLK);
        }
    }
    static void initCache();
};


}
