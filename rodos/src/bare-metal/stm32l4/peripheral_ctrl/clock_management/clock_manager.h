#pragma once

#include "rcc_config.h"
#include "peripheral_ctrl/register_maps/rcc_registers.h"

#include "peripherals.h"
#include "hw/vcore_range_manager.h"

namespace RODOS {

class LSE {
public:
    static constexpr bool enable = LSEConfig::enable;
    static constexpr uint32_t freq = enable ? 32'768 : 0;

    static inline bool isReady(){ return rcc.BDCR.read<RCC_BDCR::LSERDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
};

class MSI {
private:
    static constexpr uint8_t range = MSIConfig::range;
    static constexpr bool usePLLMode = MSIConfig::usePLLMode;


    static constexpr uint32_t freqRangeTable[12] = {
        100'000ul,
        200'000ul,
        400'000ul,
        800'000ul,
        1'000'000ul,
        2'000'000ul,
        4'000'000ul,
        8'000'000ul,
        16'000'000ul,
        24'000'000ul,
        32'000'000ul,
        48'000'000ul,
    };

    static_assert(range < sizeof(freqRangeTable));
public:
    static constexpr bool enable = MSIConfig::enable;

    static constexpr uint32_t freq = enable ? freqRangeTable[range] : 0;
    static_assert(!enable || (freq<=VCORERangeManager::maxMSIFreq));

    static_assert(!usePLLMode || LSE::enable);

    static inline bool isReady(){ return rcc.CR.read<RCC_CR::MSIRDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
    static void disableClock();
};



class HSE {
public:
    static constexpr bool enable = HSEConfig::enable;
    static constexpr uint32_t freq = enable ? HSEConfig::freq : 0;


    static_assert(!enable || (freq<=VCORERangeManager::maxHSEFreq));


    static inline bool isReady(){ return rcc.CR.read<RCC_CR::HSERDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
};


class HSI16 {
public:
    static constexpr bool enable = HSI16Config::enable;
    static constexpr uint32_t freq = enable ? 16'000'000ul : 0;

    static inline bool isReady(){ return rcc.CR.read<RCC_CR::HSIRDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
};

class PLL {
private:
    static void setMainParameters();
    static void initOutputs();
public:
    static constexpr bool enable = PLLConfig::enable;
    static constexpr PLL_SOURCE source = PLLConfig::source;
    static constexpr uint32_t PLLMConfig = PLLConfig::PLLM;
    static constexpr uint32_t PLLNConfig = PLLConfig::PLLN;

    static constexpr bool enablePLLSAI3CLK = PLLConfig::enablePLLSAI3CLK;
    static constexpr uint32_t PLLPRegValue = PLLConfig::PLLPRegValue;
    static constexpr uint32_t PLLPDIVConfig = PLLConfig::PLLPDIV;

    static constexpr bool enablePLL48M1CLK = PLLConfig::enablePLL48M1CLK;
    static constexpr uint32_t PLLQRegValue = PLLConfig::PLLQRegValue;

    static constexpr bool enablePLLCLK = PLLConfig::enablePLLCLK;
    static constexpr uint32_t PLLRRegValue = PLLConfig::PLLRRegValue;



    static constexpr uint32_t inputFreq = []{
        switch(source){
        case PLL_SOURCE::MSI:   return MSI::freq;
        case PLL_SOURCE::HSI16: return HSI16::freq;
        case PLL_SOURCE::HSE:   return HSE::freq;
        default:                return 0ul;
        }
    }();
    static_assert(!enable || (inputFreq>0));


    static constexpr uint32_t PLLMRegValue = PLLMConfig-1;
    static_assert(1<=PLLMConfig && PLLMConfig<=8);
    static constexpr uint32_t VCOInputFreq = inputFreq/PLLMConfig;
    static_assert( !enable || (4'000'000ul<=VCOInputFreq && VCOInputFreq<=16'000'000ul));

    static_assert((8<=PLLNConfig && PLLNConfig<=86));
    static constexpr uint32_t VCOOutputFreq = VCOInputFreq * PLLNConfig;
    static_assert(!enable || (64'000'000ul<=VCOOutputFreq&&VCOOutputFreq<=VCORERangeManager::maxPLLVCOFreq));



    static_assert(enable || !enablePLLSAI3CLK);
    static constexpr uint32_t PLLPTable[2] = {
        7,
        17
    };
    static_assert(PLLPRegValue<sizeof(PLLPTable));
    static constexpr uint32_t PLLP_VALUE = PLLPTable[PLLPRegValue];

    static_assert(PLLPDIVConfig==0 || (PLLPDIVConfig>=2&&PLLPDIVConfig<=31));
    static constexpr uint32_t PLLSAI3CLKdiv = PLLPDIVConfig ? PLLPDIVConfig : PLLP_VALUE;
    static constexpr uint32_t freqPLLSAI3CLK = enablePLLSAI3CLK? VCOOutputFreq/PLLSAI3CLKdiv : 0;
    static_assert(!enablePLLSAI3CLK || freqPLLSAI3CLK<=VCORERangeManager::maxPLLFreq);



    static_assert(enable || !enablePLL48M1CLK);
    static constexpr uint32_t PLLQTable[4] = {
        2,
        4,
        6,
        8
    };
    static_assert(PLLQRegValue<sizeof(PLLQTable));
    static constexpr uint32_t PLLQ_VALUE = PLLQTable[PLLQRegValue];
    static constexpr uint32_t freqPLL48M1CLK = enablePLL48M1CLK ? VCOOutputFreq/PLLQ_VALUE : 0;
    static_assert(!enablePLL48M1CLK || freqPLL48M1CLK<=VCORERangeManager::maxPLLFreq);



    static_assert(enable || !enablePLLCLK);
    static constexpr uint32_t PLLRTable[4] = {
        2,
        4,
        6,
        8
    };
    static_assert(PLLRRegValue<sizeof(PLLRTable));
    static constexpr uint32_t PLLR_VALUE = PLLRTable[PLLRRegValue];

    static constexpr uint32_t freqPLLCLK = VCOOutputFreq/PLLR_VALUE;
    static_assert(!enablePLLCLK || freqPLLCLK<=VCORERangeManager::maxPLLFreq);


    static inline bool isReady(){ return rcc.CR.read<RCC_CR::PLLRDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
};

class LSI {
public:
    static constexpr bool enable = LSIConfig::enable;
    static constexpr uint32_t freq = enable ? 32'000ul : 0;


    static inline bool isReady(){ return rcc.CSR.read<RCC_CSR::LSIRDY>();}
    static void waitUntilReady(){ while(!isReady());}
    static void initClock();
};

class SYSCLK {
public:
    static constexpr SYSCLK_SOURCE source = SYSCLKConfig::source;


    static constexpr uint32_t freq = []{
        switch(source){
        case SYSCLK_SOURCE::MSI:   return MSI::freq;
        case SYSCLK_SOURCE::HSI16: return HSI16::freq;
        case SYSCLK_SOURCE::HSE:   return HSE::freq;
        case SYSCLK_SOURCE::PLL:   return PLL::freqPLLCLK;
        default:                   return 0ul;
        }
    }();
    static_assert(freq>0);

    static void initSource();
};

class HCLK {
private:
    static constexpr uint32_t HPRETable[16] = {
        1,1,1,1,1,1,1,1,
        2,
        4,
        8,
        16,
        64,
        128,
        256,
        512
    };

public:
    static constexpr uint32_t HPREConfig = BusClockConfig::HPRE;
    static_assert(HPREConfig<sizeof(HPRETable));
    static constexpr uint32_t AHB_PRESC = HPRETable[HPREConfig];
    static constexpr uint32_t freq = SYSCLK::freq/AHB_PRESC;
};

class PCLK {
    static constexpr uint32_t PPRETable[8] = {
        1,1,1,1,
        2,
        4,
        8,
        16
    };

public:
    static constexpr uint32_t PPRE1Config = BusClockConfig::PPRE1;
    static constexpr uint32_t PPRE2Config = BusClockConfig::PPRE2;
    static_assert(PPRE1Config<sizeof(PPRETable));

    static constexpr uint32_t freq1 = HCLK::freq/PPRETable[PPRE1Config];
    static constexpr uint32_t freq2 = HCLK::freq/PPRETable[PPRE2Config];
};

class BusClockManager {
public:
    static void initPrescalers();
};

class ClockManager {
private:
    static void initClockSources();
    static void initSysClock();
    static void waitUntilAllClocksReady();
public:
    static void initClocks();
};


}
