#include "hw/flash_manager.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"

#include "clock_manager.h"

namespace RODOS {

void LSE::initClock(){
    using namespace PWR_CR1;
    using namespace RCC_BDCR;
    if(!isReady()){//LSE may still be active after a reset
        PWRDef::rccPeriph->enableClock();
        pwr.CR1.set(DBP(1));
        rcc.BDCR.set(LSEDRV(3)); //high drive capability for faster startup
        rcc.BDCR.set(LSEON(1));
        waitUntilReady();
        rcc.BDCR.set(LSEDRV(0)); //lower drive capability for lower power consumption
    }
}

void MSI::initClock(){
    using namespace RCC_CR;
    rcc.CR.set(MSIRANGE(range), MSIRGSEL(1));
    if constexpr(usePLLMode){
        rcc.CR.set(MSIPLLEN(1));
    }
    waitUntilReady();
}
void MSI::disableClock(){
    using namespace RCC_CR;
    rcc.CR.set(MSION(0));
}


void HSI16::initClock(){
    using namespace RCC_CR;
    rcc.CR.set(HSION(1));
    waitUntilReady();
}


void HSE::initClock(){
    using namespace RCC_CR;
    rcc.CR.set(HSEON(1));
    waitUntilReady();
}


void LSI::initClock(){
    using namespace RCC_CSR;
    rcc.CSR.set(LSION(1));
    waitUntilReady();
}


void PLL::setMainParameters(){
    using namespace RCC_PLLCFGR;
    rcc.PLLCFGR.set(PLLSRC(static_cast<uint32_t>(source)), PLLM(PLLMRegValue), PLLN(PLLNConfig));
}

void PLL::initOutputs(){
    using namespace RCC_PLLCFGR;
    rcc.PLLCFGR.set(
        PLLPEN(enablePLLSAI3CLK), PLLP(PLLPRegValue), PLLPDIV(PLLPDIVConfig),
        PLLQEN(enablePLL48M1CLK), PLLQ(PLLQRegValue),
        PLLREN(enablePLLCLK), PLLR(PLLRRegValue)
    );
}

void PLL::initClock(){
    using namespace RCC_CR;
    setMainParameters();
    rcc.CR.set(PLLON(1));
    initOutputs();
    waitUntilReady();
}

void SYSCLK::initSource(){
    using namespace RCC_CFGR;
    rcc.CFGR.set(SW(static_cast<uint32_t>(source)));
}


void BusClockManager::initPrescalers(){
    using namespace RCC_CFGR;
    rcc.CFGR.set(
        HPRE(HCLK::HPREConfig),
        PPRE1(PCLK::PPRE1Config),
        PPRE2(PCLK::PPRE2Config)
    );
}


void ClockManager::initClockSources(){
    if constexpr(LSI::enable){ LSI::initClock();}
    if constexpr(LSE::enable){ LSE::initClock();}
    if constexpr(HSI16::enable){ HSI16::initClock();}
    if constexpr(HSE::enable){ HSE::initClock();}
    FlashManager::initLatencyPreMSIInit();
    if constexpr(MSI::enable){ MSI::initClock();}
    if constexpr(PLL::enable){ PLL::initClock();}
}

void ClockManager::initSysClock(){
    BusClockManager::initPrescalers();

    FlashManager::initLatencyPreSYSCLKswitch();
    SYSCLK::initSource();
    FlashManager::initLatencyPostSYSCLKswitch();
}


void ClockManager::initClocks(){
    initClockSources();
    initSysClock();
    if constexpr(!MSI::enable) {MSI::disableClock();}
}

}
