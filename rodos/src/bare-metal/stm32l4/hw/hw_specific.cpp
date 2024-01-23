#include "rodos.h"

#include "hal/hal_uart.h"
#include "hw_context.h"
#include "hw_hal/gpio/exti.h"

#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "hw/cpu_manager.h"
#include "hw/flash_manager.h"
#include "cortex_m/nvic.h"
#include "hw_uart_dbg.h"


namespace RODOS {

void hwDisableInterrupts()
{
    asm volatile("cpsid i");
}

void hwEnableInterrupts()
{
    asm volatile("cpsie i");
}

void hwInit()
{
    EXTI::initEXTInterrupts();
    HWContext::init();
}

void HWContext::init(void)
{
    /* clear the reset flags */
    rcc.CSR.set(RODOS::RCC_CSR::RMVF(1));

    NVIC::setAllIrqsToDefaultPriority();
    FlashManager::initCache();
    ClockManager::initClocks();
    CPUManager::setupDebugging();

    VCORERangeManager::init();


    extern int32_t myNodeNr; // independent/globaldata.cpp
    uint32_t       uuidWord = CPUManager::getFirstUUIDWord();
    myNodeNr                = *reinterpret_cast<int32_t*>(&uuidWord);

    uartStdout.init(115200);
    PRINTF("CPU: %lu MHz\n", HCLK::freq / 1'000'000);
}

void hwResetAndReboot()
{
    NVIC::systemReset();
}

void enterSleepMode()
{
    asm volatile("wfi");
}

}
