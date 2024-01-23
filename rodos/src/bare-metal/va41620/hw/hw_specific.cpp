#include "hw_context.h"

#include "cortex_m/nvic.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/pin_config/pin_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "hw_uart_dbg.h"
#include "hw_hal/hw_hal_watchdog.h"
#include "clkgen_config.h"


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
    HWContext::init();
}

void enableIrqRouterPeripheral(){
    sysconfigPeripheralBitband->IRQ_ROUTER_ENABLE.write(1);
}

void HWContext::init()
{
    enableIrqRouterPeripheral();
    pin_config::enableIOConfigPeripheral();
    NVIC::setAllIrqsToDefaultPriority();
    uartStdout.init(115200);
    //extern int32_t myNodeNr; // independent/globaldata.cpp
    PRINTF("CPU: %lu MHz\n", globalClockSetup.getSysclkFrequency() / 1'000'000);
}

void hwResetAndReboot()
{
    NVIC::systemReset();
}

void enterSleepMode()
{
    asm volatile("wfi");
}

void hwInitWatchdog(int32_t interval) {
    HW_HAL_WATCHDOG::hwInitWatchdog(interval);
}

void hwTriggerWatchdog() {
    HW_HAL_WATCHDOG::hwTriggerWatchdog();
}

}
