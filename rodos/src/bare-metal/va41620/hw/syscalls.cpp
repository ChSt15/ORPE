#include "hal/hal_uart.h"
#include "platform-parameter.h"
#include "scheduler.h"
#include "rodos-debug.h"
#include "hw_uart_dbg.h"
#include "hw_specific.h"
#include "yprintf.h"
#include "cortex_m/peripheral_defs.h"

namespace RODOS {

bool isInInterrupt();

int putchar(int ic) {
    const char c = static_cast<char>(ic);
    if (c == '\n') {
        putchar('\r');
    }
    if(isInInterrupt()){
        HW_HAL_UART::putcharFromInterrupt(static_cast<VA41620_UART_IDX>(UART_DEBUG_IDX), c);
    }else{
        while(uartStdout.putcharNoWait(c) == -1){
            FFLUSH();
        }
    }
    return c;
}

int puts(const char * str) {
    while (*str) {
        putchar(*str++);
    }
    return 0;
}

void abort(void) { while(1); }

void sp_partition_yield() {}

static Semaphore fflushSemaphore{};

void FFLUSH() {
    if(isSchedulerRunning()&&!isInInterrupt()){
        ScopeProtector fflushProtector{&fflushSemaphore};
        uartStdout.suspendUntilWriteFinished();
    }else{
        while (!uartStdout.isWriteFinished());
    }
}

int getCurrentIRQ(){
    using namespace SCB_ICSR;
    return static_cast<int>(scb.ICSR.read<VECTACTIVE>())-16;
}

bool isInInterrupt(){
    return getCurrentIRQ()!=-16;
}

}
