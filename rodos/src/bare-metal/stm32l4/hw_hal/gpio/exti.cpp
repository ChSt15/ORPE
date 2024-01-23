#include "rodos.h"

#include "exti.h"
#include "hw_hal/hw_hal_gpio.h"
#include "cortex_m/nvic.h"

namespace RODOS {

constinit HW_HAL_GPIO* EXTI::lineOwners[16]{};

int32_t EXTI::enable(uint8_t extiLine, HW_HAL_GPIO* caller, uint8_t portNum, GPIO_IRQ_SENSITIVITY irqSensitivity) {
    if(lineOwners[extiLine]) {
        RODOS_ERROR("External IRQ Line already used by another HAL_GPIO");
        return -1;
    }
    lineOwners[extiLine] = caller;
    setPort(extiLine, portNum);
    setSensitivity(extiLine, irqSensitivity);
    exti.IMR1.setBits(1u << extiLine);
    return 0;
}

void EXTI::reset(uint8_t extiLine) {
    exti.IMR1.clearBits(1u << extiLine);
    exti.RTSR1.clearBits(1u << extiLine);
    exti.FTSR1.clearBits(1u << extiLine);
}

void EXTI::setPort(uint8_t extiLine, uint8_t portNum) {
    uint32_t EXTICRreg   = extiLine / 4;
    uint32_t EXTICRshift = (extiLine % 4) * 4u;
    syscfg.EXTICR[EXTICRreg].clearAndSetBits(
      0x0Ful << EXTICRshift,
      static_cast<RODOS::MMIO::value_t>(portNum) << EXTICRshift);
}

void EXTI::setSensitivity(uint8_t extiLine, GPIO_IRQ_SENSITIVITY irqSensitivity) {
    if(irqSensitivity == GPIO_IRQ_SENS_RISING || irqSensitivity == GPIO_IRQ_SENS_BOTH) {
        exti.RTSR1.setBits(1u << extiLine);
    }
    if(irqSensitivity == GPIO_IRQ_SENS_FALLING || irqSensitivity == GPIO_IRQ_SENS_BOTH) {
        exti.FTSR1.setBits(1u << extiLine);
    }
}

void EXTI::EXTIRQHandler(uint8_t extiLine) {
    if(lineOwners[extiLine]) {
        lineOwners[extiLine]->EXTIRQHandler();
    }
}

void EXTI::initEXTInterrupts() {
    static bool init = false;
    if(!init) {
        init = true;

        SYSCFGDef::rccPeriph->enableClock();

        constexpr uint8_t extiIRQs[]{ EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn };
        for(size_t i = 0; i < sizeof(extiIRQs); i++) {
            NVIC::enableIRQ(extiIRQs[i]);
        }
    }
}

}

extern "C" {

#ifndef RODOS_UNMOUNT_IRQ_EXTI0
void EXTI0_IRQHandler() {
    RODOS::EXTI::EXTIRQHandler(0);
    NVIC::clearPendingIRQ(EXTI::EXTI0_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI1
void EXTI1_IRQHandler() {
    RODOS::EXTI::EXTIRQHandler(1);
    NVIC::clearPendingIRQ(EXTI::EXTI1_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI2
void EXTI2_IRQHandler() {
    RODOS::EXTI::EXTIRQHandler(2);
    NVIC::clearPendingIRQ(EXTI::EXTI2_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI3
void EXTI3_IRQHandler() {
    RODOS::EXTI::EXTIRQHandler(3);
    NVIC::clearPendingIRQ(EXTI::EXTI3_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI4
void EXTI4_IRQHandler() {
    RODOS::EXTI::EXTIRQHandler(4);
    NVIC::clearPendingIRQ(EXTI::EXTI4_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI9_5
void EXTI9_5_IRQHandler() {
    uint32_t pending = exti.PR1.read();
    for(uint8_t i = 5; i <= 9; i++) {
        if(((pending >> i) & 0x01)) {
            RODOS::EXTI::EXTIRQHandler(i);
        }
    }
    NVIC::clearPendingIRQ(EXTI::EXTI9_5_IRQn);
}
#endif

#ifndef RODOS_UNMOUNT_IRQ_EXTI15_10
void EXTI15_10_IRQHandler() {
    uint32_t pending = exti.PR1.read();
    for(uint8_t i = 10; i <= 15; i++) {
        if(((pending >> i) & 0x01)) {
            RODOS::EXTI::EXTIRQHandler(i);
        }
    }
    NVIC::clearPendingIRQ(EXTI::EXTI15_10_IRQn);
}
#endif

} //extern "C"
