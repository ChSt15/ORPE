#pragma once
#include "hal/hal_gpio.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"

namespace RODOS {

class EXTI {
private:
    static constinit HW_HAL_GPIO* lineOwners[16];

    static void setPort(uint8_t extiLine, uint8_t portNum);
    static void setSensitivity(uint8_t extiLine, GPIO_IRQ_SENSITIVITY irqSensitivity);
public:
    static constexpr uint8_t EXTI0_IRQn = 6;
    static constexpr uint8_t EXTI1_IRQn = 7;
    static constexpr uint8_t EXTI2_IRQn = 8;
    static constexpr uint8_t EXTI3_IRQn = 9;
    static constexpr uint8_t EXTI4_IRQn = 10;
    static constexpr uint8_t EXTI9_5_IRQn = 23;
    static constexpr uint8_t EXTI15_10_IRQn = 40;

    static int32_t enable(uint8_t extiLine, HW_HAL_GPIO* owner, uint8_t portNum, GPIO_IRQ_SENSITIVITY irqSensitivity);
    static void reset(uint8_t extiLine);

    static void EXTIRQHandler(uint8_t extiLine);

    static void initEXTInterrupts();
};

}
