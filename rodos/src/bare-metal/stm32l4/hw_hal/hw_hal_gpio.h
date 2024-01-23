#pragma once

#include "hal/hal_gpio.h"
#include "gpio/exti.h"

#include "peripheral_ctrl/gpio/pin_group.h"

namespace RODOS {

class HW_HAL_GPIO {
    friend void EXTI::EXTIRQHandler(uint8_t);

private:
    GPIO::PinGroup config;
    HAL_GPIO* const owner;

    void EXTIRQHandler();

public:
    HW_HAL_GPIO(GPIO_PIN pinIdx, HAL_GPIO* hal_gpio);

    int32_t init();

    void enableInterrupts();
    void disableInterrupts();

    GPIO_IRQ_SENSITIVITY irqSensitivity;
    bool interruptEventOcured;

    GPIO::PinGroup& getPinGroup();
};

}
