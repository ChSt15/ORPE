#include "rodos.h"

#include "hal/hal_gpio.h"
#include "hw_hal/hw_hal_gpio.h"
#include <new>

namespace RODOS {

HAL_GPIO::HAL_GPIO(GPIO_PIN pinIdx)
{
    context = (HW_HAL_GPIO*)xmallocAndTrapOnFailure(sizeof(HW_HAL_GPIO)); // dynamic memory allocation with RODOS function
    context = new (context) HW_HAL_GPIO(pinIdx, this); // placement new to call constructor
}

int32_t HAL_GPIO::init(bool isOutput, uint8_t numOfPins, uint32_t initVal)
{
    GPIO::PinGroup& pinGroup = context->getPinGroup();
    if (numOfPins > 0)
        pinGroup.setNumPins(numOfPins);

    if (context->init() == 0) {
        if (isOutput) {
            pinGroup.setMode(GPIO::MODE::OUTPUT);
        }
        pinGroup.setOutput(static_cast<uint16_t>(initVal));
        return 0;
    } else {
        return -1;
    }
}

int32_t HAL_GPIO::config(GPIO_CFG_TYPE type, uint32_t paramVal)
{
    GPIO::PinGroup& pinGroup = context->getPinGroup();
    if (!pinGroup.isValid()) {
        RODOS_ERROR("HAL_GPIO: PinGroup instance is not valid (no port)\n");
        return -1;
    }
    switch (type) {
    case GPIO_CFG_OUTPUT_ENABLE:
        pinGroup.setMode(GPIO::MODE::OUTPUT);
        return 0;

    case GPIO_CFG_NUM_OF_PINS:
        pinGroup.setNumPins(paramVal);
        return 0;

    case GPIO_CFG_PULLUP_ENABLE:
        pinGroup.setPull(paramVal ? GPIO::PULL::UP : GPIO::PULL::NONE);
        return 0;

    case GPIO_CFG_PULLDOWN_ENABLE:
        pinGroup.setPull(paramVal ? GPIO::PULL::DOWN : GPIO::PULL::NONE);
        return 0;

    case GPIO_CFG_IRQ_SENSITIVITY:
        if (paramVal <= GPIO_IRQ_SENS_FALLING) {
            context->irqSensitivity = (RODOS::GPIO_IRQ_SENSITIVITY)paramVal;
            return 0;
        }
        return -1;

    default:
        return -1;
    }
}

void HAL_GPIO::reset()
{
    GPIO::PinGroup& pinGroup = context->getPinGroup();
    interruptEnable(false);
    pinGroup.reset();
}

void HAL_GPIO::setPins(uint32_t val)
{
    GPIO::PinGroup& pinGroup = context->getPinGroup();
    pinGroup.setOutput(static_cast<uint16_t>(val));
}

uint32_t HAL_GPIO::readPins()
{
    GPIO::PinGroup& pinGroup = context->getPinGroup();
    return pinGroup.read();
}

bool HAL_GPIO::isDataReady()
{
    return context->interruptEventOcured;
}

//When rising and/or falling edge occures dataReady() == true
void HAL_GPIO::interruptEnable(bool enable)
{
    if (enable) {
        context->enableInterrupts();
    } else {
        context->disableInterrupts();
    }
}

void HAL_GPIO::resetInterruptEventStatus()
{
    context->interruptEventOcured = false;
}

}
