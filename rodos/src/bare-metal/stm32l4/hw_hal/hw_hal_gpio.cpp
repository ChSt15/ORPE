#include "rodos.h"

#include "hal/hal_gpio.h"
#include "hw_hal_gpio.h"

using namespace GPIO;

namespace RODOS {

HW_HAL_GPIO::HW_HAL_GPIO(GPIO_PIN pinIdx, HAL_GPIO* hal_gpio)
    : config(pinIdx)
    , owner(hal_gpio)
{
    irqSensitivity = GPIO_IRQ_SENS_BOTH;
    interruptEventOcured = false;
}

int32_t HW_HAL_GPIO::init()
{
    if (!config.getPort().isValid()) {
        return -1;
    }
    config.enableClocks();
    config.setMode(MODE::INPUT);
    config.setSpeed(SPEED::HIGH);
    config.setPull(PULL::NONE);
    config.setOutputType(OUTPUTTYPE::PUSHPULL);
    return 0;
}

void HW_HAL_GPIO::EXTIRQHandler()
{
    interruptEventOcured = true;
    owner->upCallDataReady();
    exti.PR1.setBits(1u << config.getPinPos());
}

void HW_HAL_GPIO::enableInterrupts()
{
    uint8_t portNum = config.getPort().getPortNum();
    uint8_t exti = config.getPinPos();

    if (!config.isSinglePin()) {
        RODOS_ERROR("IRQ not possible with numOfPins > 1");
        return;
    }
    EXTI::enable(exti, this, portNum, this->irqSensitivity);
    this->interruptEventOcured = false;
}
void HW_HAL_GPIO::disableInterrupts()
{
    uint8_t exti = config.getPinPos();
    EXTI::reset(exti);
}

GPIO::PinGroup& HW_HAL_GPIO::getPinGroup()
{
    return config;
}

}
