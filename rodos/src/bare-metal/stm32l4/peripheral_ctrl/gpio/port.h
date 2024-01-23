#pragma once

#include "rodos.h"

#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"
#include "peripherals.h"
#include "peripheral_ctrl/register_maps/gpio_registers.h"
#include "cortex_m/register_access/masked_register.h"


namespace RODOS {

namespace GPIO {

class Port {

    bool needsVddIO2();

public:
    GPIOStruct* const gpio;



    static constexpr inline uint8_t getPortNum(GPIO_PIN pinIdx)
    {
        return static_cast<uint8_t>(((uint32_t)pinIdx) / 16);
    }

    Port(GPIO_PIN pinIdx) : gpio( (pinIdx==GPIO_INVALID) ? 0 :  &gpios[getPortNum(pinIdx)]){}

    uint8_t getPortNum();
    HWRCCPeriph& getRCCPeriph();

    void enableClock();

    inline bool isValid(){
        return gpio != 0;
    }

};

}

}
