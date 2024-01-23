#pragma once
#include "hal/hal_adc.h"

namespace RODOS {

class HW_HAL_ADC {
public:
    static void init();
    static uint16_t read(ADC_CHANNEL channel);
};

}
