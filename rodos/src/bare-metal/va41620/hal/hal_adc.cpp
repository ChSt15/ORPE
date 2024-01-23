#include "hal/hal_adc.h"
#include "hw_hal/hw_hal_adc.h"
#include "misc-rodos-funcs.h"

#include <new>

namespace RODOS {

HAL_ADC::HAL_ADC(ADC_IDX idx)
: context { nullptr }
{
    if(idx>ADC_IDX0){
        __builtin_trap();
    }
}

int32_t HAL_ADC::init(ADC_CHANNEL){
    HW_HAL_ADC::init();
    return 0;
}

int32_t HAL_ADC::config(ADC_PARAMETER_TYPE, int32_t){
    return -1;
}

void HAL_ADC::reset(){}

uint16_t HAL_ADC::read(ADC_CHANNEL channel){
    return HW_HAL_ADC::read(channel);
}

}
