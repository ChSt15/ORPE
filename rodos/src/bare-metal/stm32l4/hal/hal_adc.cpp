#include <new>

#include "hal/hal_adc.h"
#include "hw_hal/hw_hal_adc.h"
#include "rodos.h"

namespace RODOS {

HAL_ADC::HAL_ADC(ADC_IDX idx)
: context { xmallocAndTrapOnFailure<HW_HAL_ADC>() }
{
    context = new(context) HW_HAL_ADC(idx);
}

int32_t HAL_ADC::init(ADC_CHANNEL)
{
    context->init();
    return 0;
}

int32_t HAL_ADC::config([[maybe_unused]] ADC_PARAMETER_TYPE type, [[maybe_unused]] int32_t value)
{
    RODOS_ERROR("[HAL_ADC::config] Function not implemented\n");
    return 0;
}

void HAL_ADC::reset()
{
    RODOS_ERROR("[HAL_ADC::reset] Function not implemented\n");
}

uint16_t HAL_ADC::read(ADC_CHANNEL channel)
{
    return context->read(channel);
}

}
