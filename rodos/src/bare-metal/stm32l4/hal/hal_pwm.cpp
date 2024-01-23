#include "hal/hal_pwm.h"
#include "rodos.h"
#include "hw_hal/hw_hal_pwm.h"

#include <new>

namespace RODOS {

HAL_PWM::HAL_PWM(PWM_IDX idx) {
    context = new(xmallocAndTrapOnFailure<HW_HAL_PWM>()) HW_HAL_PWM(idx);
}

int32_t HAL_PWM::init(uint32_t frequency, uint32_t increments) {
    return context->init(frequency, increments);
}

int32_t HAL_PWM::config(PWM_PARAMETER_TYPE type, int32_t paramVal) {
    return context->config(type, paramVal);
}

void HAL_PWM::reset() {
    context->reset();
}

int32_t HAL_PWM::write(uint32_t pulseWidthInIncs) {
    return context->write(pulseWidthInIncs);
}


}
