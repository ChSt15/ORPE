/*
 * Copyright (c) 2022 Uni Wuerzburg, Wuerzburg
 * All rights reserved.
 *
 * @author Tom Baumann
 */
#include <new>
#include "rodos.h"

#include "hal/hal_pwm.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_device.h"
#include "em_cmu.h"


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

typedef enum
{
    TIM_CHAN0,
    TIM_CHAN1,
    TIM_CHAN2,
    TIM_CHAN3
} TIM_CHAN_TypeDef;


class HW_HAL_PWM{
public:
    HW_HAL_PWM(PWM_IDX idx);

    uint32_t max_counter_increments; // maximum counter value of timer
	uint32_t increments; // Used increments for input duty cycle (max. max_counter_increments)
	uint32_t Frequency; //  Frequency of timer
	uint32_t duty_cycle_in_increments; // current Duty cycle in increments
	float duty_cycle; // current Duty cycle (duty_cycle_in_increments/increments)
	TIM_CHAN_TypeDef channel; // timer channel
    TIMER_TypeDef* timer; // Timer
    CMU_Clock_TypeDef timer_clock;
	uint32_t timerClk; // Timer clk
    uint16_t GPIO_Pin; // Timer gpio pin
    GPIO_Port_TypeDef GPIO_Port; // Timer GPIO port
	int PwmIdx; // IDX of pwm instance

	void updateSettings(); // update configuration (eg. duty cycle change)
};

HAL_PWM::HAL_PWM(PWM_IDX idx) {

	context = new (xmalloc(sizeof(HW_HAL_PWM))) HW_HAL_PWM(idx);
}


int32_t HAL_PWM::init(uint32_t frequency, uint32_t increments) {

    if ((static_cast<int>(context->PwmIdx) < static_cast<int>(PWM_IDX00)) || (static_cast<int>(context->PwmIdx) > static_cast<int>(PWM_IDX15))) {
        return -1;
    }
    static char pwmInit[4] = { 0, 0, 0, 0 };

    char timerIdx = 1;

    context->timerClk = CMU_ClockFreqGet(context->timer_clock);
    context->increments = increments;
    context->Frequency = frequency;

	/**** configure GPIO for timer-Use ****/
    CMU_ClockEnable(cmuClock_GPIO, true);
    // Enable clock for TIMER module
    CMU_ClockEnable(context->timer_clock, true);


    // Configure TIMER1 Compare/Capture for output compare
    // Use PWM mode, which sets output on overflow and clears on compare events
    TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
    timerCCInit.mode = timerCCModePWM;

    if(context->timer == TIMER1){
        switch (context->channel) {
            case TIM_CHAN0:
                // Configure as output
                context->GPIO_Pin = 0;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 0, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  TIMER_ROUTELOC0_CC0LOC_LOC0;
                break;
            case TIM_CHAN1:
                // Configure as output
                context->GPIO_Pin = 1;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 1, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  TIMER_ROUTELOC0_CC1LOC_LOC0;
                break;
            case TIM_CHAN2:
                // Configure as output
                context->GPIO_Pin = 2;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 2, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  TIMER_ROUTELOC0_CC2LOC_LOC0;
                break;
            case TIM_CHAN3:
                // Configure as output
                context->GPIO_Pin = 3;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 3, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  TIMER_ROUTELOC0_CC3LOC_LOC0;
                break;
        }
        timerIdx = 1;
    }else if (context->timer == WTIMER0){
        switch (context->channel) {
            case TIM_CHAN0:
                // Configure as output
                context->GPIO_Pin = 4;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 0, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  WTIMER_ROUTELOC0_CC0LOC_LOC4;
                break;
            case TIM_CHAN1:
                // Configure as output
                context->GPIO_Pin = 5;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 1, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  WTIMER_ROUTELOC0_CC1LOC_LOC3;
                break;
            case TIM_CHAN2:
                // Configure as output
                context->GPIO_Pin = 6;
                context->GPIO_Port = gpioPortA;
                TIMER_InitCC(context->timer, 2, &timerCCInit);
                // Route Timer CC to location and enable CC route pin
                context->timer->ROUTELOC0 |=  WTIMER_ROUTELOC0_CC2LOC_LOC2;
                break;
            case TIM_CHAN3:
                // no chan3 on wtimer
                break;
        }
        timerIdx = 2;
    }


	/* reset timer before new init, only if it's not already used as PWM source */
	if (pwmInit[timerIdx - 1] == 0) {
		//TIM_DeInit(context->timer);
		pwmInit[timerIdx - 1] = 1;
	}

    GPIO_PinModeSet(context->GPIO_Port, context->GPIO_Pin, gpioModePushPull, 0);

    context->updateSettings();
    // Initialize the timer
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    TIMER_Init(context->timer, &timerInit);
    return 0;
}

int32_t HAL_PWM::config(PWM_PARAMETER_TYPE type, int32_t paramVal) {
    if ((static_cast<int>(context->PwmIdx) < static_cast<int>(PWM_IDX00)) || (static_cast<int>(context->PwmIdx) > static_cast<int>(PWM_IDX15))) {
        return -1;
    }

    switch (type) {
	case PWM_PARAMETER_INCREMENTS:{
	    //TODO: Implement
		//context->increments = static_cast<uint32_t>(paramVal);
		break;
	}
	case PWM_PARAMETER_FREQUENCY:
		// TODO: Implement
		//context->Frequency = static_cast<uint32_t>(paramVal);
		break;
	}

	context->updateSettings();
    // Initialize the timer
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    TIMER_Init(context->timer, &timerInit);
	return 0;
}

void HAL_PWM::reset() {
	if ((static_cast<int>(context->PwmIdx) < static_cast<int>(PWM_IDX00)) || (static_cast<int>(context->PwmIdx) > static_cast<int>(PWM_IDX15))) {
        return;
    }

	// Reset Timer
	// FIXME: This disables all channels of this timer!
    TIMER_Reset(context->timer);
    GPIO_PinModeSet(context->GPIO_Port, context->GPIO_Pin, gpioModeDisabled, 0);

}

int32_t HAL_PWM::write(uint32_t pulseWidthInIncs) {
	if ((static_cast<int>(context->PwmIdx) < static_cast<int>(PWM_IDX00)) || (static_cast<int>(context->PwmIdx) > static_cast<int>(PWM_IDX15))) {
        return -1;
    }

    pulseWidthInIncs = min(context->increments, pulseWidthInIncs);

	if (pulseWidthInIncs <= 0){ // set output = 0
        switch (context->channel) {
            case TIM_CHAN0:
                context->timer->ROUTEPEN &= ~(WTIMER_ROUTEPEN_CC0PEN);
                break;
            case TIM_CHAN1:
                context->timer->ROUTEPEN &= ~(WTIMER_ROUTEPEN_CC1PEN);
                break;
            case TIM_CHAN2:
                context->timer->ROUTEPEN &= ~(WTIMER_ROUTEPEN_CC2PEN);
                break;
            case TIM_CHAN3:
                if(context->timer != WTIMER0) context->timer->ROUTEPEN &= ~(WTIMER_ROUTEPEN_CC3PEN);
                break;
        }
        return 0;
	}

	context->duty_cycle_in_increments = pulseWidthInIncs;
    context->duty_cycle = static_cast<float>(context->duty_cycle_in_increments) / context->increments;
    uint32_t timer_compare_value = context->duty_cycle*context->max_counter_increments;

    switch (context->channel) {
    case TIM_CHAN0:
        context->timer->ROUTEPEN |= WTIMER_ROUTEPEN_CC0PEN;
        TIMER_CompareBufSet(context->timer, 0, timer_compare_value);
        break;
    case TIM_CHAN1:
        context->timer->ROUTEPEN |= WTIMER_ROUTEPEN_CC1PEN;
        TIMER_CompareBufSet(context->timer, 1, timer_compare_value);
        break;
    case TIM_CHAN2:
        context->timer->ROUTEPEN |= WTIMER_ROUTEPEN_CC2PEN;
        TIMER_CompareBufSet(context->timer, 2, timer_compare_value);
        break;
    case TIM_CHAN3:
        if(context->timer != WTIMER0){
            context->timer->ROUTEPEN |= WTIMER_ROUTEPEN_CC3PEN;
            TIMER_CompareBufSet(context->timer, 3, timer_compare_value);
        }
        break;
    }

	upCallWriteFinished();

	return 0;
}



HW_HAL_PWM::HW_HAL_PWM(PWM_IDX idx){
    increments = 2;
    duty_cycle_in_increments = 1;
    Frequency = 0;
    PwmIdx = -1;
    timer = NULL;
    channel = TIM_CHAN1;
    timerClk = 0;
    GPIO_Pin = 0;


    if ((static_cast<unsigned int>(idx) > static_cast<unsigned int>(PWM_IDX15))) {
        RODOS_ERROR("Invalid PWM Index\n");
        return;
    }

    PwmIdx = idx;

    // conversion from index to Timer and channel
    switch (PwmIdx / 4) {
    case 0:
        timer = TIMER1;
        timer_clock = cmuClock_TIMER1;
        break;
    case 1:
        timer = WTIMER0;
        timer_clock = cmuClock_WTIMER0;
        break;
    default:
        return;
    }

    channel = (TIM_CHAN_TypeDef) (PwmIdx % 4);
}



void HW_HAL_PWM::updateSettings(){
    if (Frequency == 0){
        // End, so that output is 0
        return;
    }else{
        // normal pwm mode
    }
    // Set top value to overflow at the desired PWM_FREQ frequency
    TIMER_TopSet(timer, CMU_ClockFreqGet(timer_clock) / Frequency);
    max_counter_increments = TIMER_TopGet(timer);

    increments = min(max_counter_increments, increments);

    duty_cycle_in_increments = 0; // default duty cycle = minimum = 0
    duty_cycle = static_cast<float>(duty_cycle_in_increments) / increments;
    uint32_t timer_compare_value = duty_cycle*static_cast<float>(max_counter_increments);

    switch (channel) {
    case TIM_CHAN0:
        // Set compare value for initial duty cycle
        TIMER_CompareSet(timer, 0, timer_compare_value);
        break; // channel 1 init
    case TIM_CHAN1:
        TIMER_CompareSet(timer, 1, timer_compare_value);
        break; // channel 2 init
    case TIM_CHAN2:
        TIMER_CompareSet(timer, 2, timer_compare_value);
        break; // channel 3 init
    case TIM_CHAN3:
        if(timer != WTIMER0) TIMER_CompareSet(timer, 3, timer_compare_value);
        break; // channel 4 init
    default:
        return;
    }

}

#ifndef NO_RODOS_NAMESPACE
}
#endif

