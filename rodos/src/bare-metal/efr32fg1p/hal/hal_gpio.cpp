#include <new>
#include "rodos.h"
#include "hal/hal_gpio.h"
#include "hw_hal_gpio.h"

#include "vendor-headers.h"

namespace RODOS {

/***** class HW_HAL_GPIO *****/

HW_HAL_GPIO::HW_HAL_GPIO(GPIO_PIN pinIdx, uint8_t numOfPins, bool isOutput):
    pinIdx(pinIdx),numOfPins(numOfPins),isOutput(isOutput){

   	uint16_t pin = getEFR32Pin(pinIdx);

	PORT = getEFR32Port(pinIdx);

    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(PORT, pin, gpioModeInput, 0);	//configured as input

}

GPIO_Port_TypeDef HW_HAL_GPIO::getEFR32Port(GPIO_PIN pinIdx)
{
    int32_t portIdx = pinIdx / 16;
    return static_cast<GPIO_Port_TypeDef>(gpioPortA + portIdx);
}

uint16_t HW_HAL_GPIO::getEFR32Pin(GPIO_PIN pinIdx)
{
	return pinIdx % 16 ;
}


int32_t HW_HAL_GPIO::configurePin(GPIO_PIN pinIdx, GPIO_Mode_TypeDef pinMode, uint16_t dout)
{
    if (pinIdx < GPIO_000 || pinIdx > GPIO_095) return -1;
    GPIO_Port_TypeDef port = getEFR32Port(pinIdx);
    uint16_t pin = getEFR32Pin(pinIdx);

    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(port, pin, pinMode, dout);

    return 0;
}


int32_t HW_HAL_GPIO::resetPin(GPIO_PIN pinIdx)
{
    if (pinIdx < GPIO_000 || pinIdx > GPIO_095) return -1;
    GPIO_Port_TypeDef port = getEFR32Port(pinIdx);
    uint16_t pin = getEFR32Pin(pinIdx);

    GPIO_PinModeSet(port, pin, gpioModeDisabled, 0);
    return 0;
}
/***** END class HW_HAL_GPIO *****/


HAL_GPIO::HAL_GPIO(GPIO_PIN pinIdx) {
	// context = new HW_HAL_GPIO(pinIdx,1,false);
	context = (HW_HAL_GPIO*)xmalloc(sizeof(HW_HAL_GPIO)); 	// dynamic memory allocation with RODOS function
	context = new (context) HW_HAL_GPIO(pinIdx, 1, false); 	// placement new to call constructor
	context->hal_gpio = this;
}


int32_t HAL_GPIO::init(bool isOutput, uint8_t numOfPins, uint32_t initVal)
{
	if (numOfPins > 0) context->numOfPins = numOfPins; // numOfPins has to be > 0 -> if new value is 0 keep the default value

	context->isOutput = isOutput;

	if (context->isOutput){
		config(GPIO_CFG_OUTPUT_ENABLE, 1);
	 	setPins(initVal);
	} 
	else {
	 	config(GPIO_CFG_OUTPUT_ENABLE, 0);
	}

	return 0;
}


int32_t HAL_GPIO::config(GPIO_CFG_TYPE type, uint32_t paramVal){
	switch (type){
		case GPIO_CFG_OUTPUT_ENABLE:
	 		if (paramVal > 0)
	 		{
				context->isOutput = true;
				HW_HAL_GPIO::configurePin(context->pinIdx, gpioModePushPull, 0); //configure as push-pull output
	 		}
	 		else {
	 			context->isOutput = false;
	 			HW_HAL_GPIO::configurePin(context->pinIdx, gpioModeInput, 0);	//configure as input
	 		}
	 		return 0;

	 	default: return -1;
	}
}


void HAL_GPIO::reset()
{
	HW_HAL_GPIO::resetPin(context->pinIdx);
}


void HAL_GPIO::setPins(uint32_t val) 
{
	uint16_t pin = HW_HAL_GPIO::getEFR32Pin(context->pinIdx);
	if(context->isOutput)
	{
		PRIORITY_CEILER_IN_SCOPE();
		if(val)
			GPIO_PinOutSet(context->PORT, pin);
		else
			GPIO_PinOutClear(context->PORT, pin);	
	}
}


uint32_t HAL_GPIO::readPins()
{
	uint16_t pin = HW_HAL_GPIO::getEFR32Pin(context->pinIdx);
	if (context->isOutput)
		return GPIO_PinOutGet(context->PORT, pin); 		
	return GPIO_PinInGet(context->PORT, pin);
}

bool HAL_GPIO::isDataReady(){
	// return context->interruptEventOcured;
	return true;
}

//When rising and/or falling edge occures dataReady() == true
void HAL_GPIO::interruptEnable([[gnu::unused]] bool enable){

}

// dataReady == false
void HAL_GPIO::resetInterruptEventStatus(){
	//context->interruptEventOcured=false;
}


}
