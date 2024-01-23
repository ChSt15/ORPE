#ifndef _HW_HAL_GPIO_H_
#define _HW_HAL_GPIO_H_

#include "hal/hal_gpio.h"

#include "vendor-headers.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


class HW_HAL_GPIO {
    friend class HAL_GPIO;

protected:
    GPIO_PIN pinIdx;
    HAL_GPIO* hal_gpio;
    uint8_t numOfPins;
    bool isOutput;
    uint16_t pinMask;

    GPIO_Port_TypeDef PORT;

    //bool interruptEventOcured;

	HW_HAL_GPIO(GPIO_PIN pinIdx, uint8_t numOfPins, bool isOutput);

public:
	static int32_t configurePin(GPIO_PIN pinIdx, GPIO_Mode_TypeDef pinMode, uint16_t dout); // check efr32xg1-rm.pdf page 903 to configure
	static int32_t resetPin(GPIO_PIN pinIdx);

    static GPIO_Port_TypeDef getEFR32Port(GPIO_PIN pinIdx);
    static uint16_t getEFR32Pin(GPIO_PIN pinIdx);
};

#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif // _HW_HAL_GPIO_H_
