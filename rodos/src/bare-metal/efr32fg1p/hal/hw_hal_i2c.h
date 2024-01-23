#ifndef _HW_HAL_I2C_H_
#define _HW_HAL_I2C_H_


#include "hw_hal_gpio.h"
#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

class HAL_I2C;

#define I2C_IDX_MIN I2C_IDX0
#define I2C_IDX_MAX I2C_IDX1

class HW_HAL_I2C {
	friend class HAL_I2C;
protected: // private?
	HW_HAL_I2C(I2C_IDX i2cIdx){
		idx = i2cIdx;
	}

	HW_HAL_I2C(){
		// this constructor is called to init I2C_contextArray-objects
		// !!! don't set member variables here !!! -> this might overwrite values already set by constructor call of placement new
	};

	I2C_IDX idx;
	I2C_TypeDef* I2Cx;

	GPIO_PIN GPIO_Pin_SCL;
	GPIO_PIN GPIO_Pin_SDA;

	bool isMaster;
	bool initialized;

  void initMembers(GPIO_PIN sclPin, GPIO_PIN sdaPin);
  int8_t getPinLoc(GPIO_PIN gpio, const uint8_t * lut);
  int32_t enableClocks();

	static HW_HAL_I2C I2C_contextArray[I2C_IDX_MAX+1];
};

#ifndef NO_RODOS_NAMESPACE
}
#endif
#endif // _HW_HAL_I2C_H_
