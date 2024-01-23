#pragma once

#include "hal/hal_i2c.h"
#include "rodos.h"
#include "hw_hal_gpio.h"
namespace RODOS {


class HW_HAL_I2C {
private:
    I2C_IDX idx;
    I2CStruct& i2c;
    HAL_I2C* const owner;

    const GPIO_PIN sclPin;
    const GPIO_PIN sdaPin;


    static uint32_t getTimingRegValue(uint32_t speed);
    void configureTiming(uint32_t speed);
    void configurePin(GPIO_PIN idx);
public:
    HW_HAL_I2C(HAL_I2C* owner, I2C_IDX i2cIdx, GPIO_PIN sclPin, GPIO_PIN sdaPin);
    void init(uint32_t speed);

    int32_t read(const uint8_t addr, uint8_t* rxBuf, uint32_t rxBufSize);
    int32_t write(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize, bool stop);

    static constinit HW_HAL_I2C* I2C_contextArray[3];

};

}
