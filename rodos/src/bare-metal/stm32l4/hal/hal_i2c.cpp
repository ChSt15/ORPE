#include "peripheral_ctrl/aborting_error.h"
#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"

#include "hal/hal_i2c.h"
#include "rodos.h"
#include "hw_hal/hw_hal_i2c.h"

#include <new>

namespace RODOS {

constinit HW_HAL_I2C* HW_HAL_I2C::I2C_contextArray[3];

HAL_I2C::HAL_I2C(I2C_IDX idx, GPIO_PIN sclPin, GPIO_PIN sdaPin) {
    if(HW_HAL_I2C::I2C_contextArray[idx]!=nullptr){
        abortingError("Multiple uses of the same I2C_IDX");
    }
    HW_HAL_I2C::I2C_contextArray[idx] = xmallocAndTrapOnFailure<HW_HAL_I2C>();
    context = new (HW_HAL_I2C::I2C_contextArray[idx]) HW_HAL_I2C(this, idx, sclPin, sdaPin);
}

int32_t HAL_I2C::init(uint32_t speed) {
    context->init(speed);
    return 0;
}


void HAL_I2C::reset() {
    while(1);//TODO
}


bool HAL_I2C::isWriteFinished(){
    return true;
}


bool HAL_I2C::isReadFinished(){
    return true;
}


int32_t HAL_I2C::write(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize) {
    return context->write(addr, txBuf, txBufSize, true);
}


int32_t HAL_I2C::read(const uint8_t addr, uint8_t* rxBuf, uint32_t rxBufSize) {
    return context->read(addr, rxBuf, rxBufSize);
}


int32_t HAL_I2C::writeRead(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize, uint8_t* rxBuf, uint32_t rxBufSize) {
    int32_t writeReturn = context->write(addr, txBuf, txBufSize, false);
    if(writeReturn<0){
        return writeReturn;
    }else{
        return context->read(addr, rxBuf, rxBufSize);
    }
}

}
