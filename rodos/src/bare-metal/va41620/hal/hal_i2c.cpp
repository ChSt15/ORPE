#include "hal/hal_i2c.h"
#include "hw_hal/hw_hal_i2c.h"
#include "misc-rodos-funcs.h"
#include "rodos-debug.h"

#include <new>

namespace RODOS {

HAL_I2C::HAL_I2C(I2C_IDX idx)
: context { xmallocAndTrapOnFailure<HW_HAL_I2C>() }
{
    if(idx > I2C_IDX2){
        __builtin_trap();
    }
    new (context) HW_HAL_I2C(idx);
}

HAL_I2C::HAL_I2C(I2C_IDX idx, GPIO_PIN, GPIO_PIN)
: HAL_I2C { idx }
{}

int32_t HAL_I2C::init(uint32_t speed) {
    context->init(speed);
    return 0;
}


void HAL_I2C::reset() {
    RODOS_ERROR("I2C Reset not implemented yet!");
}


bool HAL_I2C::isWriteFinished(){
    return true;
}


bool HAL_I2C::isReadFinished(){
    return true;
}


int32_t HAL_I2C::write(const uint8_t addr, const uint8_t* txBuf, uint32_t txSize) {
    return context->write(addr, txBuf, txSize, true);
}


int32_t HAL_I2C::read(const uint8_t addr, uint8_t* rxBuf, uint32_t rxSize) {
    return context->read(addr, rxBuf, rxSize);
}


int32_t HAL_I2C::writeRead(const uint8_t addr, const uint8_t* txBuf, uint32_t txSize, uint8_t* rxBuf, uint32_t rxSize) {
    int32_t writeReturn = context->write(addr, txBuf, txSize, false);
    if(writeReturn<0){
        return writeReturn;
    }else{
        return context->read(addr, rxBuf, rxSize);
    }
}

}
