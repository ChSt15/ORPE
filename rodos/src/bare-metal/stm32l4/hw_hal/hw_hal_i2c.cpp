#include "peripheral_ctrl/aborting_error.h"
#include "hw_hal_i2c.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"

namespace RODOS {

HW_HAL_I2C::HW_HAL_I2C(HAL_I2C* owner, I2C_IDX i2cIdx, GPIO_PIN sclPin, GPIO_PIN sdaPin)
    : idx(i2cIdx)
    , i2c(mcu_specific::I2CS::getPeriphStruct(idx))
    , owner(owner)
    , sclPin(sclPin)
    , sdaPin(sdaPin)
    {}

void HW_HAL_I2C::init(uint32_t speed) {
    using namespace I2C_CR1;
    mcu_specific::I2CS::getRCCPeriph(idx).enableClock();

    configurePin(sdaPin);
    configurePin(sclPin);

    configureTiming(speed);

    i2c.CR1.write(PE(1));
}

uint32_t HW_HAL_I2C::getTimingRegValue(uint32_t speed){ //calculating these values is pretty difficult, so I'm just using pre-calculated values from CubeMX
    //all timings for fast mode
    if constexpr(PCLK::freq1==80'000'000){
        switch(speed){
        case 400'000:
            return 0x0070'2991ul;
        case 100'000:
            return 0x1030'89FFul;
        case 10'000:
            return 0xF000'F3FEul;
        }
    }else if constexpr(PCLK::freq1==16'000'000){
        switch(speed){
        case 400'000:
            return 0x0010'061Aul;
        case 100'000:
            return 0x0010'0A8Eul;
        case 10'000:
            return 0x4000'3EFFul;
        }
    }
    abortingError("Unsupported I2C speed\n");
}

void HW_HAL_I2C::configureTiming(uint32_t speed) {
    i2c.TIMINGR.write(getTimingRegValue(speed));
}


void HW_HAL_I2C::configurePin(GPIO_PIN pinIdx){
    GPIO::PinGroup pinGrp = GPIO::PinGroup(pinIdx);
    if (!pinGrp.isValid()) {
        __builtin_trap();
    }
    pinGrp.enableClocks();
    pinGrp.setPull(GPIO::PULL::UP);
    pinGrp.setOutputType(GPIO::OUTPUTTYPE::OPENDRAIN);
    pinGrp.setAlternateFunction(mcu_specific::I2CS::getAlternateFunction(idx, pinIdx));
    pinGrp.setMode(GPIO::MODE::ALTERNATE);
    pinGrp.setSpeed(GPIO::SPEED::HIGH);
}

int32_t HW_HAL_I2C::read(const uint8_t addr, uint8_t* rxBuf, uint32_t rxBufSize){
    using namespace I2C_ISR;
    using namespace I2C_ICR;
    using namespace I2C_CR1;
    using namespace I2C_CR2;

    if(rxBufSize>255){
        RODOS_ERROR("currently max 255 bytes i2c transfers are supported");
        return -1;
    }
    i2c.CR2.write(SADD_7(addr), RD_WRN(1), AUTOEND(1), START(1), NBYTES(rxBufSize));
    do {
        RegisterCopy isr = i2c.ISR;
        if (isr.read<NACKF>()){
            i2c.ICR.write(NACKCF(1));
            return HAL_I2C_ERR_ADDR;
        }
        if(isr.read<BERR>()){
            i2c.ICR.write(BERRCF(1));
            return HAL_I2C_ERR_READ;
        }
        if(isr.read<ARLO>()){
            i2c.ICR.write(ARLOCF(1));
            return HAL_I2C_ERR_NOT_MASTER;
        }
        if(isr.read<RXNE>()){
            *(rxBuf++) = static_cast<uint8_t>(i2c.RXDR.read());
        }
        if(isr.read<STOPF>()){
            i2c.ICR.write(STOPCF(1));
            break;
        }
    } while(true);
    i2c.CR2.write(0ul);
    return static_cast<int32_t>(rxBufSize);
}

int32_t HW_HAL_I2C::write(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize, bool stop){
    using namespace I2C_ISR;
    using namespace I2C_ICR;
    using namespace I2C_CR1;
    using namespace I2C_CR2;

    if(txBufSize>255){
        RODOS_ERROR("currently max 255 bytes i2c transfers are supported");
        return -1;
    }
    i2c.CR2.write(SADD_7(addr), START(1), NBYTES(txBufSize), AUTOEND(stop));
    do {
        RegisterCopy isr = i2c.ISR;
        if (isr.read<NACKF>()){
            i2c.ICR.write(NACKCF(1));
            return HAL_I2C_ERR_ADDR;
        }
        if(isr.read<BERR>()){
            i2c.ICR.write(BERRCF(1));
            return HAL_I2C_ERR_WRITE;
        }
        if(isr.read<ARLO>()){
            i2c.ICR.write(ARLOCF(1));
            return HAL_I2C_ERR_NOT_MASTER;
        }
        if(isr.read<TXIS>()){
            i2c.TXDR.write((uint32_t) *(txBuf++));
        }
        if(stop){
            if(isr.read<STOPF>()){
                i2c.ICR.write(STOPCF(1));
                i2c.CR2.write(0ul);
                break;
            }
        }else{
            if(isr.read<TC>()){
                break;
            }
        }
        
    } while(true);
    return static_cast<int32_t>(txBufSize);
}


} //namespace RODOS
