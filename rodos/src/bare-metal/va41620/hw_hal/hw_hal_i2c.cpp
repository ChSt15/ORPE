#include "peripheral_ctrl/aborting_error.h"
#include "hw_hal/hw_hal_i2c.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "clkgen_config.h"
#include "rodos-debug.h"

namespace RODOS {

HW_HAL_I2C::HW_HAL_I2C(I2C_IDX idx)
: m_i2c { i2cs[idx] }
, m_idx { idx }
{}

void HW_HAL_I2C::init(uint32_t speed) const{
    using namespace I2C_CLKSCALE;
    using namespace I2C_CTRL;
    if(speed > MAX_SPEED){
        PRINTF("Maximum I2C speed: %ld", static_cast<long>(MAX_SPEED));
        abortingError("I2C speed too high");
    }
    sysconfigPeripheralBitband->I2C_ENABLE[m_idx].write(1);
    m_i2c.CLKSCALE.write(VALUE(calculateClkscaleValue(speed)), FASTMODE(1));
    setClockLowTimeout(speed);
    m_i2c.CTRL.write(ENABLE(1), ALGFILTER(1));
}

void HW_HAL_I2C::setClockLowTimeout(uint32_t speed) const{
    constexpr float desiredTimeoutSeconds { 1e-3f };
    constexpr uint32_t maxTimeoutCycles { (1<<20)-1 }; //Register is a 20-bit value
    float timeoutCycles { desiredTimeoutSeconds*static_cast<float>(FAST_MODE_CLK_MULTIPLIER)*static_cast<float>(speed) };
    timeoutCycles = (timeoutCycles < maxTimeoutCycles) ? timeoutCycles : maxTimeoutCycles;
    timeoutCycles = (timeoutCycles > 1 ) ? timeoutCycles : 1;
    m_i2c.CLKTOLIMIT.write(static_cast<uint32_t>(timeoutCycles));
}

bool HW_HAL_I2C::hasClockLowTimeoutOccured() const{
    using namespace I2C_IRQ_ENB;
    return m_i2c.IRQ_RAW.read<CLKLOTO>();
}

void HW_HAL_I2C::resetClockLowTimeoutState() const{
    using namespace I2C_IRQ_ENB;
    m_i2c.IRQ_CLR.write(CLKLOTO(1));
}

uint8_t HW_HAL_I2C::calculateClkscaleValue(uint32_t speed) const{
    return static_cast<uint8_t>((FREQUENCY_BEFORE_CLKSCALE/speed)-1);
}


int32_t HW_HAL_I2C::read(const uint8_t address, uint8_t* rxBuffer, uint32_t rxSize) const{
    using namespace I2C_ADDRESS;
    using namespace I2C_STATUS;
    using namespace I2C_CMD;
    using namespace I2C_FIFO_CLR;

    if(rxBuffer==nullptr){
        return -1;
    }
    if(rxSize>MAX_TRANSFER_SIZE){
        RODOS_ERROR("I2C transfers are limited to 0x7FE bytes on the VA41620");
        return -1;
    }
    m_i2c.FIFO_CLR.write(RXFIFO(1));
    m_i2c.WORDS.write(rxSize);
    m_i2c.ADDRESS.write(DIRECTION(I2C_DIRECTION_RX), ADDRESS(address));
    m_i2c.CMD.write(START(1), STOP(1));

    uint32_t rxIndex { 0 };
    RegisterCopy status;
    do {
        status = m_i2c.STATUS;
        if(status.read<RXNEMPTY>() && rxIndex<rxSize){
            rxBuffer[rxIndex++] = static_cast<uint8_t>(m_i2c.DATA.read());
        }
        if(hasClockLowTimeoutOccured()){
            m_i2c.CMD.write(CANCEL(1));
            return HAL_I2C_ERR_READ;
        }
    } while(!status.read<IDLE>());

    if(status.read<NACKADDR>()){
        return HAL_I2C_ERR_ADDR;
    }else if(status.read<ARBLOST>()){
        return HAL_I2C_ERR_NOT_MASTER;
    }

    while(m_i2c.STATUS.read<RXNEMPTY>() && rxIndex<rxSize){
        rxBuffer[rxIndex++] = static_cast<uint8_t>(m_i2c.DATA.read());
    }
    uint32_t actualBytesRead { m_i2c.RXCOUNT.read() };
    bool wasCorrectNumberOfBytesRead { rxSize == actualBytesRead && rxSize == rxIndex };
    if(!wasCorrectNumberOfBytesRead){
        return HAL_I2C_ERR_READ;
    }

    return static_cast<int32_t>(rxSize);
}

int32_t HW_HAL_I2C::write(const uint8_t address, const uint8_t* txBuffer, uint32_t txSize, bool stop) const{
    using namespace I2C_ADDRESS;
    using namespace I2C_STATUS;
    using namespace I2C_CMD;
    using namespace I2C_FIFO_CLR;

    if(txBuffer==nullptr){
        return -1;
    }
    if(txSize>MAX_TRANSFER_SIZE){
        RODOS_ERROR("I2C transfers are limited to 0x7FE bytes on the VA41620");
        return -1;
    }
    m_i2c.FIFO_CLR.write(TXFIFO(1));
    m_i2c.WORDS.write(txSize);
    m_i2c.ADDRESS.write(DIRECTION(I2C_DIRECTION_TX), ADDRESS(address));
    m_i2c.CMD.write(START(1), STOP(stop?1:0));

    uint32_t txIndex { 0 };
    RegisterCopy status;
    do {
        status = m_i2c.STATUS;
        if( status.read<TXNFULL>() && (txIndex<txSize) ){
            m_i2c.DATA.write(txBuffer[txIndex++]);
        }
        if(hasClockLowTimeoutOccured()){
            m_i2c.CMD.write(CANCEL(1));
            return HAL_I2C_ERR_WRITE;
        }
    } while(!(status.read<IDLE>() || status.read<WAITING>()));

    if(status.read<NACKADDR>()){
        return HAL_I2C_ERR_ADDR;
    }else if(status.read<NACKDATA>()){
        return HAL_I2C_ERR_WRITE;
    }else if(status.read<ARBLOST>()){
        return HAL_I2C_ERR_NOT_MASTER;
    }

    uint32_t actualBytesWritten { m_i2c.TXCOUNT.read() };
    bool wasCorrectNumberOfBytesWritten { txSize == actualBytesWritten && txSize == txIndex };
    if(!wasCorrectNumberOfBytesWritten){
        return HAL_I2C_ERR_WRITE;
    }
    return static_cast<int32_t>(txSize);
}


}
