#include "rodos.h"
#include <new>

#include "hw_hal/hw_hal_spi.h"

namespace RODOS {


HAL_SPI::HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin)
{
    context = new (xmallocAndTrapOnFailure<HW_HAL_SPI>()) HW_HAL_SPI(idx, sckPin, misoPin, mosiPin, nssPin);
}

int32_t HAL_SPI::init(uint32_t baudrate, bool slave, bool tiMode){
    if(slave){
        RODOS_ERROR("SPI slave mode currently not supported");
        return -1;
    }
    return context->init(baudrate, tiMode);
}

bool HAL_SPI::isWriteFinished(){
    return true;
}

void HAL_SPI::reset() {
    context->reset();
}

int32_t HAL_SPI::config(SPI_PARAMETER_TYPE type, int32_t value) {
    switch(type){
    case SPI_PARAMETER_BAUDRATE:
        return context->setBaudrate(static_cast<uint32_t>(value)) ? 0 : -1;
    case SPI_PARAMETER_MODE:
        if (value < 0 || value > UINT8_MAX) {
            return -1;
        }
        return context->setMode(static_cast<uint8_t>(value)) ? 0 : -1;
    default:
        return -1;
    }
}

int32_t HAL_SPI::status(SPI_STATUS_TYPE type) {
    switch(type){
    case SPI_PARAMETER_BAUDRATE:
        return *reinterpret_cast<int32_t*>(&(context->baudrate));
    default:
        return -1;
    }
}

bool HAL_SPI::isReadFinished(){
    return true;
}

int32_t HAL_SPI::write(const void* sendBuf, size_t len) {
    if(len > INT32_MAX) len = INT32_MAX;
    bool ok = context->writeRead(sendBuf, len, nullptr, 0);
    return ok ? static_cast<int32_t>(len) : -1;
}


int32_t HAL_SPI::read(void* recBuf, size_t maxLen) {
    if(maxLen > INT32_MAX) maxLen = INT32_MAX;
    bool ok = context->writeRead(nullptr, 0, recBuf, maxLen);
    return ok ? static_cast<int32_t>(maxLen) : -1;
}

int32_t HAL_SPI::writeRead(const void* sendBuf, size_t len, void* recBuf, size_t maxLen) {
    if(maxLen > INT32_MAX) maxLen = INT32_MAX;
    bool ok = context->writeRead(sendBuf, len, recBuf, maxLen);
    return ok ? static_cast<int32_t>(maxLen) : -1;
}

}
