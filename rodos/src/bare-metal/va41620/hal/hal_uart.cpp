#include "hal/hal_uart.h"
#include "hw_hal/hw_hal_gpio.h"
#include <new>

#include "hw_hal/hw_hal_uart.h"

namespace RODOS {

HAL_UART::HAL_UART(UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin)
: context { xmallocAndTrapOnFailure<HW_HAL_UART>() }
{
    context = new(context) HW_HAL_UART{*this, uartIdx, txPin, rxPin, rtsPin, ctsPin};
}

int32_t HAL_UART::init(uint32_t iBaudrate) {
    return context->init(iBaudrate);
}

int32_t HAL_UART::config(UART_PARAMETER_TYPE type, int32_t paramVal) {
    switch(type) {
        case UART_PARAMETER_BAUDRATE:
            if(paramVal > 0) {
                context->setBaudrate(static_cast<uint32_t>(paramVal));
                return 0;
            } else {
                return -1;
            }
        default:
            return -1;
    }
}

void HAL_UART::reset() {
    context->reset();
}

size_t HAL_UART::read(void* buf, size_t size) {
    return context->read(buf, size);
}

size_t HAL_UART::write(const void* buf, size_t size) {
    return context->write(buf, size);
}

int16_t HAL_UART::getcharNoWait() {
    return context->getcharNoWait();
}

int16_t HAL_UART::putcharNoWait(uint8_t c) {
    return context->putcharNoWait(c);
}

int32_t HAL_UART::status(UART_STATUS_TYPE type) {
    switch(type) {
        case UART_STATUS_RX_BUF_LEVEL:
            return context->getRXBufLevel();
        case UART_STATUS_RX_ERROR:
            return context->getRXErrorCnt();
        default:
            return -1;
    }
}

bool HAL_UART::isWriteFinished() {
    return context->isWriteFinished();
}

bool HAL_UART::isDataReady() {
    return context->isDataReady();
}

}
