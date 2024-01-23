#include "hw_hal/can/hw_hal_can.h"
#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"

#include "hal/hal_can.h"
#include "rodos.h"
#include "hw_hal/hw_hal_gpio.h"

#include <new>


namespace RODOS {

HAL_CAN::HAL_CAN(CAN_IDX canIdx, GPIO_PIN rxPin, GPIO_PIN txPin) {
    context = new(xmallocAndTrapOnFailure<HW_HAL_CAN>()) HW_HAL_CAN(canIdx, rxPin, txPin, this);
}

int32_t HAL_CAN::init(uint32_t baudrate) {
    context->init(baudrate);
    return 0;
}

void HAL_CAN::reset() {
    while(1);
    //context->reset();
}

CanErrorMsg HAL_CAN::status(CAN_STATUS_TYPE type) {
    return context->status(type);
}

bool HAL_CAN::isWriteFinished() {
    return context->isWriteFinished();
}

bool HAL_CAN::isDataReady() {
    return context->isDataReady();
}

bool HAL_CAN::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr) {
    return context->addIncomingFilter(ID, IDMask, extID, rtr);
}

int8_t HAL_CAN::write(const uint8_t* sendBuf, uint8_t len, uint32_t canID, bool extID, bool rtr) {
    CANMsg msg = {
        {},
        canID,
        len,
        extID,
        rtr
    };
    memcpy(msg.data, sendBuf, len);

    if (context->putIntoTxMailbox(msg)) {
        return 0;
    }else{
        return -1;
    }
}

int8_t HAL_CAN::read(uint8_t* recBuf, uint32_t* canID, bool* isExtID, bool* rtr) {
    return context->read(recBuf, canID, isExtID, rtr);
}


}
