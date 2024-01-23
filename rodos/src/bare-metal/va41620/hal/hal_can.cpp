#include "hal/hal_can.h"
#include "hw_hal/can/hw_hal_can.h"

#include <new>

namespace RODOS {

HAL_CAN::HAL_CAN(CAN_IDX canIdx, GPIO_PIN, GPIO_PIN)
: context { xmallocAndTrapOnFailure<HW_HAL_CAN>() }
{
    new (context) HW_HAL_CAN(canIdx, this);
}

int32_t HAL_CAN::init(uint32_t baudrate){
    context->init(baudrate);
    return 0;
}

bool HAL_CAN::isWriteFinished(){
    return context->isWriteFinished();
}

bool HAL_CAN::isDataReady(){
    return context->isDataReady();
}

bool HAL_CAN::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr){
    return context->addIncomingFilter(ID, IDMask, extID, rtr);
}

int8_t HAL_CAN::write(const uint8_t* sendBuf, uint8_t len, uint32_t canID, bool extID, bool rtr){
    if(rtr){
        RODOS_ERROR("VA41620 HAL_CAN doesn't support RTR");
        return -1;
    }else{
        return context->write(sendBuf, len, canID, extID) ? 0 : -1;
    }
}

int8_t HAL_CAN::read(uint8_t* recBuf, uint32_t* canID, bool* isExtID, bool* rtr){
    return context->read(recBuf, canID, isExtID, rtr);
}

CanErrorMsg HAL_CAN::status(CAN_STATUS_TYPE type){
    return context->status(type);
}

}
