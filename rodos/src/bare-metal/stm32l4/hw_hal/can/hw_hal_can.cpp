#include "hw_hal_can.h"
#include "can_ctrl.h"
#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"

#include "hal/hal_can.h"
#include "rodos.h"
#include "hw_hal/hw_hal_gpio.h"

#include <new>


namespace RODOS {

HW_HAL_CAN::HW_HAL_CAN(CAN_IDX idx, GPIO_PIN rxPin, GPIO_PIN txPin, HAL_CAN* const owner)
    : idx(idx)
    , owner(owner)
    , ctrl(getCANCtrl())
{
    if (rxPin != GPIO_INVALID){
        ctrl.setPins(rxPin, txPin);
    }
    acquireMailbox();
}

CAN_Ctrl& HW_HAL_CAN::getCANCtrl(){
    if(canCtrls[idx]==nullptr){
        canCtrls[idx] = new(xmalloc<CAN_Ctrl>()) CAN_Ctrl(idx);
    }
    return *canCtrls[idx];
}

void HW_HAL_CAN::acquireMailbox(){
    mailboxNum = ctrl.addOwner(owner);
    if (mailboxNum < 0) {
        __builtin_trap();
    }
}

void HW_HAL_CAN::init(uint32_t baudrate) {
    canCtrls[idx]->init(baudrate);
}

bool HW_HAL_CAN::putIntoTxMailbox(const CANMsg& msg){
    return ctrl.putIntoTxMailbox(mailboxNum, msg);
}

void HW_HAL_CAN::addRXMsg(const CANMsg& message){
    rxBuf.put(message);
    owner->upCallDataReady();
}

CanErrorMsg HW_HAL_CAN::status(CAN_STATUS_TYPE type) const {
    return ctrl.status(type);
}

bool HW_HAL_CAN::isWriteFinished() const {
    return !ctrl.isMailboxFull(mailboxNum);
}

bool HW_HAL_CAN::isDataReady() {
    return !rxBuf.isEmpty();
}

bool HW_HAL_CAN::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr) {
    return ctrl.addIncomingFilter(ID, IDMask, extID, rtr, this);
}

int8_t HW_HAL_CAN::read(uint8_t* recBuf, uint32_t* canID, bool* isExtID, bool* rtr) {
    CANMsg msg;
    if(!rxBuf.get(msg)){
        return -1;
    }
    if(canID != nullptr){
        *canID = msg.canID;
    }
    if (isExtID != nullptr){
        *isExtID = msg.extID;
    }
    if(rtr != nullptr){
        *rtr = msg.rtr;
    }
    if(recBuf != nullptr){
        memcpy(recBuf, msg.data, 8);
    }
    return static_cast<int8_t>(msg.len);
}

}
