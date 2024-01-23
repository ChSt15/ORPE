#include "hw_hal_can.h"
#include "hw_hal/can/can_controller.h"
#include "peripheral_ctrl/peripheral_defs.h"

#include "hal/hal_can.h"
#include "hw_hal/hw_hal_gpio.h"

#include <new>


namespace RODOS {

HW_HAL_CAN::HW_HAL_CAN(CAN_IDX idx, HAL_CAN* owner)
    : m_owner { *owner }
    , m_ctrl { CanController::getSingleton(idx) }
    , m_rxBuf {}
    , m_txBufferIdx { m_ctrl.addOwner(owner) }
{}


void HW_HAL_CAN::init(uint32_t baudrate) {
    m_ctrl.init(baudrate);
}

bool HW_HAL_CAN::write(const uint8_t* sendBuf, uint8_t len, uint32_t canId, bool ide){
    if(len > sizeof(CanMessage::data)){
        RODOS_ERROR("HW_HAL_CAN::write len exceeds CAN payload limit");
        return false;
    }
    CanMessage message {
        .data = {},
        .metadata = {canId, ide, false},
        .length = len
    };
    memcpy(message.data, sendBuf, len);
    return m_ctrl.putIntoTxBuffer(m_txBufferIdx, message);
}

void HW_HAL_CAN::addRxFrame(const CanMessage& message){
    m_rxBuf.put(message);
    m_owner.upCallDataReady();
}

bool HW_HAL_CAN::isWriteFinished() const {
    return m_ctrl.isTxBufferEmpty(m_txBufferIdx);
}

bool HW_HAL_CAN::isDataReady() const {
    return !m_rxBuf.isEmpty();
}

bool HW_HAL_CAN::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool ide, bool rtr) {
    return m_ctrl.addIncomingFilter(ID, IDMask, ide, rtr, this);
}

int8_t HW_HAL_CAN::read(uint8_t* recBuf, uint32_t* canID, bool* ide, bool* rtr) {
    CanMessage message;
    if(!m_rxBuf.get(message)){
         return -1;
    }
    if(canID != nullptr){
        *canID = message.metadata.getId();
    }
    if (ide != nullptr){
        *ide = message.metadata.getIde();
    }
    if(rtr != nullptr){
        *rtr = message.metadata.getRTR();
    }
    if(recBuf != nullptr){
        memcpy(recBuf, message.data, message.length);
    }
    return static_cast<int8_t>(message.length);
}

CanErrorMsg HW_HAL_CAN::status(CAN_STATUS_TYPE type){
    switch(type){
    case CAN_STATUS_RX_LEVEL:
        return {static_cast<int>(m_rxBuf.getElementCount()), CAN_ERROR_TYPE::UNKNOWN};
    case CAN_STATUS_TX_ERROR:
        return {m_ctrl.getTxErrorCounter(), m_ctrl.getErrorType()};
    case CAN_STATUS_RX_ERROR:
        return {m_ctrl.getRxErrorCounter(), m_ctrl.getErrorType()};
    }
    return {0, CAN_ERROR_TYPE::UNKNOWN};
}

}
