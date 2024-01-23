#pragma once

#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"

#include "hal/hal_can.h"
#include "rodos.h"

namespace RODOS {

struct CANMsg {
    uint32_t data[2];
    uint32_t canID;
    uint8_t len;
    bool extID;
    bool rtr;
};
class CAN_Ctrl;


class HW_HAL_CAN {
    const CAN_IDX idx;
    HAL_CAN* const owner;
    CAN_Ctrl& ctrl;
    Fifo<CANMsg, 10> rxBuf;
    int8_t mailboxNum;

    CAN_Ctrl& getCANCtrl();
    void acquireMailbox();
public:
    void addRXMsg(const CANMsg& message);

    HW_HAL_CAN(CAN_IDX idx, GPIO_PIN rxPin, GPIO_PIN txPin, HAL_CAN* const owner);
    void init(uint32_t baudrate);
    CanErrorMsg status(CAN_STATUS_TYPE type) const;
    bool isWriteFinished() const;
    bool isDataReady();
    bool addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr);
    bool putIntoTxMailbox(const CANMsg& msg);
    int8_t read(uint8_t* recBuf, uint32_t* canID, bool* isExtID, bool* rtr);
};

}
