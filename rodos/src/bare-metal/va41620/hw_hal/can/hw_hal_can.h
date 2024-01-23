#pragma once

#include "fifo.h"
#include "hw_hal/can/can_controller.h"
#include "hal/hal_can.h"

namespace RODOS {

class HW_HAL_CAN {
public:
    HW_HAL_CAN(CAN_IDX idx, HAL_CAN* const owner);
    void init(uint32_t baudrate);
    bool isWriteFinished() const;
    bool isDataReady() const;
    bool addIncomingFilter(uint32_t ID, uint32_t IDMask, bool ide, bool rtr);
    int8_t read(uint8_t* recBuf, uint32_t* canID, bool* ide, bool* rtr);
    bool write(const uint8_t* sendBuf, uint8_t len, uint32_t canID, bool ide);
    CanErrorMsg status(CAN_STATUS_TYPE type);
    void addRxFrame(const CanMessage& message);
private:
    static constexpr auto RX_BUFFER_SIZE { 10 };

    HAL_CAN& m_owner;
    CanController& m_ctrl;
    Fifo<CanMessage, RX_BUFFER_SIZE> m_rxBuf;
    const int8_t m_txBufferIdx;
};

}
