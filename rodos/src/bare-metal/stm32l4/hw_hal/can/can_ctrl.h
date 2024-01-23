#pragma once

#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"
#include "hw_hal_can.h"

#include "hal/hal_can.h"
#include "rodos.h"

namespace RODOS {

namespace CANTimingConfig {
    static constexpr uint8_t t_RJW = 1;
    static constexpr uint8_t t_SYNC = 1;
    static constexpr uint8_t t_BS1 = 13;
    static constexpr uint8_t t_BS2 = 2;
    static constexpr uint8_t bitTime = t_SYNC + t_BS1 + t_BS2;
    static constexpr uint32_t baudrateNoPrescaler = PCLK::freq1/bitTime;
    static_assert((PCLK::freq1%bitTime) == 0);
}


class CAN_Ctrl {
    static constexpr bool debugEnableLoopback = false;

    static constexpr uint8_t numFiltersSingle = 14;
    static constexpr uint8_t numFiltersTotal = numFiltersSingle*mcu_specific::CANS::getNumPeriphs();

    static constexpr int8_t numTxMailboxes = 3;
    static constexpr uint8_t getFirstFilterIndex(CAN_IDX idx);

    bool initialized;
    const CAN_IDX idx;
    GPIO_PIN rxPin;
    GPIO_PIN txPin;
    CANStruct& can;
    HAL_CAN* owners[numTxMailboxes];
    static constinit HW_HAL_CAN* filterOwners[numFiltersTotal];
    int8_t currentTxMailboxIndex;
    uint8_t currentFilterIndex;

    bool arePinsSet() const;
    void setInitMode(bool enable);
    void configurePin(GPIO_PIN pinIdx);
    void configurePins();
    void configureBaudrate(uint32_t baudrate);
    void enableDebugOptions();
    void enableInterrupts();
    void initClocks();
    int16_t reserveFilter(HW_HAL_CAN* const filterOwner);
    void configureFilter(uint8_t filterIndex, uint32_t ID, uint32_t IDMask, bool extID, bool rtr);

    CanErrorMsg getRxError() const;
    CanErrorMsg getTxError() const;
public:
    void init(uint32_t baudrate);
    void setPins(GPIO_PIN rxPin, GPIO_PIN txPin);
    bool isUnusedMailboxAvailable() const;
    int8_t addOwner(HAL_CAN* owner);
    bool addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr, HW_HAL_CAN* filterOwner);

    CanErrorMsg status(CAN_STATUS_TYPE type) const;
    bool isMailboxFull(int8_t mailboxNum) const;

    void TxIRQHandler();
    void RxIRQHandler();

    bool isWriteFinished(int8_t mailbox);
    bool putIntoTxMailbox(int8_t mailbox, const CANMsg& msg);

    CAN_Ctrl(CAN_IDX idx);
};

extern constinit CAN_Ctrl* volatile canCtrls[mcu_specific::CANS::getNumPeriphs()];


}
