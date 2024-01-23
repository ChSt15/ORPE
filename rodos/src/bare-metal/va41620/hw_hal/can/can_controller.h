#pragma once

#include "hw_hal/can/can_filter.h"
#include "hw_hal/can/can_message.h"
#include "hal/hal_can.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/register_maps/can_registers.h"

#include <atomic>
#include <cstdint>

class CanTest;

namespace RODOS {

namespace CANTimingConfig {
    static constexpr uint8_t t_RJW { 1 };
    static constexpr uint8_t t_SYNC { 1 };
    static constexpr uint8_t t_BS1 { 7 };
    static constexpr uint8_t t_BS2 { 2 };
    static constexpr uint8_t bitTime { t_SYNC + t_BS1 + t_BS2 };
    static constexpr uint32_t baudrateNoPrescaler { globalClockSetup.getAPB1Frequency()/bitTime };
    static_assert((globalClockSetup.getAPB1Frequency()%bitTime) == 0);
}

class HW_HAL_CAN;

class CanController {
public:
    static constexpr int8_t NUM_BUFFERS_TOTAL { 15 };
    static constexpr int8_t NUM_RX_BUFFERS { 3 };
    static constexpr int8_t NUM_TX_BUFFERS { NUM_BUFFERS_TOTAL-NUM_RX_BUFFERS };
    static constexpr int8_t FIRST_TX_BUFFER_INDEX { NUM_RX_BUFFERS };
    static constexpr int8_t NUM_SOFTWARE_FILTERS { 10 };

    static CanController& getSingleton(CAN_IDX idx);

    void init(uint32_t baudrate);
    bool isUnusedMailboxAvailable() const;
    int8_t addOwner(HAL_CAN* owner);
    bool addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr, HW_HAL_CAN* filterOwner);

    CAN_ERROR_TYPE getErrorType();
    uint8_t getRxErrorCounter() const;
    uint8_t getTxErrorCounter() const;

    static void callIrqHandler(CAN_IDX idx);

    bool isWriteFinished(int8_t mailbox);
    bool putIntoTxBuffer(int8_t mailbox, const CanMessage& msg);
    bool isTxBufferEmpty(int8_t bufferIndex) const;
    friend class ::CanTest; ///<unfortunately necessary to reset the singletons on each test
private:
    static CanController* singletons[2];

    CanFilter m_softwareFilters[NUM_SOFTWARE_FILTERS] {};
    HAL_CAN* m_txBufferOwners[NUM_TX_BUFFERS] {};
    std::atomic_flag m_initialized { false };
    const CAN_IDX m_idx;
    std::atomic<int8_t> m_usedTxBufferCounter { 0 };
    std::atomic<int8_t> m_usedFilterCounter { 0 };
    CANStruct& m_can;

    CanController(CAN_IDX idx);

    void irqHandler();
    void handleInterruptFromRxBuffer(int8_t bufferId);
    void handleInterruptFromTxBuffer(int8_t bufferId);
    void configureRxBuffers();
    void configureTxBuffers();
    void setGlobalMaskToMatchAnything();
    void configureBaudrate(uint32_t baudrate);
    void enableInterrupts();
    void initClock();
    void configureFilter(uint8_t filterIndex, uint32_t ID, uint32_t IDMask, bool extID);
    uint8_t getIRQn() const;
};


}
