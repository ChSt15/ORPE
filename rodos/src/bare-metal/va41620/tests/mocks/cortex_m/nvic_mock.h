#pragma once
#include <cstdint>
#include <cstddef>
#include "gmock/gmock.h"
#include "cortex_m/nvic.h"

namespace RODOS::testing {

class NVICMock{
public: 
    MOCK_METHOD(void, enableIRQ, (uint8_t irq));
    MOCK_METHOD(void, disableIRQ, (uint8_t irq));
    MOCK_METHOD(void, clearPendingIRQ, (uint8_t irq));
    MOCK_METHOD(void, setIRQPriority, (uint8_t irq, uint8_t priority));
    MOCK_METHOD(void, setSystemHandlerPriority, (NVIC::SYSTEM_HANDLER irq, uint8_t priority));
    
    MOCK_METHOD(void, setAllIrqsToDefaultPriority, ());

    MOCK_METHOD(void, systemReset, ());
};

extern ::testing::StrictMock<NVICMock>* globalNVICMock;

}
