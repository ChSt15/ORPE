#include "thread.h"
#include "hal/hal_can.h"
#include "rodos-debug.h"

class CanSendingTestThread : RODOS::StaticThread<> {
    RODOS::HAL_CAN m_can { RODOS::CAN_IDX0 };
    void init() override {
        m_can.init(1'000'000);
    }

    void run() override {
        int64_t testValue { 0 };
        uint16_t channelId { 0 };
        TIME_LOOP(1*RODOS::SECONDS, 1*RODOS::SECONDS){
            m_can.write(reinterpret_cast<uint8_t*>(&testValue), sizeof(testValue), channelId%(1<<11), false);
            RODOS::PRINTF("wrote value: 0x%llx\n", static_cast<long long unsigned>(testValue));
            testValue+=0x0102'0304'0506'0708;
            channelId++;
        }
    }
};


class CanReceivingTestThread : RODOS::StaticThread<> {
    RODOS::HAL_CAN m_can { RODOS::CAN_IDX1 };
    void init() override {
        m_can.init(1'000'000);
        m_can.addIncomingFilter(0, 0x1fff'ffff, true);
        m_can.addIncomingFilter(0, 0x7ff, false);
    }

    void run() override {
        while(1){
            int64_t readValue {};
            uint32_t id {};
            bool ide {};
            bool rtr {};
            m_can.suspendUntilDataReady();
            m_can.read(reinterpret_cast<uint8_t*>(&readValue), &id, &ide, &rtr);
            RODOS::PRINTF("received Message: 0x%llx, id: %ld, ide: %d, rtr: %d\n",
                static_cast<long long unsigned>(readValue),
                static_cast<long>(id),
                static_cast<int>(ide),
                static_cast<int>(rtr)
            );
        }
    }
};

CanSendingTestThread test1{};
CanReceivingTestThread test2{};
