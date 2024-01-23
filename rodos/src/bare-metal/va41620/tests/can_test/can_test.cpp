#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "hal/hal_can.h"
#include "hw_hal/can/can_controller.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "tests/peripheral_test_fixture.h"
#include "cortex_m/nvic_mock.h"
#include "tests/mocks/io_event_receiver_mock.h"

#include <cstring>

class CanTest : public RODOS::testing::PeripheralTestFixture {
    void TearDown() override{
        RODOS::testing::PeripheralTestFixture::TearDown();
        RODOS::CanController::singletons[0] = nullptr;
        RODOS::CanController::singletons[1] = nullptr;
    }
};


namespace {
using ::testing::Sequence;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Return;
using ::testing::Mock;
using ::testing::Expectation;
using ::testing::ExpectationSet;

using namespace RODOS::testing;


TEST_F(CanTest, VerifyInit){
    using namespace RODOS::CAN_CNSTAT;
    using namespace RODOS::CAN_CIEN;
    using namespace RODOS::CAN_CTIM;
    using namespace RODOS::CAN_CGCR;

    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };
    constexpr uint32_t baudrate { 1'000'000 };
    RODOS::HAL_CAN testCan { idx };
    
    Expectation clockInitExpectation = EXPECT_CALL(RODOS::sysconfigPeripheralBitband->CAN_ENABLE[idx], write(1));

    ExpectationSet thingsToBeDoneBeforeEnabling;
    for(int i = 0; i<RODOS::CanController::NUM_RX_BUFFERS;i++){
        thingsToBeDoneBeforeEnabling += EXPECT_CALL(RODOS::cans[idx].buffers[i].CNSTAT, write(uint32_t(ST(uint32_t(RODOS::CAN_ST_VALUE::RX_READY)))))
            .After(clockInitExpectation);
    }

    ExpectationSet txBufferConfigExpectations;
    for(int i = RODOS::CanController::FIRST_TX_BUFFER_INDEX; i<RODOS::CanController::NUM_BUFFERS_TOTAL;i++){
        thingsToBeDoneBeforeEnabling += EXPECT_CALL(RODOS::cans[idx].buffers[i].CNSTAT, write(uint32_t(ST(uint32_t(RODOS::CAN_ST_VALUE::TX_NOT_ACTIVE)))))
            .After(clockInitExpectation);
    }

    thingsToBeDoneBeforeEnabling += EXPECT_CALL(RODOS::cans[idx].GMSKB, write(0xffff))
        .After(clockInitExpectation);
    thingsToBeDoneBeforeEnabling += EXPECT_CALL(RODOS::cans[idx].GMSKX, write(0xffff))
        .After(clockInitExpectation);
    
    {
        using namespace RODOS::CANTimingConfig;
        const uint32_t expectedPrescaler = baudrateNoPrescaler/baudrate;
        thingsToBeDoneBeforeEnabling += EXPECT_CALL(RODOS::cans[idx].CTIM, write(TSEG2(t_BS2-1) | TSEG1(t_BS1-1) | SJW(t_RJW-1) | PSC(expectedPrescaler-2)))
            .After(clockInitExpectation);
    }
    EXPECT_CALL(RODOS::cans[idx].CIEN, write(uint32_t(IEN(0x7fff))))
        .After(clockInitExpectation);
    EXPECT_CALL(RODOS::cans[idx].CICEN, write(uint32_t(IEN(0x7fff))))
        .After(clockInitExpectation);

    constexpr uint8_t can0IRQn { 72 };
    EXPECT_CALL(*globalNVICMock, enableIRQ(can0IRQn));
    EXPECT_CALL(RODOS::cans[idx].CGCR, write(CANEN(1) | BUFFLOCK(1) | DDIR(1)))
        .After(thingsToBeDoneBeforeEnabling);

    testCan.init(baudrate);
}

TEST_F(CanTest, TestWriteFailsWhenBufferFull){
    using namespace RODOS::CAN_CNSTAT;

    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };
    RODOS::HAL_CAN testCan { idx };

    EXPECT_CALL(RODOS::cans[idx].buffers[RODOS::CanController::FIRST_TX_BUFFER_INDEX].CNSTAT, read())
        .WillOnce(Return(ST(uint32_t(RODOS::CAN_ST_VALUE::TX_ONCE))));
    ASSERT_EQ(testCan.write(nullptr, 0, 0), -1);
}

TEST_F(CanTest, TestMultipleHalCansMapCorrectlyOntoBuffers){
    using namespace RODOS::CAN_CNSTAT;

    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };

    for(int8_t i = 0; i<RODOS::CanController::NUM_TX_BUFFERS; i++){
        RODOS::HAL_CAN testCan { idx };

        EXPECT_CALL(RODOS::cans[idx].buffers[RODOS::CanController::FIRST_TX_BUFFER_INDEX+i].CNSTAT, read())
            .WillOnce(Return(ST(uint32_t(RODOS::CAN_ST_VALUE::TX_ONCE))));
        ASSERT_EQ(testCan.write(nullptr, 0, 0), -1);
        Mock::VerifyAndClear(&RODOS::cans[idx].buffers[RODOS::CanController::FIRST_TX_BUFFER_INDEX+i].CNSTAT);
   }
}

TEST_F(CanTest, TestWrite){
    using namespace RODOS::CAN_CNSTAT;

    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };
    RODOS::HAL_CAN testCan { idx };

    constexpr uint64_t testData { 0xABCD'E987'6456'4654 };
    constexpr uint32_t testId { 0x1234'5678 };
    constexpr bool ide { true };
    constexpr bool rtr { false };
    constexpr RODOS::VA41620CanFrameMetadata metadata { testId, ide, rtr };

    RODOS::CANHardwareBuffer& buffer { RODOS::cans[idx].buffers[RODOS::CanController::FIRST_TX_BUFFER_INDEX] };
    for(size_t length = 0; length<=8; length++){
        uint16_t modifiedTestData[4] { };
        std::memcpy(modifiedTestData, &testData, length);
        EXPECT_CALL(buffer.CNSTAT, read())
            .WillOnce(Return(ST(uint32_t(RODOS::CAN_ST_VALUE::TX_NOT_ACTIVE))));
        EXPECT_CALL(buffer.ID0, write(metadata.getId0Value()));
        EXPECT_CALL(buffer.ID1, write(metadata.getId1Value()));
        EXPECT_CALL(buffer.DATA0, write(modifiedTestData[3]));
        EXPECT_CALL(buffer.DATA1, write(modifiedTestData[2]));
        EXPECT_CALL(buffer.DATA2, write(modifiedTestData[1]));
        EXPECT_CALL(buffer.DATA3, write(modifiedTestData[0]));
        EXPECT_CALL(buffer.CNSTAT, write(ST(uint32_t(RODOS::CAN_ST_VALUE::TX_ONCE)) | DLC(length)));

        ASSERT_EQ(testCan.write(reinterpret_cast<const uint8_t*>(&testData), length, testId), 0);
        Mock::VerifyAndClear(&buffer.CNSTAT);
        Mock::VerifyAndClear(&buffer.ID0);
        Mock::VerifyAndClear(&buffer.ID1);
        Mock::VerifyAndClear(&buffer.DATA0);
        Mock::VerifyAndClear(&buffer.DATA1);
        Mock::VerifyAndClear(&buffer.DATA2);
        Mock::VerifyAndClear(&buffer.DATA3);
    }
}


TEST_F(CanTest, TestTxInterruptHandling){
    using namespace RODOS::CAN_CSTPND;
    using namespace RODOS::CAN_CICLR;
    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };
    RODOS::HAL_CAN testCan { idx };

    IOEventReceiverMock eventReceiver;
    testCan.setIoEventReceiver(&eventReceiver);

    Sequence s;
    EXPECT_CALL(RODOS::cans[idx].CSTPND, read())
        .InSequence(s)
        .WillOnce(Return(uint32_t(IST(RODOS::CanController::FIRST_TX_BUFFER_INDEX+1))));
    EXPECT_CALL(RODOS::cans[idx].CICLR, write(uint32_t(ICLR(1<<RODOS::CanController::FIRST_TX_BUFFER_INDEX))))
        .InSequence(s);
    EXPECT_CALL(RODOS::cans[idx].CSTPND, read())
        .InSequence(s)
        .WillOnce(Return(0));

    EXPECT_CALL(eventReceiver, onWriteFinished());

    RODOS::CanController::callIrqHandler(idx);
}


TEST_F(CanTest, TestRead){
    using namespace RODOS::CAN_CSTPND;
    using namespace RODOS::CAN_CICLR;
    using namespace RODOS::CAN_CNSTAT;
    constexpr RODOS::CAN_IDX idx { RODOS::CAN_IDX0 };
    RODOS::HAL_CAN testCan1 { idx };
    RODOS::HAL_CAN testCan2 { idx };
    IOEventReceiverMock eventReceiver1;
    IOEventReceiverMock eventReceiver2;
    testCan1.setIoEventReceiver(&eventReceiver1);
    testCan2.setIoEventReceiver(&eventReceiver2);

    uint32_t incomingFilterMask { 0 };
    bool rtr { false };
    bool ide { true };
    uint32_t id1 { 1 };
    uint32_t id2 { 2 };
    RODOS::VA41620CanFrameMetadata metadata1 {id1, ide, rtr};
    RODOS::VA41620CanFrameMetadata metadata2 {id2, ide, rtr};
    testCan1.addIncomingFilter(id1, incomingFilterMask, ide, rtr);
    testCan2.addIncomingFilter(id2, incomingFilterMask, ide, rtr);

    uint64_t testData1 { 0x7654'6546'6546'5646 };
    uint64_t testData2 { 0x4342'6764'8689'0876 };

    Sequence s;

    RODOS::CANHardwareBuffer& buffer0 { RODOS::cans[0].buffers[0] };
    RODOS::CANHardwareBuffer& buffer2 { RODOS::cans[0].buffers[2] };

    EXPECT_CALL(RODOS::cans[idx].CSTPND, read())
        .InSequence(s)
        .WillOnce(Return(uint32_t(IST(1))));
    EXPECT_CALL(RODOS::cans[idx].CICLR, write(uint32_t(ICLR(1<<0))))
        .InSequence(s);
    EXPECT_CALL(RODOS::cans[idx].CSTPND, read())
        .InSequence(s)
        .WillOnce(Return(uint32_t(IST(3))));
    EXPECT_CALL(RODOS::cans[idx].CICLR, write(uint32_t(ICLR(1<<2))))
        .InSequence(s);
    EXPECT_CALL(RODOS::cans[idx].CSTPND, read())
        .InSequence(s)
        .WillOnce(Return(0));

    EXPECT_CALL(buffer0.CNSTAT, read())
        .WillOnce(Return(DLC(sizeof(testData1)) | ST(uint32_t(RODOS::CAN_ST_VALUE::RX_FULL))));
    EXPECT_CALL(buffer2.CNSTAT, read())
        .WillOnce(Return(DLC(sizeof(testData2)) | ST(uint32_t(RODOS::CAN_ST_VALUE::RX_FULL))));
    EXPECT_CALL(buffer0.ID0, read())
        .WillOnce(Return(metadata1.getId0Value()));
    EXPECT_CALL(buffer2.ID0, read())
        .WillOnce(Return(metadata2.getId0Value()));
    EXPECT_CALL(buffer0.ID1, read())
        .WillOnce(Return(metadata1.getId1Value()));
    EXPECT_CALL(buffer2.ID1, read())
        .WillOnce(Return(metadata2.getId1Value()));
    EXPECT_CALL(buffer0.DATA3, read())
        .WillOnce(Return(testData1>>0));
    EXPECT_CALL(buffer2.DATA3, read())
        .WillOnce(Return(testData2>>0));
    EXPECT_CALL(buffer0.DATA2, read())
        .WillOnce(Return(testData1>>16));
    EXPECT_CALL(buffer2.DATA2, read())
        .WillOnce(Return(testData2>>16));
    EXPECT_CALL(buffer0.DATA1, read())
        .WillOnce(Return(testData1>>32));
    EXPECT_CALL(buffer2.DATA1, read())
        .WillOnce(Return(testData2>>32));
    EXPECT_CALL(buffer0.DATA0, read())
        .WillOnce(Return(testData1>>48));
    EXPECT_CALL(buffer2.DATA0, read())
        .WillOnce(Return(testData2>>48));



    EXPECT_CALL(buffer0.CNSTAT, write(uint32_t(ST(uint32_t(RODOS::CAN_ST_VALUE::RX_READY)))));
    EXPECT_CALL(buffer2.CNSTAT, write(uint32_t(ST(uint32_t(RODOS::CAN_ST_VALUE::RX_READY)))));

    EXPECT_CALL(eventReceiver1, onDataReady());
    EXPECT_CALL(eventReceiver2, onDataReady());

    RODOS::CanController::callIrqHandler(idx);

    uint64_t receiveBuffer1;
    uint32_t receivedId1;
    bool receivedIde1;
    bool receivedRtr1;
    uint64_t receiveBuffer2;
    uint32_t receivedId2;
    bool receivedIde2;
    bool receivedRtr2;

    ASSERT_EQ(testCan1.read(reinterpret_cast<uint8_t*>(&receiveBuffer1), &receivedId1, &receivedIde1, &receivedRtr1), sizeof(testData1));
    ASSERT_EQ(receiveBuffer1, testData1);
    ASSERT_EQ(receivedId1, id1);
    ASSERT_EQ(receivedIde1, ide);
    ASSERT_EQ(receivedRtr1, rtr);

    ASSERT_EQ(testCan2.read(reinterpret_cast<uint8_t*>(&receiveBuffer2), &receivedId2, &receivedIde2, &receivedRtr2), sizeof(testData2));
    ASSERT_EQ(receiveBuffer2, testData2);
    ASSERT_EQ(receivedId2, id2);
    ASSERT_EQ(receivedIde2, ide);
    ASSERT_EQ(receivedRtr2, rtr);

}

}
