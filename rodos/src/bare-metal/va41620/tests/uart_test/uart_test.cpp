#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "hal/hal_uart.h"
#include "hw_hal/hw_hal_uart.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "peripheral_ctrl/pin_config/pin_config_mock.h"
#include "tests/peripheral_test_fixture.h"
#include "cortex_m/nvic_mock.h"
#include "tests/mocks/io_event_receiver_mock.h"

#include <cstring>

namespace {
using ::testing::Sequence;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Return;
using ::testing::Mock;

using namespace RODOS::testing;

class UARTTest : public PeripheralTestFixture {};


void assertCLKSCALERegister(uint32_t attemptedBaudrate, uint32_t clkscaleValue){
    constexpr int8_t fixedPointDigits { 6 };
    double clockDivider { double(clkscaleValue)/(1<<fixedPointDigits) };
    double achievedSampleRate { RODOS::globalClockSetup.getAPB2Frequency()/clockDivider };
    constexpr double overSampling { 16 };
    double achievedBaudRate { achievedSampleRate/overSampling };
    constexpr double allowedRelatileError { 0.01 };
    ASSERT_NEAR(achievedBaudRate/attemptedBaudrate, 1, allowedRelatileError);
}

TEST_F(UARTTest, VerifyInit){
    using namespace RODOS::UART_ENABLE;
    using namespace RODOS::UART_IRQ_ENB;
    constexpr RODOS::UART_IDX idx { RODOS::UART_IDX0 };
    constexpr RODOS::GPIO_PIN txPin { RODOS::GPIO_096 };
    constexpr RODOS::GPIO_PIN rxPin { RODOS::GPIO_097};
    constexpr uint32_t attemptedBaudrate { 115200 };
    RODOS::HAL_UART testUART{idx, txPin, rxPin};

    Sequence s;
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->UART_RESET[idx], write(0))
        .InSequence(s);
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->UART_RESET[idx], write(1))
        .InSequence(s);
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->UART_ENABLE[idx], write(1))
        .InSequence(s);
    EXPECT_CALL(RODOS::uart0->CLKSCALE, write(_))
        .InSequence(s)
        .WillOnce([](uint32_t x){assertCLKSCALERegister(attemptedBaudrate, x);});
    EXPECT_CALL(RODOS::uart0->ENABLE, write(RXENABLE(1) | TXENABLE(1)))
        .InSequence(s);
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, write(IRQ_RX(1) | IRQ_RX_TO(1)))
        .InSequence(s);

    constexpr uint8_t rxIRQn { 24 };
    constexpr uint8_t txIRQn { 25 };
    EXPECT_CALL(*globalNVICMock, enableIRQ(rxIRQn));
    EXPECT_CALL(*globalNVICMock, enableIRQ(txIRQn));


    constexpr auto expectedFunction { RODOS::pin_config::PIN_FUNCTION::FUN1 };
    constexpr auto expectedPull { RODOS::pin_config::PULL::OFF };
    constexpr auto expectedOutputType { RODOS::pin_config::OUTPUT_TYPE::PUSHPULL };
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(txPin), expectedFunction, expectedOutputType, expectedPull));
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(rxPin), expectedFunction, expectedOutputType, expectedPull));

    testUART.init(attemptedBaudrate);
}

TEST_F(UARTTest, VerifyReadProcess){
    using namespace RODOS::UART_RXSTATUS;
    constexpr RODOS::UART_IDX idx { RODOS::UART_IDX0 };
    constexpr RODOS::GPIO_PIN txPin { RODOS::GPIO_096 };
    constexpr RODOS::GPIO_PIN rxPin { RODOS::GPIO_097};
    RODOS::HAL_UART testUART{idx, txPin, rxPin};
    IOEventReceiverMock eventReceiver;
    testUART.setIoEventReceiver(&eventReceiver);


    const char testedInput[] { "This is a char array." };
    size_t currentInputIndex { 0 };
    EXPECT_CALL(eventReceiver, onDataReady());
    EXPECT_CALL(RODOS::uart0->DATA, read())
        .Times(sizeof(testedInput))
        .WillRepeatedly([&]{
            return testedInput[currentInputIndex++];
        });
    EXPECT_CALL(RODOS::uart0->RXSTATUS, read())
        .WillRepeatedly([&]{
            return static_cast<uint32_t>(RDAVL((currentInputIndex<sizeof(testedInput))?1:0));
        });

    RODOS::HW_HAL_UART::contextArray[idx]->rxIrqHandler();

    char readBuffer[sizeof(testedInput)] { };
    ASSERT_EQ(testUART.read(readBuffer, sizeof(readBuffer)), sizeof(readBuffer));
    ASSERT_EQ(std::memcmp(testedInput, readBuffer, sizeof(testedInput)), 0);
}

TEST_F(UARTTest, VerifyGetCharNoWait){
    using namespace RODOS::UART_RXSTATUS;
    constexpr RODOS::UART_IDX idx { RODOS::UART_IDX0 };
    constexpr RODOS::GPIO_PIN txPin { RODOS::GPIO_096 };
    constexpr RODOS::GPIO_PIN rxPin { RODOS::GPIO_097};
    RODOS::HAL_UART testUART{idx, txPin, rxPin};


    const char testedInput[] { "This is a char array." };
    size_t currentInputIndex { 0 };
    EXPECT_CALL(RODOS::uart0->DATA, read())
        .Times(sizeof(testedInput))
        .WillRepeatedly([&]{
            return testedInput[currentInputIndex++];
        });
    EXPECT_CALL(RODOS::uart0->RXSTATUS, read())
        .WillRepeatedly([&]{
            return static_cast<uint32_t>(RDAVL((currentInputIndex<sizeof(testedInput))?1:0));
        });

    RODOS::HW_HAL_UART::contextArray[idx]->rxIrqHandler();

    for(size_t i = 0; i<sizeof(testedInput); i++){
        ASSERT_EQ(testUART.getcharNoWait(), testedInput[i]);
    }
    ASSERT_EQ(testUART.getcharNoWait(), -1);
}

TEST_F(UARTTest, VerifyWriteProcess){
    using namespace RODOS::UART_TXSTATUS;
    using namespace RODOS::UART_IRQ_ENB;
    constexpr RODOS::UART_IDX idx { RODOS::UART_IDX0 };
    constexpr RODOS::GPIO_PIN txPin { RODOS::GPIO_096 };
    constexpr RODOS::GPIO_PIN rxPin { RODOS::GPIO_097 };
    RODOS::HAL_UART testUART{idx, txPin, rxPin};
    IOEventReceiverMock eventReceiver;
    testUART.setIoEventReceiver(&eventReceiver);

    const char testedOutput[] { "Generic test message" };

    Sequence s;
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, read)
        .InSequence(s)
        .WillOnce(Return(IRQ_TX(0) | IRQ_TX_EMPTY(1)));
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, write(IRQ_TX(1) | IRQ_TX_EMPTY(0)))
        .InSequence(s);
    testUART.write(testedOutput, sizeof(testedOutput));

    ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::uart0->IRQ_ENB));


    size_t currentOutputIndex { 0 };
    size_t freeSpaceInHardwareFifo { 8 };

    EXPECT_CALL(RODOS::uart0->DATA, write(_))
        .Times(sizeof(testedOutput))
        .InSequence(s)
        .WillRepeatedly([&](uint32_t x){
            ASSERT_GT(freeSpaceInHardwareFifo, 0);
            freeSpaceInHardwareFifo--;
            ASSERT_EQ(x, testedOutput[currentOutputIndex++]);
        });
    EXPECT_CALL(RODOS::uart0->TXSTATUS, read())
        .WillRepeatedly([&]{
            return WRRDY((freeSpaceInHardwareFifo>0)?1:0) | WRBUSY(1);
        });
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, read)
        .WillRepeatedly([]{
            return static_cast<uint32_t>(IRQ_TX(1));
        });
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, write(IRQ_TX(0) | IRQ_TX_EMPTY(1)))
        .InSequence(s);

    while(currentOutputIndex < sizeof(testedOutput)){
        RODOS::HW_HAL_UART::contextArray[idx]->txIrqHandler();
        freeSpaceInHardwareFifo+=2;
    }

    ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::uart0->TXSTATUS));
    ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::uart0->DATA));
    ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::uart0->IRQ_ENB));


    EXPECT_CALL(RODOS::uart0->IRQ_ENB, read)
        .InSequence(s)
        .WillRepeatedly([]{
            return static_cast<uint32_t>(IRQ_TX_EMPTY(0));
        });
    EXPECT_CALL(RODOS::uart0->IRQ_ENB, write(IRQ_TX(0) | IRQ_TX_EMPTY(0)))
        .InSequence(s);
    EXPECT_CALL(RODOS::uart0->TXSTATUS, read())
        .WillRepeatedly(Return(WRRDY(1) | WRBUSY(0)));
    EXPECT_CALL(eventReceiver, onWriteFinished);
    RODOS::HW_HAL_UART::contextArray[idx]->txIrqHandler();
}

TEST_F(UARTTest, VerifyPutCharFromInterrupt){
    using namespace RODOS::UART_TXSTATUS;
    constexpr RODOS::UART_IDX idx { RODOS::UART_IDX0 };
    constexpr RODOS::GPIO_PIN txPin { RODOS::GPIO_096 };
    constexpr RODOS::GPIO_PIN rxPin { RODOS::GPIO_097 };
    RODOS::HAL_UART testUART{idx, txPin, rxPin};

    char testChar = '6';
    InSequence s;
    EXPECT_CALL(RODOS::uart0->TXSTATUS, read())
        .Times(30)
        .WillRepeatedly(Return(static_cast<uint32_t>(WRRDY(0))));
    EXPECT_CALL(RODOS::uart0->TXSTATUS, read())
        .WillOnce(Return(static_cast<uint32_t>(WRRDY(1))));
    EXPECT_CALL(RODOS::uart0->DATA, write(testChar));

    RODOS::HW_HAL_UART::putcharFromInterrupt(idx, testChar);
}

}
