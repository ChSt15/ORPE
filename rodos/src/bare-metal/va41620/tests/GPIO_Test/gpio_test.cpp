#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "hal/hal_gpio.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "tests/peripheral_test_fixture.h"

namespace {
using ::testing::Sequence;
using ::testing::_;
using ::testing::Return;

using namespace RODOS::testing;

class GPIOTest : public PeripheralTestFixture {};

TEST_F(GPIOTest, VerifyOutputInit){
    RODOS::HAL_GPIO testGPIO_OutputInit{RODOS::GPIO_000};

    constexpr bool isOutput = true;
    constexpr uint8_t numOfPins = 2;
    constexpr uint8_t bitMask = 0b11;
    constexpr uint32_t initVal = 0b11;

    constexpr uint16_t originalPinOutputs = 0xedc5;
    constexpr uint16_t originalDirections = 0xa8c5;
    uint16_t pinOutputs = originalPinOutputs;
    uint16_t directionState = originalDirections;

    Sequence s1, s2;
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->GPIO_PORT_ENABLE[0], write(1))
        .InSequence(s1, s2);
    EXPECT_CALL(RODOS::gpioStructs[0].SETOUT, write(_))
        .InSequence(s1)
        .WillOnce([&](uint32_t x){ pinOutputs |= static_cast<uint16_t>(x); });
    EXPECT_CALL(RODOS::gpioStructs[0].CLROUT, write(_))
        .InSequence(s2)
        .WillOnce([&](uint32_t x){ pinOutputs &= static_cast<uint16_t>(~x); });
    EXPECT_CALL(RODOS::gpioStructs[0].DIR, read())
        .InSequence(s1, s2)
        .WillOnce([&]()->uint32_t {return directionState;});
    EXPECT_CALL(RODOS::gpioStructs[0].DIR, write(_))
        .InSequence(s1, s2)
        .WillOnce([&](uint32_t x){directionState = static_cast<uint16_t>(x);});


    testGPIO_OutputInit.init(isOutput, numOfPins, initVal);
    ASSERT_EQ((directionState&bitMask), bitMask)
        << "Pins not set to output";
    ASSERT_EQ((directionState&~bitMask), (originalDirections&~bitMask))
        << "Direction of other pins changed";
    ASSERT_EQ((pinOutputs&bitMask), initVal)
        << "Output incorrectly set";
    ASSERT_EQ((pinOutputs&~bitMask), (originalPinOutputs&~bitMask))
        << "Output of other pins changed";
}


TEST_F(GPIOTest, VerifyInputInit){
    RODOS::HAL_GPIO testGPIO_InputInit{RODOS::GPIO_020};

    constexpr bool isOutput = false;
    constexpr uint8_t numOfPins = 2;

    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->GPIO_PORT_ENABLE[1], write(1));
    testGPIO_InputInit.init(isOutput, numOfPins);
}

TEST_F(GPIOTest, PinsMapCorrectlyOntoRegisters){
    constexpr uint8_t totalNumPins = 104;
    constexpr bool isOutput = true;
    constexpr uint8_t numOfPins = 1;
    constexpr uint8_t initValue = 1;
    Sequence s;
    for(uint8_t i = 0; i<totalNumPins; i++){
        const uint8_t port = i/16;
        const uint16_t bitMask = 1<<(i%16);

        EXPECT_CALL(RODOS::sysconfigPeripheralBitband->GPIO_PORT_ENABLE[port], write(1))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].SETOUT, write(bitMask))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].CLROUT, write(0))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].DIR, read())
            .InSequence(s)
            .WillOnce(Return(0));
        EXPECT_CALL(RODOS::gpioStructs[port].DIR, write(bitMask))
            .InSequence(s);
    }

    for(uint8_t i = 0; i<totalNumPins; i++){
        RODOS::HAL_GPIO testGPIO{static_cast<RODOS::GPIO_PIN>(i)};
        testGPIO.init(isOutput, numOfPins, initValue);
    }
}

TEST_F(GPIOTest, InitFailsWithInvalidPinCount){
    constexpr bool isOutput = true;
    constexpr uint8_t numOfPins = 10;

    RODOS::HAL_GPIO testGPIO{RODOS::GPIO_047};
    ASSERT_EQ(testGPIO.init(isOutput, numOfPins), -1);
}

/**
 * needed because the MMIO is a StrictMock and unexpected calls lead to errors
 */
void setupInitExpectCalls(RODOS::GPIO_PIN pin, bool isOutput, Sequence& s){
    const uint8_t port = static_cast<uint8_t>(pin)/16;
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->GPIO_PORT_ENABLE[port], write(1))
        .InSequence(s);
    if(isOutput){
        EXPECT_CALL(RODOS::gpioStructs[port].SETOUT, write(_))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].CLROUT, write(_))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].DIR, read())
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[port].DIR, write(_))
            .InSequence(s);
    }
}

TEST_F(GPIOTest, VerifyWriteFunctionality){
    constexpr RODOS::GPIO_PIN pin = RODOS::GPIO_008;
    RODOS::HAL_GPIO testGPIO{pin};
    constexpr bool isOutput = true;
    constexpr uint8_t numOfPins = 8;
    constexpr uint8_t pinInPort = 8;
    constexpr uint16_t pinMask = 0xff00;
    Sequence s;
    setupInitExpectCalls(pin, isOutput, s);
    for(uint16_t i = 0; i<256; i++){
        EXPECT_CALL(RODOS::gpioStructs[0].SETOUT, write(i<<pinInPort))
            .InSequence(s);
        EXPECT_CALL(RODOS::gpioStructs[0].CLROUT, write((~(i<<pinInPort))&pinMask))
            .InSequence(s);
    }

    testGPIO.init(isOutput, numOfPins);
    for(uint16_t i = 0; i<256; i++){
        testGPIO.setPins(i);
    }
}


TEST_F(GPIOTest, VerifyReadFunctionality){
    constexpr RODOS::GPIO_PIN pin = RODOS::GPIO_008;
    RODOS::HAL_GPIO testGPIO{pin};
    constexpr bool isOutput = false;
    constexpr uint8_t numOfPins = 8;
    constexpr uint8_t pinInPort = 8;
    constexpr uint16_t pinMask = 0xff00;
    Sequence s;
    setupInitExpectCalls(pin, isOutput, s);
    for(uint16_t i = 0; i<255; i++){
        EXPECT_CALL(RODOS::gpioStructs[0].DATAIN, read())
            .InSequence(s)
            .WillOnce(Return( (i<<pinInPort)|~pinMask ) );
    }

    testGPIO.init(isOutput, numOfPins);
    for(uint16_t i = 0; i<255; i++){
        ASSERT_EQ(testGPIO.readPins(), i);
    }
}

}
