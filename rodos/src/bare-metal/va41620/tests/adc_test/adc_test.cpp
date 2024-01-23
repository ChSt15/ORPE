#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "hal/hal_adc.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "tests/peripheral_test_fixture.h"

#include <cstring>

namespace {
using ::testing::Sequence;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Return;
using ::testing::Mock;

using namespace RODOS::testing;

class ADCTest : public PeripheralTestFixture {};


TEST_F(ADCTest, VerifyInit){
    constexpr RODOS::ADC_IDX idx { RODOS::ADC_IDX0 };
    constexpr RODOS::ADC_CHANNEL channel { RODOS::ADC_CH_000 };
    RODOS::HAL_ADC testADC{idx};

    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->ADC_ENABLE, write(1));

    testADC.init(channel);
}

void testAdcRead(RODOS::ADC_CHANNEL channel, uint16_t voltageValue){
    using namespace RODOS::ADC_CTRL;
    using namespace RODOS::ADC_STATUS;
    RODOS::HAL_ADC testADC{RODOS::ADC_IDX0};

    InSequence s;
    EXPECT_CALL(RODOS::adc->FIFO_CLR, write(1));
    EXPECT_CALL(RODOS::adc->CTRL, write(CHAN_EN(uint16_t(1<<channel)) | MANUAL_TRIG(1)));
    EXPECT_CALL(RODOS::adc->STATUS, read())
        .Times(10)
        .WillRepeatedly(Return(uint32_t(ADC_BUSY(1))));
    EXPECT_CALL(RODOS::adc->STATUS, read())
        .WillOnce(Return(0));
    EXPECT_CALL(RODOS::adc->FIFO_DATA, read())
        .WillOnce(Return(voltageValue));

    ASSERT_EQ(testADC.read(static_cast<RODOS::ADC_CHANNEL>(channel)), voltageValue);
    Mock::VerifyAndClear(&RODOS::adc->FIFO_CLR);
    Mock::VerifyAndClear(&RODOS::adc->CTRL);
    Mock::VerifyAndClear(&RODOS::adc->STATUS);
    Mock::VerifyAndClear(&RODOS::adc->FIFO_DATA);
}


TEST_F(ADCTest, VerifyReadForAllChannels){
    for(int channel = 0; channel<16; channel++){
        testAdcRead(static_cast<RODOS::ADC_CHANNEL>(channel), 0);
    }
}


TEST_F(ADCTest, VerifyReadForAllVoltages){
    for(int voltageValue = 0; voltageValue<(1<<12); voltageValue++){
        testAdcRead(RODOS::ADC_CH_000, voltageValue);
    }
}

}
