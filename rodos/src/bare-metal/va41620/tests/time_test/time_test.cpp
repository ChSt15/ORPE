#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "hw_specific.h"
#include "timemodel.h"
#include "tests/peripheral_test_fixture.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "clkgen_config.h"

#include <algorithm>
#include <array>
#include <random>
#include <limits>

namespace {
using ::testing::InSequence;
using ::testing::Return;
using ::testing::Mock;

using namespace RODOS::testing;

class TimeTest : public PeripheralTestFixture {};

constexpr uint32_t expectedReloadValue { 0xFFFF'FFFF };

TEST_F(TimeTest, VerifyTimeInit){
    using namespace RODOS::TIM_CTRL;
    using namespace RODOS::TIM_CSD_CTRL;
    constexpr uint32_t expectedReloadValue { 0xFFFF'FFFF };

    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->TIM_ENABLE[0], write(uint32_t(1)));
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->TIM_ENABLE[1], write(uint32_t(1)));

    EXPECT_CALL(RODOS::tim0to15[0].RST_VALUE, write(expectedReloadValue));
    EXPECT_CALL(RODOS::tim0to15[0].CNT_VALUE, write(expectedReloadValue));

    EXPECT_CALL(RODOS::tim0to15[1].RST_VALUE, write(expectedReloadValue));
    EXPECT_CALL(RODOS::tim0to15[1].CNT_VALUE, write(expectedReloadValue));
    EXPECT_CALL(RODOS::tim0to15[1].CSD_CTRL, write(CSDEN0(1).shiftedValue));
    EXPECT_CALL(RODOS::tim0to15[1].CASCADE[0], write(static_cast<uint32_t>(RODOS::CascadeSource::fromTimer(0))));

    EXPECT_CALL(RODOS::tim0to15[0].CTRL, write(ENABLE(1).shiftedValue));
    EXPECT_CALL(RODOS::tim0to15[1].CTRL, write(ENABLE(1).shiftedValue));

    RODOS::hwInitTime();
}

TEST_F(TimeTest, VerifyTimeConversion){
    constexpr int64_t timePerClock { 1*RODOS::SECONDS/RODOS::globalClockSetup.getAPB1Frequency() };

    std::random_device rd;
    std::mt19937 mt { rd() };
    std::uniform_int_distribution<int64_t> timeDistribution {0, std::numeric_limits<int64_t>::max()/timePerClock};
    constexpr size_t numValuesToTest { 100'000 };
    std::array<int64_t, numValuesToTest> timeValuesToTest;
    std::generate(timeValuesToTest.begin(), timeValuesToTest.end(), [&](){return timeDistribution(mt)*timePerClock;});

    for(int64_t time : timeValuesToTest){
        int64_t cycles { time/timePerClock };
        uint32_t lowerCounter { expectedReloadValue-static_cast<uint32_t>(cycles&0xFFFF'FFFF) };
        uint32_t upperCounter { expectedReloadValue-static_cast<uint32_t>(cycles>>32) };
        EXPECT_CALL(RODOS::tim0to15[0].CNT_VALUE, read())
            .WillOnce(Return(lowerCounter));
        EXPECT_CALL(RODOS::tim0to15[1].CNT_VALUE, read())
            .Times(2)
            .WillRepeatedly(Return(upperCounter));
        ASSERT_EQ(RODOS::hwGetNanoseconds(), time);
        Mock::VerifyAndClear(&RODOS::tim0to15[0].CNT_VALUE);
        Mock::VerifyAndClear(&RODOS::tim0to15[1].CNT_VALUE);
    }
}

TEST_F(TimeTest, VerifyHandlingOfLowCounterChangeDuringUpperCounterRead){
    InSequence s;
    constexpr uint32_t initialUpperCounter { 0xffff'ffff };
    constexpr uint32_t finalUpperCounter { 0xffff'fff0 };
    constexpr int32_t timePerClock { 1*RODOS::SECONDS/RODOS::globalClockSetup.getAPB1Frequency() };
    for(uint32_t i = initialUpperCounter; i>=finalUpperCounter; i--){
        EXPECT_CALL(RODOS::tim0to15[1].CNT_VALUE, read())
            .WillOnce(Return(i));
        EXPECT_CALL(RODOS::tim0to15[0].CNT_VALUE, read())
            .WillOnce(Return(0xffff'ffff));
    }
    EXPECT_CALL(RODOS::tim0to15[1].CNT_VALUE, read())
        .WillOnce(Return(finalUpperCounter));

    ASSERT_EQ(RODOS::hwGetNanoseconds(), (static_cast<int64_t>(0xffff'ffff - finalUpperCounter)<<32) * timePerClock);
}


}
