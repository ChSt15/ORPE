#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "peripheral_ctrl/clock_management/clock_initializers.h"
#include "peripheral_ctrl/clock_management/hbo_busywait_mock.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "tests/peripheral_test_fixture.h"

namespace {
using ::testing::Sequence;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Return;
using ::testing::Ge;

using namespace RODOS::testing;

class CLKGENTest : public PeripheralTestFixture {
    void SetUp() override{
        PeripheralTestFixture::SetUp();
        RODOS::globalHboBusywaitMock = new RODOS::HboBusywaitMock;
    }

    void TearDown() override{
        PeripheralTestFixture::TearDown();
        delete RODOS::globalHboBusywaitMock;
    }
};

template<RODOS::XTALSetup setup>
void TestXTALSetup(){
    using namespace RODOS::CLKGEN_CTRL1;
    RODOS::UInt32BitField fakeCTRL1Register{0x0000'0018};
    EXPECT_CALL(RODOS::clkgen->CTRL1, read())
        .WillOnce([&]{return fakeCTRL1Register.read();});
    EXPECT_CALL(RODOS::clkgen->CTRL1, write(_))
        .WillOnce([&](uint32_t value){fakeCTRL1Register.write(value);});
    if constexpr (setup.getMode() == RODOS::XTAL_MODE::XTAL){
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(15'000)));
    } else if constexpr (setup.getMode() == RODOS::XTAL_MODE::EXT_CLK){
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(5'000)));
    }
    RODOS::XTALInitializer::init(setup);

    ASSERT_EQ(fakeCTRL1Register.read<XTAL_EN>(), setup.getMode() == RODOS::XTAL_MODE::XTAL);
    ASSERT_EQ(fakeCTRL1Register.read<XTAL_N_EN>(), setup.getMode() == RODOS::XTAL_MODE::EXT_CLK);
    
}

TEST_F(CLKGENTest, XTALInitTurnsXTALOffCorrectly){
    constexpr RODOS::XTALSetup testedSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::OFF, 0>() };
    TestXTALSetup<testedSetup>();
}

TEST_F(CLKGENTest, XTALInitConfiguresXTALModeCorrectly){
    constexpr RODOS::XTALSetup testedSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::XTAL, 5'000'000>() };
    TestXTALSetup<testedSetup>();
}

TEST_F(CLKGENTest, XTALInitConfiguresEXT_CLKModeCorrectly){
    constexpr RODOS::XTALSetup testedSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::EXT_CLK, 0>() };
    TestXTALSetup<testedSetup>();
}


template<RODOS::PLLSetup setup>
void TestPllSetup(){
    using namespace RODOS::CLKGEN_CTRL0;
    Sequence s1, s2;
    if constexpr(setup.getSource() != RODOS::PLL_SOURCE::NONE){
        RODOS::UInt32BitField fakeCTRL0Register{0x0000'0030};
        EXPECT_CALL(RODOS::clkgen->CTRL0, read())
            .InSequence(s1)
            .WillRepeatedly([&]{return fakeCTRL0Register.read();});
        EXPECT_CALL(RODOS::clkgen->CTRL0, write(_))
            .InSequence(s2)
            .WillRepeatedly([&](uint32_t value){fakeCTRL0Register.write(value);});
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(5'000)))
            .InSequence(s1, s2)
            .WillOnce([&]{
                ASSERT_EQ(fakeCTRL0Register.read<REF_CL_SEL>(), static_cast<uint32_t>(setup.getSource()));
                ASSERT_EQ(fakeCTRL0Register.read<PLL_PWDN>(), 0);
            });


        EXPECT_CALL(RODOS::clkgen->CTRL0, read())
            .InSequence(s1)
            .WillRepeatedly([&]{return fakeCTRL0Register.read();});
        EXPECT_CALL(RODOS::clkgen->CTRL0, write(_))
            .InSequence(s2)
            .WillRepeatedly([&](uint32_t value){fakeCTRL0Register.write(value);});
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(5'000)))
            .InSequence(s1, s2)
            .WillOnce([&]{
                ASSERT_EQ(fakeCTRL0Register.read<PLL_PWDN>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_RESET>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_INTFB>(), 1);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BYPASS>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_TEST>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BWADJ>(), setup.getBWADJRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKOD>(), setup.getCLKODRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKF>(), setup.getCLKFRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKR>(), setup.getCLKRRegValue());
            });


        EXPECT_CALL(RODOS::clkgen->CTRL0, read())
            .InSequence(s1)
            .WillRepeatedly([&]{return fakeCTRL0Register.read();});
        EXPECT_CALL(RODOS::clkgen->CTRL0, write(_))
            .InSequence(s2)
            .WillRepeatedly([&](uint32_t value){fakeCTRL0Register.write(value);});
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(5'000)))
            .InSequence(s1, s2)
            .WillOnce([&]{
                ASSERT_EQ(fakeCTRL0Register.read<PLL_PWDN>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_RESET>(), 1);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_INTFB>(), 1);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BYPASS>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_TEST>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BWADJ>(), setup.getBWADJRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKOD>(), setup.getCLKODRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKF>(), setup.getCLKFRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKR>(), setup.getCLKRRegValue());
            });

        EXPECT_CALL(RODOS::clkgen->CTRL0, read())
            .InSequence(s1)
            .WillRepeatedly([&]{return fakeCTRL0Register.read();});
        EXPECT_CALL(RODOS::clkgen->CTRL0, write(_))
            .InSequence(s2)
            .WillRepeatedly([&](uint32_t value){fakeCTRL0Register.write(value);});
        EXPECT_CALL(*RODOS::globalHboBusywaitMock, waitMicroseconds(Ge(5'000)))
            .InSequence(s1, s2)
            .WillOnce([&]{
                ASSERT_EQ(fakeCTRL0Register.read<PLL_PWDN>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_RESET>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_INTFB>(), 1);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BYPASS>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_TEST>(), 0);
                ASSERT_EQ(fakeCTRL0Register.read<PLL_BWADJ>(), setup.getBWADJRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKOD>(), setup.getCLKODRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKF>(), setup.getCLKFRegValue());
                ASSERT_EQ(fakeCTRL0Register.read<PLL_CLKR>(), setup.getCLKRRegValue());
            });
    }
    RODOS::PLLInitializer::init(setup);
}

TEST_F(CLKGENTest, PLLSetupDoesNothingIfDisabled){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::OFF, 0>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<false, testedXTALSetup, 0, false>() };
    TestPllSetup<testedPLLSetup>();
}

TEST_F(CLKGENTest, PLLWorksWithXTAL){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::XTAL, 5'000'000>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<true, testedXTALSetup, 100'000'000, true>() };
    TestPllSetup<testedPLLSetup>();
}

TEST_F(CLKGENTest, PLLWorksWithEXTCLK){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::EXT_CLK, 25'000'000>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<true, testedXTALSetup, 30'000'000, true>() };
    TestPllSetup<testedPLLSetup>();
}

template<RODOS::SYSCLKSetup setup>
void testSysclkSetup(){
    using namespace RODOS::CLKGEN_CTRL0;
    RODOS::UInt32BitField fakeCTRL0Register { 0x0000'0030 };
    InSequence s;
    EXPECT_CALL(RODOS::clkgen->CTRL0, read())
        .WillOnce([&]{return fakeCTRL0Register.read();});
    EXPECT_CALL(RODOS::clkgen->CTRL0, write(_))
        .WillOnce([&](uint32_t val){fakeCTRL0Register.write(val);});

    RODOS::SYSCLKInitializer::init(setup);
    ASSERT_EQ(fakeCTRL0Register.read<CLKSEL_SYS>(), static_cast<uint32_t>(setup.getSource()));
}

TEST_F(CLKGENTest, SYSCLKInitWorksWithHBO){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::OFF, 0>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<false, testedXTALSetup, 0, true>() };
    constexpr RODOS::SYSCLKSetup testedSYSCLKSetup { RODOS::SYSCLKSetup::generateSetup<RODOS::SYSCLK_SOURCE::HBO, testedXTALSetup, testedPLLSetup>() };
    testSysclkSetup<testedSYSCLKSetup>();
}

TEST_F(CLKGENTest, SYSCLKInitWorksWithXTAL){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::XTAL, 5'000'000>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<false, testedXTALSetup, 0, true>() };
    constexpr RODOS::SYSCLKSetup testedSYSCLKSetup { RODOS::SYSCLKSetup::generateSetup<RODOS::SYSCLK_SOURCE::XTAL, testedXTALSetup, testedPLLSetup>() };
    testSysclkSetup<testedSYSCLKSetup>();
}

TEST_F(CLKGENTest, SYSCLKInitWorksWithEXTCLK){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::EXT_CLK, 5'000'000>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<false, testedXTALSetup, 0, true>() };
    constexpr RODOS::SYSCLKSetup testedSYSCLKSetup { RODOS::SYSCLKSetup::generateSetup<RODOS::SYSCLK_SOURCE::EXT_CLK, testedXTALSetup, testedPLLSetup>() };
    testSysclkSetup<testedSYSCLKSetup>();
}

TEST_F(CLKGENTest, SYSCLKInitWorksWithPLL){
    constexpr RODOS::XTALSetup testedXTALSetup { RODOS::XTALSetup::generateSetup<RODOS::XTAL_MODE::EXT_CLK, 5'000'000>() };
    constexpr RODOS::PLLSetup testedPLLSetup { RODOS::PLLSetup::generateSetup<true, testedXTALSetup, 100'000'000, true>() };
    constexpr RODOS::SYSCLKSetup testedSYSCLKSetup { RODOS::SYSCLKSetup::generateSetup<RODOS::SYSCLK_SOURCE::PLL, testedXTALSetup, testedPLLSetup>() };
    testSysclkSetup<testedSYSCLKSetup>();
}

}
