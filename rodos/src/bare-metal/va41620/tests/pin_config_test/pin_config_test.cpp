#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "peripheral_ctrl/pin_config/pin_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "tests/peripheral_test_fixture.h"

namespace {
using ::testing::Sequence;
using ::testing::InSequence;
using ::testing::_;
using ::testing::Return;
using ::testing::Mock;

using namespace RODOS::testing;
using namespace RODOS::pin_config;

class PinConfigTest : public PeripheralTestFixture {};

TEST_F(PinConfigTest, TestEnableIOCONFIGPeripheral){
    EXPECT_CALL(RODOS::sysconfigPeripheralBitband->IOCONFIG_ENABLE, write(1));
    enableIOConfigPeripheral();
}

TEST_F(PinConfigTest, TestConfigurePinWorksForAllConfigs){
    using namespace RODOS::IOCONFIG_PINx;
    InSequence s;
    OUTPUT_TYPE outputTypes[] {OUTPUT_TYPE::PUSHPULL, OUTPUT_TYPE::OPENDRAIN};
    PIN_FUNCTION functions[] {PIN_FUNCTION::FUN0, PIN_FUNCTION::FUN1, PIN_FUNCTION::FUN2, PIN_FUNCTION::FUN3};
    PULL pulls[] {PULL::OFF, PULL::UP, PULL::DOWN};
    for (OUTPUT_TYPE outputType : outputTypes){
        for (PIN_FUNCTION function : functions){
            for(PULL pull : pulls){
                EXPECT_CALL(RODOS::ioconfig->PINx[0], write(
                    FUNSEL(static_cast<uint32_t>(function)) |
                    OPENDRN(static_cast<uint32_t>(outputType)) |
                    PEN((pull!=PULL::OFF)?1:0) |
                    PLEVEL((pull==PULL::UP?1:0))
                ));
                configurePin(0, function, outputType, pull);
                ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::ioconfig->PINx[0]));
            }
        }
    }
}

TEST_F(PinConfigTest, TestConfigurePinConfiguresCorrectPin){
    InSequence s;
    for(uint32_t i = 0; i<RODOS::TOTAL_NUMBER_OF_PINS; i++){
        EXPECT_CALL(RODOS::ioconfig->PINx[i], write(0));
        configurePin(i, PIN_FUNCTION::FUN0);
        ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::ioconfig->PINx[i]));
    }
}

TEST_F(PinConfigTest, TestResetPin){
    InSequence s;
    for(uint32_t i = 0; i<RODOS::TOTAL_NUMBER_OF_PINS; i++){
        EXPECT_CALL(RODOS::ioconfig->PINx[i], write(0));
        resetPin(i);
        ASSERT_TRUE(Mock::VerifyAndClear(&RODOS::ioconfig->PINx[i]));
    }
}

TEST_F(PinConfigTest, TestInvalidPinGetsIgnored){
    configurePin(RODOS::GPIO_INVALID, PIN_FUNCTION::FUN0);
}

}
