#pragma once
#include "peripheral_ctrl/pin_config/enums.h"
#include "gmock/gmock.h"

namespace RODOS::testing {

using namespace RODOS::pin_config;
class PinConfigMock {
public:
    MOCK_METHOD(void, configurePin, (VA41620_PIN pin, PIN_FUNCTION function, OUTPUT_TYPE type, PULL pull));
    MOCK_METHOD(void, resetPin, (VA41620_PIN pin));
};

extern ::testing::StrictMock<PinConfigMock>* globalPinConfigMock;

}
