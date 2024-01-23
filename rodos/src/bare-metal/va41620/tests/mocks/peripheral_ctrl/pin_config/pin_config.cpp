#include "peripheral_ctrl/pin_config/pin_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "pin_config_mock.h"

namespace RODOS::pin_config {

void configurePin(VA41620_PIN pin, PIN_FUNCTION function, OUTPUT_TYPE type, PULL pull){
    RODOS::testing::globalPinConfigMock->configurePin(pin, function, type, pull);
}
void resetPin(VA41620_PIN pin){
    RODOS::testing::globalPinConfigMock->resetPin(pin);
}
}
