#include "hw_hal_adc.h"
#include "peripheral_ctrl/peripheral_defs.h"

namespace RODOS {

void HW_HAL_ADC::init() {
    sysconfigPeripheralBitband->ADC_ENABLE.write(1);
}

uint16_t HW_HAL_ADC::read(ADC_CHANNEL channel){
    using namespace ADC_CTRL;
    using namespace ADC_STATUS;
    if(channel>=16){
        return static_cast<uint16_t>(ADC_ERR_INVALID_CHANNEL);
    }
    adc->FIFO_CLR.write(1);
    adc->CTRL.write(CHAN_EN(uint16_t(1<<channel)), MANUAL_TRIG(1), CONV_CNT(0));

    while(adc->STATUS.read<ADC_BUSY>());
    return static_cast<uint16_t>(adc->FIFO_DATA.read());
}

}
