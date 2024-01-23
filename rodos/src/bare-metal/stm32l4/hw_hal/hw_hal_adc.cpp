#include "hw_hal_adc.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripheral_ctrl/gpio/pin_group.h"
#include "hal/hal_adc.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"
#include "rodos.h"

#include "peripherals.h"

namespace RODOS {

HW_HAL_ADC::HW_HAL_ADC(ADC_IDX idx)
    : m_idx { idx }
    , m_adc { getADCStruct(idx) }
{
}

SingleADCStruct& HW_HAL_ADC::getADCStruct(ADC_IDX idx)
{
    return ADCDef::periphStruct->adcX[idx];
}

void HW_HAL_ADC::exitDeepPowerDown(){
    using namespace RODOS::ADC_CR;
    m_adc.CR.set(DEEPPWD(0));
}

void HW_HAL_ADC::enableVoltageRegulator(){
    using namespace RODOS::ADC_CR;
    m_adc.CR.set(ADVREGEN(1));
    busyWaitUntil(NOW()+ADCVREG_SETUP_TIME);
}

void HW_HAL_ADC::calibrate(){
    using namespace RODOS::ADC_CR;
    m_adc.CR.set(ADCAL(1));
    while (m_adc.CR.read<ADCAL>());
}

void HW_HAL_ADC::enableAdc(){
    using namespace RODOS::ADC_CR;
    using namespace RODOS::ADC_ISR;
    m_adc.CR.set(ADEN(1));
    while(!m_adc.ISR.read<ADRDY>());
}

void HW_HAL_ADC::setupSampleTimes(){
    using namespace RODOS::ADC_SMPR1;
    using namespace RODOS::ADC_SMPR2;
    constexpr uint32_t DEFAULT_SMP { static_cast<uint32_t>(ADC_SAMPLE_TIME::CYCLES_24_POINT_5) }; // value arbitrarily chosen
    constexpr uint32_t TEMPERATURE_SENSOR_SMP { static_cast<uint32_t>(ADC_SAMPLE_TIME::CYCLES_640_POINT_5) }; // sensor needs at least 5us sample time(datasheet 6.3.23), which requires 400 cycles at 80MHz
    m_adc.SMPR1.write(
        SMP0(DEFAULT_SMP),
        SMP1(DEFAULT_SMP),
        SMP2(DEFAULT_SMP),
        SMP3(DEFAULT_SMP),
        SMP4(DEFAULT_SMP),
        SMP5(DEFAULT_SMP),
        SMP6(DEFAULT_SMP),
        SMP7(DEFAULT_SMP),
        SMP8(DEFAULT_SMP),
        SMP9(DEFAULT_SMP)
    );
    if(m_idx==ADC_IDX0 || m_idx==ADC_IDX2){
        m_adc.SMPR2.write(
            SMP10(DEFAULT_SMP),
            SMP11(DEFAULT_SMP),
            SMP12(DEFAULT_SMP),
            SMP13(DEFAULT_SMP),
            SMP14(DEFAULT_SMP),
            SMP15(DEFAULT_SMP),
            SMP16(DEFAULT_SMP),
            SMP17(TEMPERATURE_SENSOR_SMP),
            SMP18(DEFAULT_SMP)
        );
    }else{
        m_adc.SMPR2.write(
            SMP10(DEFAULT_SMP),
            SMP11(DEFAULT_SMP),
            SMP12(DEFAULT_SMP),
            SMP13(DEFAULT_SMP),
            SMP14(DEFAULT_SMP),
            SMP15(DEFAULT_SMP),
            SMP16(DEFAULT_SMP),
            SMP17(DEFAULT_SMP),
            SMP18(DEFAULT_SMP)
        );
    }
}

void HW_HAL_ADC::setupSysclkAsAdcClock(){
    using namespace RCC_CCIPR;
    constexpr uint32_t ADCSEL_SYSCLK_VALUE { 0b11 };
    rcc.CCIPR.set(ADCSEL(ADCSEL_SYSCLK_VALUE));
}

void HW_HAL_ADC::init()
{
    ADCDef::rccPeriph->enableClock();

    setupSysclkAsAdcClock();

    exitDeepPowerDown();
    enableVoltageRegulator();
    calibrate();
    setupSampleTimes();
    enableAdc();
}

void HW_HAL_ADC::setSingleConversionSequence(ADC_CHANNEL channel){
    using namespace ADC_SQR1;
    m_adc.SQR1.write(L(0), SQ1(static_cast<uint32_t>(channel)));
}

uint16_t HW_HAL_ADC::read(ADC_CHANNEL channel)
{
    using namespace RODOS::ADC_CR;
    using namespace RODOS::ADC_ISR;
    using namespace RODOS::ADC_DR;

    if ((static_cast<int>(channel) > static_cast<int>(STM32L4_MAX_ADC_CHANNEL))) {
        return ADC_ERR_INVALID_CHANNEL;
    }

    setSingleConversionSequence(channel);
    m_adc.CR.set(ADSTART(1)); // Start Conversion
    while (m_adc.CR.read<ADSTART>());

    uint16_t adcVal = static_cast<uint16_t>(m_adc.DR.read<RDATA>());

    return adcVal;
}

}
