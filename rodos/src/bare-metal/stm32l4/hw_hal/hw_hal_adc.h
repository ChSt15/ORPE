#pragma once

#include "hal/hal_adc.h"
#include "hw_hal_adc.h"

#include "peripherals.h"

namespace RODOS {

constexpr ADC_IDX STM32L4_MIN_ADC_IDX = ADC_IDX0;
constexpr ADC_IDX STM32L4_MAX_ADC_IDX = ADC_IDX2;
constexpr ADC_CHANNEL STM32L4_MIN_ADC_CHANNEL = ADC_CH_000;
constexpr ADC_CHANNEL STM32L4_MAX_ADC_CHANNEL = ADC_CH_018;


constexpr int64_t ADCVREG_SETUP_TIME = static_cast<int64_t>(static_cast<float>(20 * RODOS::MICROSECONDS)); /**< 20us: p.178, https://www.st.com/resource/en/datasheet/stm32l476je.pdf */
constexpr int64_t ADC_CALIBRATION_TIME_IN_NANOSECONDS = static_cast<int64_t>(15.0F * (1.0F / 140'000.0F) * RODOS::SECONDS); /**< fADC,min = 0.14MHz, https://www.st.com/resource/en/datasheet/stm32l476je.pdf */


/**
 * See reference manual chapter 18.4.12
 */
enum class ADC_SAMPLE_TIME : uint8_t
{
    CYCLES_2_POINT_5   = 0b000,
    CYCLES_6_POINT_5   = 0b001,
    CYCLES_12_POINT_5  = 0b010,
    CYCLES_24_POINT_5  = 0b011,
    CYCLES_47_POINT_5  = 0b100,
    CYCLES_92_POINT_5  = 0b101,
    CYCLES_247_POINT_5 = 0b110,
    CYCLES_640_POINT_5 = 0b111,
};

class HW_HAL_ADC {
public:
    HW_HAL_ADC(ADC_IDX idx);
    void init();
    uint16_t read(ADC_CHANNEL channel);

private:
    ADC_IDX m_idx;
    SingleADCStruct& m_adc;
    bool isGeneralInitializationDone;

    void setupSysclkAsAdcClock();
    void exitDeepPowerDown();
    void enableVoltageRegulator();
    void calibrate();
    void enableAdc();
    void setupSampleTimes();
    void setSingleConversionSequence(ADC_CHANNEL channel);

    static SingleADCStruct& getADCStruct(ADC_IDX idx);
};

}
