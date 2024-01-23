#include "hal/hal_adc.h"
#include "thread.h"
#include "rodos-debug.h"

RODOS::HAL_ADC adc {RODOS::ADC_IDX0};

class ADCTestThread : RODOS::StaticThread<>{
    void run() final {
        adc.init(RODOS::ADC_CH_000);
        TIME_LOOP(0, 1*RODOS::SECONDS){
            printValue("AN_IN0      ", RODOS::ADC_CH_000);
            printValue("AN_IN1      ", RODOS::ADC_CH_001);
            printValue("AN_IN2      ", RODOS::ADC_CH_002);
            printValue("AN_IN3      ", RODOS::ADC_CH_003);
            printValue("AN_IN4      ", RODOS::ADC_CH_004);
            printValue("AN_IN5      ", RODOS::ADC_CH_005);
            printValue("AN_IN6      ", RODOS::ADC_CH_006);
            printValue("AN_IN7      ", RODOS::ADC_CH_007);

            printValue("DAC0        ", RODOS::ADC_CH_008);
            printValue("DAC1        ", RODOS::ADC_CH_009);
            printValue("Temperature ", RODOS::ADC_CH_010, true);
            printValue("Bandgap 1.0V", RODOS::ADC_CH_011);
            printValue("Bandgap 1.5V", RODOS::ADC_CH_012);
            printValue("AVDD15      ", RODOS::ADC_CH_013);
            printValue("DVDD15      ", RODOS::ADC_CH_014);
            printValue("VREFP5      ", RODOS::ADC_CH_015);
            RODOS::PRINTF("\n");
        }
    }

    static void printValue(const char* channelName, RODOS::ADC_CHANNEL channel, bool convertToTemperature = false){
        const uint16_t value { adc.read(channel) };
        const float volts { convertToVolts(value) };
        RODOS::PRINTF("Channel %2d(%s): %4uLSB = %1.4fV", channel, channelName, value, volts);
        if(convertToTemperature){
            const float temperature { (volts-0.6796f)/-0.0022f };
            RODOS::PRINTF(" = %3.1fdegC", temperature);
        }
        RODOS::PRINTF("\n");
    }

    static constexpr float convertToVolts(uint16_t value){
        constexpr float VREF { 3.3f };
        constexpr float conversionFactor { VREF / static_cast<float>(0xFFF) };
        return value*conversionFactor;
    }
};

ADCTestThread adcTestThread {};
