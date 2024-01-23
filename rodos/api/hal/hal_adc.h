#pragma once


#include "hal/genericIO.h"
#include <stdint.h>

namespace RODOS {

/*
 * HAL_ADC
 *
 * STM32F4xx
 * - there are 3 ADCs (ADC_IDX1, ADC_IDX2, ADC_IDX3)
 * - each ADC has 16 general purpose channels, BUT the channels share same pins!!!
 *   -> so there is a maximum of 24 analog inputs (instead of 3x16)
 *   -> see hal_adc.cpp for details
 *
 * AVR32
 * - TBD
 *
 * STM32F1
 * - TBD
 *
 */


enum ADC_ERROR{
    ADC_ERR_INVALID_INDEX           = -1,
    ADC_ERR_INVALID_CHANNEL         = -2,
    ADC_ERR_CONV_FAIL               = -3,
    ADC_ERR_NO_INIT                 = -4,
    ADC_ERR_PARAMETER_INVALID       = -5,
    ADC_ERR_PARAMETER_VALUE_INVALID = -6
};

/**
 * index of ADC
 * - not every device offers all
 */
enum ADC_IDX {          // STM32F4      AVR32       STM32F1 ...
    ADC_IDX0 = 0,       //                 ?            ?
	ADC_IDX1,           //    X            ?            ?
	ADC_IDX2,           //    X            ?            ?
	ADC_IDX3            //    X            ?            ?
};

/**
 * index of ADC channel
 * - not every device offers all
 */
enum ADC_CHANNEL {  // STM32F4      AVR32       STM32F1 ...     EFR32
                    // ADC1/2/3
    ADC_CH_000,     // PA0/PA0/PA0     ?            ?           PA0
    ADC_CH_001,     // PA1/PA1/PA1     ?            ?           PA1
    ADC_CH_002,     // PA2/PA2/PA2     ?            ?           PA2
    ADC_CH_003,     // PA3/PA3/PA3     ?            ?           PA3
    ADC_CH_004,     // PA4/PA4/PF6     ?            ?           PA4
    ADC_CH_005,     // PA5/PA5/PF7     ?            ?           PA5
    ADC_CH_006,     // PA6/PA6/PF8     ?            ?           PB11
    ADC_CH_007,     // PA7/PA7/PF9     ?            ?           PB12
    ADC_CH_008,     // PB0/PB0/PF10    ?            ?           PB13
    ADC_CH_009,     // PB1/PB1/PF3     ?            ?           PB14
    ADC_CH_010,     // PC0/PC0/PC0     ?            ?           PB15
    ADC_CH_011,     // PC1/PC1/PC1     ?            ?           PC6
    ADC_CH_012,     // PC2/PC2/PC2     ?            ?           PC7
    ADC_CH_013,     // PC3/PC3/PC3     ?            ?           PC8
    ADC_CH_014,     // PC4/PC4/PF4     ?            ?           PC9
    ADC_CH_015,     // PC5/PC5/PF5     ?            ?           PC10
    ADC_CH_016,     // intern          ?            ?           PC11
    ADC_CH_017,     // intern          ?            ?           PD13
    ADC_CH_018,     // intern          ?            ?           PD14
    ADC_CH_019,     //                                          PD15
    ADC_CH_020,     //                                          PF0
    ADC_CH_021,     //                                          PF1
    ADC_CH_022,     //                                          PF2
    ADC_CH_023,     //                                          PF3
    ADC_CH_024,     //                                          PF4
    ADC_CH_025,     //                                          PF5
    ADC_CH_026,     //                                          PF6
    ADC_CH_027,     //                                          PF7
    ADC_CH_028      //                                          PF9
};

enum ADC_PARAMETER_TYPE {
    ADC_PARAMETER_RESOLUTION,
    ADC_PARAMETER_MUX,
    ADC_PARAMETER_REF
    //ADC_PARAMETER_SAMPLETIME
};

enum ADC_VREF_TYPE {
    ADC_VREF_VCC,
    ADC_VREF_EXT,
    ADC_VREF_5V,
    ADC_VREF_3V3,
    ADC_VREF_2V5,
    ADC_VREF_1V25,
    ADC_VREF_HALF_VCC,
    ADC_VREF_DOUBLE_VCC
};

class HW_HAL_ADC;

class HAL_ADC : public GenericIOInterface {
    HW_HAL_ADC *context;

public:
	HAL_ADC(ADC_IDX idx);

	int32_t init(ADC_CHANNEL adc_Channel);
	int32_t config(ADC_PARAMETER_TYPE type, int32_t value);
	void reset();


	uint16_t read(ADC_CHANNEL channel);

    virtual bool isWriteFinished(){return false;}
    virtual bool isReadFinished(){return true;}
};

}
