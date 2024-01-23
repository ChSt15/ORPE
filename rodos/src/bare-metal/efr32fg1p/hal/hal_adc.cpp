//#include <new>
//#include <string.h>

//#include "rodos.h"
#include "hal/hal_adc.h"
#include "rodos.h"

#include "vendor-headers.h"

#define adcFreq   16000000


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

#define EFR32_MIN_ADC_IDX     ADC_IDX0
#define EFR32_MAX_ADC_IDX     ADC_IDX0

#define EFR32_MIN_ADC_CHAN    ADC_CH_000
#define EFR32_MAX_ADC_CHAN    ADC_CH_028



ADC_PosSel_TypeDef adcChannelMap[] = {

    //                     ADC_CHANNEL  efr32 pin
    adcPosSelAPORT3XCH8,  //ADC_CH_000,   PA0
    adcPosSelAPORT4XCH9,  //ADC_CH_001,   PA1
    adcPosSelAPORT3XCH10, //ADC_CH_002,   PA2
    adcPosSelAPORT4XCH11, //ADC_CH_003,   PA3
    adcPosSelAPORT3XCH12, //ADC_CH_004,   PA4
    adcPosSelAPORT4XCH13, //ADC_CH_005,   PA5
    adcPosSelAPORT4XCH27, //ADC_CH_006,   PB11
    adcPosSelAPORT3XCH28, //ADC_CH_007,   PB12
    adcPosSelAPORT4XCH29, //ADC_CH_008,   PB13
    adcPosSelAPORT3XCH30, //ADC_CH_009,   PB14
    adcPosSelAPORT4XCH31, //ADC_CH_010,   PB15
    adcPosSelAPORT1XCH6,  //ADC_CH_011,   PC6
    adcPosSelAPORT2XCH7,  //ADC_CH_012,   PC7
    adcPosSelAPORT1XCH8,  //ADC_CH_013,   PC8
    adcPosSelAPORT2XCH9,  //ADC_CH_014,   PC9
    adcPosSelAPORT1XCH10, //ADC_CH_015,   PC10
    adcPosSelAPORT2XCH11, //ADC_CH_016,   PC11
    adcPosSelAPORT4XCH5,  //ADC_CH_017,   PD13
    adcPosSelAPORT3XCH6,  //ADC_CH_018,   PD14
    adcPosSelAPORT4XCH7,  //ADC_CH_019,   PD15
    adcPosSelAPORT1XCH16, //ADC_CH_020,   PF0
    adcPosSelAPORT2XCH17, //ADC_CH_021,   PF1
    adcPosSelAPORT1XCH18, //ADC_CH_022,   PF2
    adcPosSelAPORT2XCH19, //ADC_CH_023,   PF3
    adcPosSelAPORT1XCH20, //ADC_CH_024,   PF4
    adcPosSelAPORT2XCH21, //ADC_CH_025,   PF5
    adcPosSelAPORT1XCH22, //ADC_CH_026,   PF6
    adcPosSelAPORT2XCH23, //ADC_CH_027,   PF7
    adcPosSelAPORT2XCH25  //ADC_CH_028,   PF9
};



class HW_HAL_ADC{
    friend class HAL_ADC;
public:
    ADC_TypeDef *adc;
    int32_t idx;
    ADC_Res_TypeDef resolution;
    ADC_Ref_TypeDef reference;

    bool initialized;


    HW_HAL_ADC(){/* do not initialize any member variables here !!! */}

    HW_HAL_ADC(ADC_IDX idx, ADC_TypeDef * adc){
        if ( (idx < EFR32_MIN_ADC_IDX) || (idx > EFR32_MAX_ADC_IDX) ) return;

        initialized = false;
        this->idx=idx;
        this->adc=adc;

    }

private:

};


HW_HAL_ADC adcContextArray[EFR32_MAX_ADC_IDX+1] = { HW_HAL_ADC(ADC_IDX0,ADC0) };


HAL_ADC::HAL_ADC(ADC_IDX idx) {
    if ( (static_cast<int>(idx) < static_cast<int>(EFR32_MIN_ADC_IDX)) || (static_cast<int>(idx) > static_cast<int>(EFR32_MAX_ADC_IDX)) ) {
        context->idx = EFR32_MAX_ADC_IDX + 1; // set idx bigger than STM32F4_MAX_ADC_IDX to easily find out that idx is wrong (in all other methods)
        return;
    }

    context = &adcContextArray[idx];
}


int32_t HAL_ADC::init(ADC_CHANNEL channel){
    if (static_cast<int>(context->idx) > static_cast<int>(EFR32_MAX_ADC_IDX)) return ADC_ERR_INVALID_INDEX;
    if ( (static_cast<int>(channel) > static_cast<int>(EFR32_MAX_ADC_CHAN))) return ADC_ERR_INVALID_CHANNEL;

    // Enable ADC0 clock
    CMU_ClockEnable(cmuClock_ADC0, true);

    // Declare init structs
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;


    // Modify init structs and initialize
    init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1



    init.timebase = ADC_TimebaseCalc(0);

    ADC_Init(context->adc, &init);


    context->resolution = adcRes12Bit;
    context->reference = adcRef2V5;
	context->initialized = true;

	return 0;
}


int32_t HAL_ADC::config(ADC_PARAMETER_TYPE type, int32_t value){
    if (static_cast<int>(context->idx) > static_cast<int>(EFR32_MAX_ADC_IDX)) return ADC_ERR_INVALID_INDEX;



    switch(type){
    case ADC_PARAMETER_RESOLUTION:

        switch(value){
        case 6:  context->resolution = adcRes6Bit; break;
        case 8:  context->resolution = adcRes8Bit; break;
        case 12: context->resolution = adcRes12Bit; break;
        default: return ADC_ERR_PARAMETER_VALUE_INVALID;
        }

        break;
    case ADC_PARAMETER_REF:
        switch (value) {
            case ADC_VREF_VCC: context->reference = adcRefVDD; break;
            case ADC_VREF_EXT: context->reference = adcRefExtSingle; break;
            case ADC_VREF_5V: context->reference = adcRef5V; break;
            case ADC_VREF_2V5: context->reference = adcRef2V5; break;
            case ADC_VREF_1V25: context->reference = adcRef1V25; break;
            case ADC_VREF_DOUBLE_VCC: context->reference = adcRef2xVDD; break;
            default: return ADC_ERR_PARAMETER_VALUE_INVALID;
        }
        break;
    default:
        return ADC_ERR_PARAMETER_INVALID;
    }

    return 0;
}


void HAL_ADC::reset(){
    if (static_cast<int>(context->idx) > static_cast<int>(EFR32_MAX_ADC_IDX)) return;

    init(ADC_CH_000);


}


uint16_t HAL_ADC::read(ADC_CHANNEL channel) {
    if (static_cast<int>(context->idx) > static_cast<int>(EFR32_MAX_ADC_IDX)) return ADC_ERR_INVALID_INDEX;
    if ( (static_cast<int>(channel) > static_cast<int>(EFR32_MAX_ADC_CHAN))) return ADC_ERR_INVALID_CHANNEL;
    if(!context->initialized) return ADC_ERR_NO_INIT;

    ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;
    initSingle.diff       = false;        // single ended
    initSingle.reference  = context->reference;    // voltage reference, set by config
    initSingle.resolution = context->resolution;  // resolution, set by config
    initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

    // Select ADC input. See README for corresponding EXP header pin.
    initSingle.posSel = adcChannelMap[(int)channel];
    ADC_InitSingle(context->adc, &initSingle);

    // Start ADC conversion
    ADC_Start(context->adc, adcStartSingle);

    // Wait for conversion to be complete
    while(!(context->adc->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    uint32_t adcVal = ADC_DataSingleGet(context->adc);


    return static_cast<uint16_t>(adcVal);
}









#ifndef NO_RODOS_NAMESPACE
}
#endif

