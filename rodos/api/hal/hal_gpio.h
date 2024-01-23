/*
 * Copyright (c) 2012 Uni Wuerzburg, Wuerzburg
 * All rights reserved.
 *
 * @author Sergio Montenegro, Michael Ruffer
 */

#pragma once


#include "genericIO.h"
#include <stdint.h>

namespace RODOS {

typedef enum {					// STM32F4      // AVR32 UC3A0512                   // NEXYS3-LEON3                 // EFR32FG12
    // Port A
	  GPIO_000 = 0,       // PA00         // AVR32_PIN_PA00,                  // 7SEG_DISPLAY_4_A             // PA00
	  GPIO_001,           // PA01         // AVR32_PIN_PA01,                  // 7SEG_DISPLAY_4_B             // PA01
    GPIO_002, 					// PA02         // AVR32_PIN_PA02,                  // 7SEG_DISPLAY_4_C             // PA02
    GPIO_003, 					// PA03         // AVR32_PIN_PA03,                  // 7SEG_DISPLAY_4_D             // PA03
    GPIO_004, 					// PA04         // AVR32_PIN_PA04,                  // 7SEG_DISPLAY_4_E             // PA04
    GPIO_005, 					// PA05         // AVR32_PIN_PA05,                  // 7SEG_DISPLAY_4_F             // PA05
    GPIO_006, 					// PA06         // AVR32_PIN_PA06,                  // 7SEG_DISPLAY_4_G             // PA06
    GPIO_007, 					// PA07         // AVR32_PIN_PA07,                  // 7SEG_DISPLAY_4_P             // PA07
    GPIO_008, 					// PA08         // AVR32_PIN_PA08,                  // 7SEG_DISPLAY_3_A             // PA08
    GPIO_009, 					// PA09         // AVR32_PIN_PA09,                  // 7SEG_DISPLAY_3_B             // PA09

    GPIO_010, 					// PA10         // AVR32_PIN_PA10,                  // 7SEG_DISPLAY_3_C             // PA10 - not used
    GPIO_011, 					// PA11         // AVR32_PIN_PA11,                  // 7SEG_DISPLAY_3_D             // PA11 - not used
    GPIO_012, 					// PA12         // AVR32_PIN_PA12,                  // 7SEG_DISPLAY_3_E             // PA12 - not used
    GPIO_013, 					// PA13         // AVR32_PIN_PA13,                  // 7SEG_DISPLAY_3_F             // PA13 - not used
    GPIO_014, 					// PA14         // AVR32_PIN_PA14,                  // 7SEG_DISPLAY_3_G             // PA14 - not used
    GPIO_015, 					// PA15         // AVR32_PIN_PA15,                  // 7SEG_DISPLAY_3_P             // PA15 - not used
    GPIO_016, 					// PB00         // AVR32_PIN_PA16,                  // 7SEG_DISPLAY_2_A             // PB00 - not used
    GPIO_017, 					// PB01         // AVR32_PIN_PA17,                  // 7SEG_DISPLAY_2_B             // PB01 - not used
    GPIO_018, 					// PB02         // AVR32_PIN_PA18,                  // 7SEG_DISPLAY_2_C             // PB02 - not used
    GPIO_019, 					// PB03         // AVR32_PIN_PA19,                  // 7SEG_DISPLAY_2_D             // PB03 - not used

    GPIO_020, 					// PB04         // AVR32_PIN_PA20,                  // 7SEG_DISPLAY_2_E             // PB04 - not used
    GPIO_021, 					// PB05         // AVR32_PIN_PA21,                  // 7SEG_DISPLAY_2_F             // PB05 - not used
    GPIO_022, 					// PB06         // AVR32_PIN_PA22,                  // 7SEG_DISPLAY_2_G             // PB06
    GPIO_023, 					// PB07         // AVR32_PIN_PA23,                  // 7SEG_DISPLAY_2_P             // PB07
    GPIO_024, 					// PB08         // AVR32_PIN_PA24,                  // 7SEG_DISPLAY_1_A             // PB08
    GPIO_025, 					// PB09         // AVR32_PIN_PA25,                  // 7SEG_DISPLAY_1_B             // PB09
    GPIO_026, 					// PB10         // AVR32_PIN_PA26,                  // 7SEG_DISPLAY_1_C             // PB10
    GPIO_027, 					// PB11         // AVR32_PIN_PA27,                  // 7SEG_DISPLAY_1_D             // PB11
    GPIO_028, 					// PB12         // AVR32_PIN_PA28,                  // 7SEG_DISPLAY_1_E             // PB12
    GPIO_029, 					// PB13         // AVR32_PIN_PA29,                  // 7SEG_DISPLAY_1_F             // PB13

    GPIO_030, 					// PB14         // AVR32_PIN_PA30,                  // 7SEG_DISPLAY_1_G             // PB14
    GPIO_031,				    // PB15         // is not connected to a pad/pin    // 7SEG_DISPLAY_1_P             // PB15
    GPIO_032, 					// PC00         // AVR32_PIN_PB00,                  // LED_1                        // PC00
    GPIO_033, 					// PC01         // AVR32_PIN_PB01,                  // LED_2                        // PC01
    GPIO_034, 					// PC02         // AVR32_PIN_PB02,                  // LED_3                        // PC02
    GPIO_035, 					// PC03         // AVR32_PIN_PB03,                  // LED_4                        // PC03
    GPIO_036, 					// PC04         // AVR32_PIN_PB04,                  // LED_5                        // PC04
    GPIO_037, 					// PC05         // AVR32_PIN_PB05,                  // LED_6                        // PC05
    GPIO_038, 					// PC06         // AVR32_PIN_PB06,                  // LED_7                        // PC06
    GPIO_039, 					// PC07         // AVR32_PIN_PB07,                  // LED_8                        // PC07

    GPIO_040, 					// PC08         // AVR32_PIN_PB08,                  // SWITCH_1                     // PC08
    GPIO_041, 					// PC09         // AVR32_PIN_PB09,                  // SWITCH_2                     // PC09
    GPIO_042, 					// PC10         // AVR32_PIN_PB10,                  // SWITCH_3                     // PC10
    GPIO_043, 					// PC11         // AVR32_PIN_PB11,                  // SWITCH_4                     // PC11
    GPIO_044, 					// PC12         // AVR32_PIN_PB12,                  // SWITCH_5                     // PC12 - not used
    GPIO_045, 					// PC13         // AVR32_PIN_PB13,                  // SWITCH_6                     // PC13 - not used
    GPIO_046, 					// PC14         // AVR32_PIN_PB14,                  // SWITCH_7                     // PC14 - not used
    GPIO_047, 					// PC15         // AVR32_PIN_PB15,                  // SWITCH_8                     // PC15 - not used
    GPIO_048, 					// PD0          // AVR32_PIN_PB16,                  // BUTTON_LEFT                  // PD0  - not used
    GPIO_049, 					// PD1          // AVR32_PIN_PB17,                  // BUTTON_DOWN                  // PD1  - not used

    GPIO_050, 					// PD2          // AVR32_PIN_PB18,                  // BUTTON_RIGHT                 // PD2  - not used
    GPIO_051, 					// PD3          // AVR32_PIN_PB19,                  // BUTTON_UP                    // PD3  - not used
    GPIO_052, 					// PD4          // AVR32_PIN_PB20,                  // -not used-                   // PD4  - not used
    GPIO_053, 					// PD5          // AVR32_PIN_PB21,                  // -not used-                   // PD5  - not used
    GPIO_054, 					// PD6          // AVR32_PIN_PB22,                  // -not used-                   // PD6  - not used
    GPIO_055, 					// PD7          // AVR32_PIN_PB23,                  // -not used-                   // PD7  - not used
    GPIO_056, 					// PD8          // AVR32_PIN_PB24,                  // -not used-                   // PD8
    GPIO_057, 					// PD9          // AVR32_PIN_PB25,                  // -not used-                   // PD9
    GPIO_058, 					// PD10         // AVR32_PIN_PB26,                  // -not used-                   // PD10
    GPIO_059, 					// PD11         // AVR32_PIN_PB27,                  // -not used-                   // PD11

    GPIO_060, 					// PD12         // AVR32_PIN_PB28,                  // -not used-                   // PD12
    GPIO_061, 					// PD13         // AVR32_PIN_PB29,                  // -not used-                   // PD13
    GPIO_062, 					// PD14         // AVR32_PIN_PB30,                  // -not used-                   // PD14
    GPIO_063, 					// PD15         // AVR32_PIN_PB31,                  // -not used-                   // PD15
    GPIO_064, 					// PE00         // AVR32_PIN_PC00,                  // PMOD_JC_GPIO_1_OUT           // PE00 - not used
    GPIO_065, 					// PE01         // AVR32_PIN_PC01,                  // PMOD_JC_GPIO_2_OUT           // PE01 - not used
    GPIO_066, 					// PE02         // AVR32_PIN_PC02,                  // PMOD_JC_GPIO_3_IN            // PE02 - not used
    GPIO_067, 					// PE03         // AVR32_PIN_PC03,                  // PMOD_JC_GPIO_4_IN            // PE03 - not used
    GPIO_068, 					// PE04         // AVR32_PIN_PC04,                  // -not used-                   // PE04 - not used
    GPIO_069, 					// PE05         // AVR32_PIN_PC05,                  // -not used-                   // PE05 - not used

    GPIO_070, 					// PE06         // AVR32_PIN_PX34,                  // -not used-                   // PE06 - not used
    GPIO_071, 					// PE07         // AVR32_PIN_PX33,                  // -not used-                   // PE07 - not used
    GPIO_072, 					// PE08         // AVR32_PIN_PX32,                  // -not used-                   // PE08 - not used
    GPIO_073, 					// PE09         // AVR32_PIN_PX31,                  // -not used-                   // PE09 - not used
    GPIO_074, 					// PE10         // AVR32_PIN_PX30,                  // -not used-                   // PE10 - not used
    GPIO_075, 					// PE11         // AVR32_PIN_PX29,                  // -not used-                   // PE11 - not used
    GPIO_076,		        // PE12         // AVR32_PIN_PX28,                  // -not used-                   // PE12 - not used
    GPIO_077, 					// PE13         // AVR32_PIN_PX27,                  // -not used-                   // PE13 - not used
    GPIO_078, 					// PE14         // AVR32_PIN_PX26,                  // -not used-                   // PE14 - not used
    GPIO_079, 					// PE15         // AVR32_PIN_PX25,                  // -not used-                   // PE15 - not used

    GPIO_080, 					// PF00         // AVR32_PIN_PX24,                  // -not used-                   // PF00
    GPIO_081, 					// PF01         // AVR32_PIN_PX23,                  // -not used-                   // PF01
    GPIO_082, 					// PF02         // AVR32_PIN_PX22,                  // -not used-                   // PF02
    GPIO_083, 					// PF03         // AVR32_PIN_PX21,                  // -not used-                   // PF03
    GPIO_084, 					// PF04         // AVR32_PIN_PX20,                  // -not used-                   // PF04
    GPIO_085, 					// PF05         // AVR32_PIN_PX19,                  // -not used-                   // PF05
    GPIO_086, 					// PF06         // AVR32_PIN_PX18,                  // -not used-                   // PF06
    GPIO_087, 					// PF07         // AVR32_PIN_PX17,                  // -not used-                   // PF07
    GPIO_088, 					// PF08         // AVR32_PIN_PX16,                  // -not used-                   // PF08
    GPIO_089, 					// PF09         // AVR32_PIN_PX15,                  // -not used-                   // PF09

    GPIO_090, 					// PF10         // AVR32_PIN_PX10,                  // -not used-                   // PF10
    GPIO_091, 					// PF11         // AVR32_PIN_PX09,                  // -not used-                   // PF11
    GPIO_092, 					// PF12         // AVR32_PIN_PX08,                  // -not used-                   // PF12
    GPIO_093, 					// PF13         // AVR32_PIN_PX07,                  // -not used-                   // PF13
    GPIO_094, 					// PF14         // AVR32_PIN_PX06,                  // -not used-                   // PF14
    GPIO_095, 					// PF15         // AVR32_PIN_PX05,                  // -not used-                   // PF15
    GPIO_096,           // PG00         // AVR32_PIN_PX04,                                                  // PG00 - not used
    GPIO_097,           // PG01         // AVR32_PIN_PX03,                                                  // PG01 - not used
    GPIO_098,           // PG02         // AVR32_PIN_PX02,                                                  // PG02 - not used
    GPIO_099,           // PG03         // AVR32_PIN_PX01,                                                  // PG03 - not used

    GPIO_100,           // PG04         // AVR32_PIN_PX00,                                                  // PG04 - not used
    GPIO_101,           // PG05         // AVR32_PIN_PX39,                                                  // PG05 - not used
    GPIO_102,           // PG06         // AVR32_PIN_PX38,                                                  // PG06 - not used
    GPIO_103,           // PG07         // AVR32_PIN_PX37,                                                  // PG07 - not used
    GPIO_104,           // PG08         // AVR32_PIN_PX36,                                                  // PG08 - not used
    GPIO_105,           // PG09         // AVR32_PIN_PX35,                                                  // PG09 - not used
    GPIO_106,           // PG10         // AVR32_PIN_PX14,                                                  // PG10 - not used
    GPIO_107,           // PG11         // AVR32_PIN_PX13,                                                  // PG11 - not used
    GPIO_108,           // PG12         // AVR32_PIN_PX12,                                                  // PG12 - not used
    GPIO_109,           // PG13         // AVR32_PIN_PX11                                                   // PG13 - not used

    GPIO_110, 					// PG14                                                                             // PG14 - not used
    GPIO_111, 					// PG15                                                                             // PG15 - not used
    GPIO_112, 					// PH00                                                                             // PH00 - not used
    GPIO_113, 					// PH01                                                                             // PH01 - not used
    GPIO_114, 					// PH02                                                                             // PH02 - not used
    GPIO_115, 					// PH03                                                                             // PH03 - not used
    GPIO_116, 					// PH04                                                                             // PH04 - not used
    GPIO_117, 					// PH05                                                                             // PH05 - not used
    GPIO_118, 					// PH06                                                                             // PH06 - not used
    GPIO_119, 					// PH07                                                                             // PH07 - not used

    GPIO_120, 					// PH08                                                                             // PH08 - not used
    GPIO_121, 					// PH09                                                                             // PH09 - not used
    GPIO_122, 					// PH10                                                                             // PH10 - not used
    GPIO_123, 					// PH11                                                                             // PH11 - not used
    GPIO_124, 					// PH12                                                                             // PH12 - not used
    GPIO_125, 					// PH13                                                                             // PH13 - not used
    GPIO_126,           // PH14                                                                             // PH14 - not used
    GPIO_127,           // PH15                                                                             // PH15 - not used
    GPIO_128,           // PI00                                                                             // PI00
    GPIO_129,           // PI01                                                                             // PI01

    GPIO_130,           // PI02                                                                             // PI02
    GPIO_131,           // PI03                                                                             // PI03
    GPIO_132,           // PI04                                                                             // PI04 - not used
    GPIO_133,           // PI05                                                                             // PI05 - not used
    GPIO_134,           // PI06                                                                             // PI06 - not used
    GPIO_135,           // PI07                                                                             // PI07 - not used
    GPIO_136,           // PI08                                                                             // PI08 - not used
    GPIO_137,           // PI09                                                                             // PI09 - not used
    GPIO_138,           // PI10                                                                             // PI10 - not used
    GPIO_139,           // PI11                                                                             // PI11 - not used

    GPIO_140,                                                                                               // PI12 - not used
    GPIO_141,                                                                                               // PI13 - not used
    GPIO_142,                                                                                               // PI14 - not used
    GPIO_143,                                                                                               // PI15 - not used
    GPIO_144,                                                                                               // PJ00 - not used
    GPIO_145,                                                                                               // PJ01 - not used
    GPIO_146,                                                                                               // PJ02 - not used
    GPIO_147,                                                                                               // PJ03 - not used
    GPIO_148,                                                                                               // PJ04 - not used
    GPIO_149,                                                                                               // PJ05 - not used

    GPIO_150,                                                                                               // PJ06 - not used
    GPIO_151,                                                                                               // PJ07 - not used
    GPIO_152,                                                                                               // PJ08 - not used
    GPIO_153,                                                                                               // PJ09 - not used
    GPIO_154,                                                                                               // PJ10 - not used
    GPIO_155,                                                                                               // PJ11 - not used
    GPIO_156,                                                                                               // PJ12 - not used
    GPIO_157,                                                                                               // PJ13 - not used
    GPIO_158,                                                                                               // PJ14
    GPIO_159,                                                                                               // PJ15

    GPIO_160,                                                                                               // PK00
    GPIO_161,                                                                                               // PK01
    GPIO_162,                                                                                               // PK02
    GPIO_163,                                                                                               // PK03 - not used
    GPIO_164,                                                                                               // PK04 - not used
    GPIO_165,                                                                                               // PK05 - not used
    GPIO_166,                                                                                               // PK06 - not used
    GPIO_167,                                                                                               // PK07 - not used
    GPIO_168,                                                                                               // PK08 - not used
    GPIO_169,                                                                                               // PK09 - not used

    GPIO_170,                                                                                               // PK10 - not used
    GPIO_171,                                                                                               // PK11 - not used
    GPIO_172,                                                                                               // PK12 - not used
    GPIO_173,                                                                                               // PK13 - not used
    GPIO_174,                                                                                               // PK14 - not used
    GPIO_175,                                                                                               // PK15 - not used
    GPIO_INVALID = -1

}GPIO_PIN;


typedef enum {
	GPIO_CFG_OUTPUT_ENABLE = 0,
	GPIO_CFG_NUM_OF_PINS,
	GPIO_CFG_PULLUP_ENABLE,
	GPIO_CFG_PULLDOWN_ENABLE,
	//GPIO_CFG_OPENDRAIN_ENABLE,
	GPIO_CFG_IRQ_SENSITIVITY // falling, rising, both edge(s)
} GPIO_CFG_TYPE;


typedef enum {
	GPIO_IRQ_SENS_BOTH = 0,
	GPIO_IRQ_SENS_RISING,
	GPIO_IRQ_SENS_FALLING
} GPIO_IRQ_SENSITIVITY;


class HW_HAL_GPIO;

class HAL_GPIO : public GenericIOInterface {
private:
	HW_HAL_GPIO *context;
public:
	HAL_GPIO(GPIO_PIN pinIdx = GPIO_000);

	/* Be careful when defining pin-groups using "numOfPins". The use of all functions is
	 * limited to port boundaries! When port boundaries are exceeded only pins up to most
	 * significant pin of port will be set/read.
	 */
	int32_t init(bool isOutput = false, uint8_t numOfPins = 1, uint32_t initVal = 0x00);
	void reset();
	int32_t config(GPIO_CFG_TYPE type, uint32_t paramVal);

	bool isDataReady();
	bool isWriteFinished() {return true;}
	bool isReadFinished() {return true;}

	void setPins(uint32_t);
	uint32_t readPins();

	/* interrupts supported by: STM32F4 */
	//When rising and/or falling edge occurs isDataReady() == true
	void interruptEnable(bool enable);
	// dataReady == false
	//After this function, isDataReady() returns false, until the configured edge occurs on the gpio pin.
	//For e.g if you want to wait until a signal has high level, configure for rising edge and enable interrupts.
	//Then use this Function, then check if the signal already has high level, if not do suspendUntilDataReady(), then the signal should have high level.
	//This is all thread save so you won't miss the edge while suspending
	void resetInterruptEventStatus();


};

}

