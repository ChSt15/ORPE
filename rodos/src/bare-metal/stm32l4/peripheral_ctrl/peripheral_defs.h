#pragma once

#include "register_maps/adc_registers.h"
#include "register_maps/gpio_registers.h"
#include "register_maps/pwr_registers.h"
#include "register_maps/rcc_registers.h"
#include "register_maps/spi_registers.h"
#include "register_maps/can_registers.h"
#include "register_maps/i2c_registers.h"
#include "register_maps/uart_registers.h"
#include "register_maps/exti_registers.h"
#include "register_maps/syscfg_registers.h"
#include "register_maps/dbg_registers.h"
#include "register_maps/dwt_registers.h"
#include "register_maps/dbgmcu_registers.h"
#include "register_maps/uid_registers.h"
#include "register_maps/flash_registers.h"
#include "register_maps/basic_timer_registers.h"
#include "register_maps/tim_registers.h"

#include "periph.h"
#include "pinmap.h"
#include "irqn.h"

#include "cortex_m/hw_periph_bitband.h"

namespace RODOS {

using AF = GPIO::ALTERNATEFUNCTION;

using GPIOADef = Periph<0x4800'0000, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 0>>;
using GPIOBDef = Periph<0x4800'0400, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 1>>;
using GPIOCDef = Periph<0x4800'0800, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 2>>;
using GPIODDef = Periph<0x4800'0C00, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 3>>;
using GPIOEDef = Periph<0x4800'1000, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 4>>;
using GPIOFDef = Periph<0x4800'1400, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 5>>;
using GPIOGDef = Periph<0x4800'1800, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 6>>;
using GPIOHDef = Periph<0x4800'1C00, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 7>>;
using GPIOIDef = Periph<0x4800'2000, GPIOStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 8>>;

using DMA1Def  = Periph<0x4002'0000, int, HWRCCBitBand<RCC_AHB1RSTR_BASE, 0>>;
using DMA2Def  = Periph<0x4002'0400, int, HWRCCBitBand<RCC_AHB1RSTR_BASE, 1>>;
using DMA2DDef = Periph<0x4002'B000, int, HWRCCBitBand<RCC_AHB1RSTR_BASE, 17>>;

using FLASHDef = Periph<0x4002'2000, int, HWRCCBitBand<RCC_AHB1RSTR_BASE, 8>>;
using CRCDef   = Periph<0x4002'3000, int, HWRCCBitBand<RCC_AHB1RSTR_BASE, 12>>;

using RNGDef   = Periph<0x5006'0800, int, HWRCCBitBand<RCC_AHB2RSTR_BASE, 18>>;
using ADCDef   = Periph<0x5004'0000, ADCStruct, HWRCCBitBand<RCC_AHB2RSTR_BASE, 13>>;
using PWRDef   = Periph<0x4000'7000, PWRStruct, HWRCCBitBand<RCC_APB1RSTR1_BASE, 28>>;
using DAC1Def  = Periph<0x4000'7400, int, HWRCCBitBand<RCC_APB1RSTR1_BASE, 29>>;
using CRSDef   = Periph<0x4000'6000, int, HWRCCBitBand<RCC_APB1RSTR1_BASE, 24>>;
using OPAMPDef = Periph<0x4000'7800, int, HWRCCBitBand<RCC_APB1RSTR1_BASE, 30>>;

using SYSCFGDef = Periph<0x4001'0000, int, HWRCCBitBand<RCC_APB2RSTR_BASE, 0>>;

using USART1Def  = GPIOPeriph<0x4001'3800, UARTStruct, IRQn<37>, HWRCCBitBand<RCC_APB2RSTR_BASE, 14>, pinmap::USART1AF>;
using USART2Def  = GPIOPeriph<0x4000'4400, UARTStruct, IRQn<38>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 17>, pinmap::USART2AF>;
using USART3Def  = GPIOPeriph<0x4000'4800, UARTStruct, IRQn<39>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 18>, pinmap::USART3AF>;
using UART4Def   = GPIOPeriph<0x4000'4C00, UARTStruct, IRQn<52>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 19>, pinmap::UART4AF>;
using UART5Def   = GPIOPeriph<0x4000'5000, UARTStruct, IRQn<53>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 20>, pinmap::UART5AF>;
using LPUART1Def = GPIOPeriph<0x4000'8000, UARTStruct, IRQn<70>, HWRCCBitBand<RCC_APB1RSTR2_BASE, 0>, pinmap::LPUART1AF>;

using TSCDef  = GPIOPeriph<0x4002'4000, int, IRQn<77>, HWRCCBitBand<RCC_AHB1RSTR_BASE, 16>, pinmap::AFStub>;
using QSPIDef = GPIOPeriph<0xA000'1000, int, IRQn<71>, HWRCCBitBand<RCC_AHB3RSTR_BASE, 8>, pinmap::AFStub>;

using SPI1Def = GPIOPeriph<0x4001'3000, SPIStruct, IRQn<35>, HWRCCBitBand<RCC_APB2RSTR_BASE, 12>, pinmap::SPI1AF>;
using SPI2Def = GPIOPeriph<0x4000'3800, SPIStruct, IRQn<36>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 14>, pinmap::SPI2AF>;
using SPI3Def = GPIOPeriph<0x4000'3C00, SPIStruct, IRQn<51>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 15>, pinmap::SPI3AF>;

using I2C1Def = GPIOPeriph<0x4000'5400, I2CStruct, IRQn<31>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 21>, pinmap::I2C1AF>;
using I2C2Def = GPIOPeriph<0x4000'5800, I2CStruct, IRQn<33>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 22>, pinmap::I2C2AF>;
using I2C3Def = GPIOPeriph<0x4000'5C00, I2CStruct, IRQn<72>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 23>, pinmap::I2C3AF>;
using I2C4Def = GPIOPeriph<0x4000'8400, I2CStruct, IRQn<83>, HWRCCBitBand<RCC_APB1RSTR2_BASE, 1>, pinmap::I2C4AF>;

using CAN1Def = GPIOPeriph<0x4000'6400, CANStruct, CanIRQn<19>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 25>, pinmap::CAN1AF>;
using CAN2Def = GPIOPeriph<0x4000'6800, CANStruct, CanIRQn<86>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 26>, pinmap::CAN2AF>;

using LPTIM1Def = GPIOPeriph<0x4000'7C00, int, IRQn<65>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 31>, pinmap::AFStub>;
using LPTIM2Def = GPIOPeriph<0x4000'9400, int, IRQn<66>, HWRCCBitBand<RCC_APB1RSTR2_BASE, 5>, pinmap::AFStub>;

using TIM1Def  = GPIOPeriph<0x4001'2C00, TIMGeneralPurposeStruct, IRQn<27>, HWRCCBitBand<RCC_APB2RSTR_BASE, 11>, pinmap::TIM1AF>;
using TIM2Def  = GPIOPeriph<0x4000'0000, TIMGeneralPurposeStruct, IRQn<28>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 0>, pinmap::TIM2AF>;
using TIM3Def  = GPIOPeriph<0x4000'0400, TIMGeneralPurposeStruct, IRQn<29>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 1>, pinmap::TIM3AF>;
using TIM4Def  = GPIOPeriph<0x4000'0800, TIMGeneralPurposeStruct, IRQn<30>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 2>, pinmap::TIM4AF>;
using TIM5Def  = GPIOPeriph<0x4000'0C00, int, IRQn<50>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 3>, pinmap::AFStub>;
using TIM6Def  = GPIOPeriph<0x4000'1000, BasicTimerStruct, IRQn<54>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 4>, pinmap::AFStub>;
using TIM7Def  = GPIOPeriph<0x4000'1400, BasicTimerStruct, IRQn<55>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 5>, pinmap::AFStub>;
using TIM8Def  = GPIOPeriph<0x4001'3400, int, IRQn<43>, HWRCCBitBand<RCC_APB2RSTR_BASE, 13>, pinmap::AFStub>;
using TIM15Def = GPIOPeriph<0x4001'4000, int, IRQn<24>, HWRCCBitBand<RCC_APB2RSTR_BASE, 16>, pinmap::AFStub>;
using TIM16Def = GPIOPeriph<0x4001'4400, int, IRQn<25>, HWRCCBitBand<RCC_APB2RSTR_BASE, 17>, pinmap::AFStub>;
using TIM17Def = GPIOPeriph<0x4001'4800, int, IRQn<26>, HWRCCBitBand<RCC_APB2RSTR_BASE, 18>, pinmap::AFStub>;

using SWPMI1Def = GPIOPeriph<0x4000'8800, int, IRQn<76>, HWRCCBitBand<RCC_APB1RSTR2_BASE, 2>, pinmap::AFStub>;

using SAI1Def = GPIOPeriph<0x4001'5400, int, IRQn<74>, HWRCCBitBand<RCC_APB2RSTR_BASE, 21>, pinmap::AFStub>;
using SAI2Def = GPIOPeriph<0x4001'5800, int, IRQn<75>, HWRCCBitBand<RCC_APB2RSTR_BASE, 22>, pinmap::AFStub>;

using OTGFSDef  = GPIOPeriph<0x5000'0000, int, IRQn<67>, HWRCCBitBand<RCC_AHB2RSTR_BASE, 12>, pinmap::AFStub>;
using DCMIDef   = GPIOPeriph<0x5005'0000, int, IRQn<85>, HWRCCBitBand<RCC_AHB2RSTR_BASE, 14>, pinmap::AFStub>;
using LCDDef    = GPIOPeriph<0x4000'2400, int, IRQn<78>, HWRCCBitBand<RCC_APB1RSTR1_BASE, 9>, pinmap::AFStub>;
using DFSDM1Def = GPIOPeriph<0x4001'6000, int, IRQn<62>, HWRCCBitBand<RCC_APB2RSTR_BASE, 24>, pinmap::AFStub>;
using SDMMCDef  = GPIOPeriph<0x4001'2800, int, IRQn<49>, HWRCCBitBand<RCC_APB2RSTR_BASE, 10>, pinmap::AFStub>;

inline RCCStruct&  rcc   = *reinterpret_cast<RCCStruct*>(0x4002'1000);
inline GPIOStruct* gpios = GPIOADef::periphStruct;
inline PWRStruct&  pwr   = *PWRDef::periphStruct;
inline EXTIStruct& exti  = *reinterpret_cast<EXTIStruct*>(0x4001'0400);
inline SYSCFGStruct& syscfg = *reinterpret_cast<SYSCFGStruct*>(0x4001'0000);
inline DBGStruct& dbg = *reinterpret_cast<DBGStruct*>(0xE000EDF0);
inline DWTStruct&  dwt = *reinterpret_cast<DWTStruct*>(0xE000'1000);
inline DBGMCUStruct& dbgmcu = *reinterpret_cast<DBGMCUStruct*>(0xE004'2004);
inline UIDStruct& uid = *reinterpret_cast<UIDStruct*>(0x1FFF'7590);
inline FlashStruct& flash = *reinterpret_cast<FlashStruct*>(0x4002'2000);

}
