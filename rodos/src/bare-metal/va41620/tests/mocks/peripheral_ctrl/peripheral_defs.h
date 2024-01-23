#pragma once
#include "peripheral_ctrl/register_maps/adc_registers.h"
#include "peripheral_ctrl/register_maps/can_registers.h"
#include "peripheral_ctrl/register_maps/clkgen_registers.h"
#include "peripheral_ctrl/register_maps/gpio_registers.h"
#include "peripheral_ctrl/register_maps/ioconfig_registers.h"
#include "peripheral_ctrl/register_maps/sysconfig_registers.h"
#include "peripheral_ctrl/register_maps/sysconfig_peripheral_bitband.h"
#include "peripheral_ctrl/register_maps/uart_registers.h"
#include "peripheral_ctrl/register_maps/tim_registers.h"
#include "peripheral_ctrl/register_maps/spi_registers.h"
#include "peripheral_ctrl/register_maps/i2c_registers.h"

namespace RODOS {

extern ADCStruct* adc;
extern SPIStruct* spis;
extern I2CStruct* i2cs;
extern CANStruct* cans;
extern UARTStruct* uart0;
extern UARTStruct* uart1;
extern UARTStruct* uart2;
extern IOCONFIGStruct* ioconfig;
extern TIMStruct* tim0to15;
extern TIMStruct* tim16to23;
extern CLKGENStruct* clkgen;
extern SYSCONFIGStruct* sysconfig;
extern GPIOStruct* gpioStructs;
constexpr size_t numGpioStructs {7};
extern SYSCONFIGPeripheralBitbandStruct* sysconfigPeripheralBitband;

}
