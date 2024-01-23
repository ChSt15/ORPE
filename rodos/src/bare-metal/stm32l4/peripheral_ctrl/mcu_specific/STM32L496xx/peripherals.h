#pragma once

#include "peripheral_ctrl/peripheral_defs.h"
#include "peripheral_ctrl/periph_map.h"

namespace RODOS::mcu_specific {
    using UARTS = GPIOPeriphMap<LPUART1Def, USART1Def, USART2Def, USART3Def, UART4Def, UART5Def>;
    using SPIS = GPIOPeriphMap<SPI1Def, SPI2Def, SPI3Def>;
    using CANS = GPIOPeriphMap<CAN1Def, CAN2Def>;
    using I2CS = GPIOPeriphMap<I2C1Def, I2C2Def, I2C3Def, I2C4Def>;
    using TIMS = GPIOPeriphMap<TIM1Def, TIM2Def, TIM3Def, TIM4Def>;
}
