#pragma once

#include "peripheral_ctrl/af_map.h"

namespace pinmap {
template <char port, uint8_t pin>
static constexpr uint8_t P() {
    return (port - 'A') * 16 + pin;
}

constexpr GPIO::ALTERNATEFUNCTION AF0  = GPIO::ALTERNATEFUNCTION::AF0;
constexpr GPIO::ALTERNATEFUNCTION AF1  = GPIO::ALTERNATEFUNCTION::AF1;
constexpr GPIO::ALTERNATEFUNCTION AF2  = GPIO::ALTERNATEFUNCTION::AF2;
constexpr GPIO::ALTERNATEFUNCTION AF3  = GPIO::ALTERNATEFUNCTION::AF3;
constexpr GPIO::ALTERNATEFUNCTION AF4  = GPIO::ALTERNATEFUNCTION::AF4;
constexpr GPIO::ALTERNATEFUNCTION AF5  = GPIO::ALTERNATEFUNCTION::AF5;
constexpr GPIO::ALTERNATEFUNCTION AF6  = GPIO::ALTERNATEFUNCTION::AF6;
constexpr GPIO::ALTERNATEFUNCTION AF7  = GPIO::ALTERNATEFUNCTION::AF7;
constexpr GPIO::ALTERNATEFUNCTION AF8  = GPIO::ALTERNATEFUNCTION::AF8;
constexpr GPIO::ALTERNATEFUNCTION AF9  = GPIO::ALTERNATEFUNCTION::AF9;
constexpr GPIO::ALTERNATEFUNCTION AF10 = GPIO::ALTERNATEFUNCTION::AF10;
constexpr GPIO::ALTERNATEFUNCTION AF11 = GPIO::ALTERNATEFUNCTION::AF11;
constexpr GPIO::ALTERNATEFUNCTION AF12 = GPIO::ALTERNATEFUNCTION::AF12;
constexpr GPIO::ALTERNATEFUNCTION AF13 = GPIO::ALTERNATEFUNCTION::AF13;
constexpr GPIO::ALTERNATEFUNCTION AF14 = GPIO::ALTERNATEFUNCTION::AF14;
constexpr GPIO::ALTERNATEFUNCTION AF15 = GPIO::ALTERNATEFUNCTION::AF15;

using AFStub = AFMap<PinAF<P<'A', 0>(), AF0>>;

using USART1AF = AFMap<
  PinAF<P<'A', 8>(), AF7>,
  PinAF<P<'A', 9>(), AF7>,
  PinAF<P<'A', 10>(), AF7>,
  PinAF<P<'A', 11>(), AF7>,
  PinAF<P<'A', 12>(), AF7>,
  PinAF<P<'B', 3>(), AF7>,
  PinAF<P<'B', 4>(), AF7>,
  PinAF<P<'B', 5>(), AF7>,
  PinAF<P<'B', 6>(), AF7>,
  PinAF<P<'B', 7>(), AF7>,
  PinAF<P<'G', 9>(), AF7>,
  PinAF<P<'G', 10>(), AF7>,
  PinAF<P<'G', 11>(), AF7>,
  PinAF<P<'G', 12>(), AF7>,
  PinAF<P<'G', 13>(), AF7>>;

using USART2AF = AFMap<
  PinAF<P<'A', 0>(), AF7>,
  PinAF<P<'A', 1>(), AF7>,
  PinAF<P<'A', 2>(), AF7>,
  PinAF<P<'A', 3>(), AF7>,
  PinAF<P<'A', 4>(), AF7>,
  PinAF<P<'D', 3>(), AF7>,
  PinAF<P<'D', 4>(), AF7>,
  PinAF<P<'D', 5>(), AF7>,
  PinAF<P<'D', 6>(), AF7>,
  PinAF<P<'D', 7>(), AF7>,
  PinAF<P<'A', 15>(), AF3>>;

using USART3AF = AFMap<
  PinAF<P<'A', 15>(), AF7>,
  PinAF<P<'B', 1>(), AF7>,
  PinAF<P<'B', 2>(), AF7>,
  PinAF<P<'B', 10>(), AF7>,
  PinAF<P<'B', 11>(), AF7>,
  PinAF<P<'B', 12>(), AF7>,
  PinAF<P<'B', 13>(), AF7>,
  PinAF<P<'B', 14>(), AF7>,
  PinAF<P<'C', 4>(), AF7>,
  PinAF<P<'C', 5>(), AF7>,
  PinAF<P<'C', 10>(), AF7>,
  PinAF<P<'C', 11>(), AF7>,
  PinAF<P<'C', 12>(), AF7>,
  PinAF<P<'D', 2>(), AF7>,
  PinAF<P<'D', 8>(), AF7>,
  PinAF<P<'D', 9>(), AF7>,
  PinAF<P<'D', 10>(), AF7>,
  PinAF<P<'D', 11>(), AF7>,
  PinAF<P<'D', 12>(), AF7>>;

using UART4AF = AFMap<
  PinAF<P<'A', 0>(), AF8>,
  PinAF<P<'A', 1>(), AF8>,
  PinAF<P<'A', 15>(), AF8>,
  PinAF<P<'B', 7>(), AF8>,
  PinAF<P<'C', 10>(), AF8>,
  PinAF<P<'C', 11>(), AF8>>;

using UART5AF = AFMap<
  PinAF<P<'B', 4>(), AF8>,
  PinAF<P<'B', 5>(), AF8>,
  PinAF<P<'C', 12>(), AF8>,
  PinAF<P<'D', 2>(), AF8>>;

using LPUART1AF = AFMap<
  PinAF<P<'A', 2>(), AF8>,
  PinAF<P<'A', 3>(), AF8>,
  PinAF<P<'A', 6>(), AF8>,
  PinAF<P<'B', 1>(), AF8>,
  PinAF<P<'B', 10>(), AF8>,
  PinAF<P<'B', 11>(), AF8>,
  PinAF<P<'B', 12>(), AF8>,
  PinAF<P<'B', 13>(), AF8>,
  PinAF<P<'C', 0>(), AF8>,
  PinAF<P<'C', 1>(), AF8>,
  PinAF<P<'G', 5>(), AF8>,
  PinAF<P<'G', 6>(), AF8>,
  PinAF<P<'G', 7>(), AF8>,
  PinAF<P<'G', 8>(), AF8>>;

using SPI1AF = AFMap<
  PinAF<P<'A', 1>(), AF5>,
  PinAF<P<'A', 4>(), AF5>,
  PinAF<P<'A', 5>(), AF5>,
  PinAF<P<'A', 6>(), AF5>,
  PinAF<P<'A', 7>(), AF5>,
  PinAF<P<'A', 11>(), AF5>,
  PinAF<P<'A', 12>(), AF5>,
  PinAF<P<'A', 15>(), AF5>,
  PinAF<P<'B', 0>(), AF5>,
  PinAF<P<'B', 3>(), AF5>,
  PinAF<P<'B', 4>(), AF5>,
  PinAF<P<'B', 5>(), AF5>,
  PinAF<P<'E', 12>(), AF5>,
  PinAF<P<'E', 13>(), AF5>,
  PinAF<P<'E', 14>(), AF5>,
  PinAF<P<'E', 15>(), AF5>,
  PinAF<P<'G', 2>(), AF5>,
  PinAF<P<'G', 3>(), AF5>,
  PinAF<P<'G', 4>(), AF5>,
  PinAF<P<'G', 5>(), AF5>>;

using SPI2AF = AFMap<
  PinAF<P<'A', 9>(), AF3>,
  PinAF<P<'C', 1>(), AF3>,
  PinAF<P<'B', 9>(), AF5>,
  PinAF<P<'B', 10>(), AF5>,
  PinAF<P<'B', 12>(), AF5>,
  PinAF<P<'B', 13>(), AF5>,
  PinAF<P<'B', 14>(), AF5>,
  PinAF<P<'B', 15>(), AF5>,
  PinAF<P<'C', 2>(), AF5>,
  PinAF<P<'C', 3>(), AF5>,
  PinAF<P<'D', 0>(), AF5>,
  PinAF<P<'D', 1>(), AF5>,
  PinAF<P<'D', 3>(), AF5>,
  PinAF<P<'D', 4>(), AF5>,
  PinAF<P<'I', 0>(), AF5>,
  PinAF<P<'I', 1>(), AF5>,
  PinAF<P<'I', 2>(), AF5>,
  PinAF<P<'I', 3>(), AF5>>;

using SPI3AF = AFMap<
  PinAF<P<'A', 4>(), AF6>,
  PinAF<P<'A', 15>(), AF6>,
  PinAF<P<'B', 3>(), AF6>,
  PinAF<P<'B', 4>(), AF6>,
  PinAF<P<'B', 5>(), AF6>,
  PinAF<P<'C', 10>(), AF6>,
  PinAF<P<'C', 11>(), AF6>,
  PinAF<P<'C', 12>(), AF6>,
  PinAF<P<'G', 9>(), AF6>,
  PinAF<P<'G', 10>(), AF6>,
  PinAF<P<'G', 11>(), AF6>,
  PinAF<P<'G', 12>(), AF6>>;

using CAN1AF = AFMap<
  PinAF<P<'A', 11>(), AF9>,
  PinAF<P<'A', 12>(), AF9>,
  PinAF<P<'B', 8>(), AF9>,
  PinAF<P<'B', 9>(), AF9>,
  PinAF<P<'D', 0>(), AF9>,
  PinAF<P<'D', 1>(), AF9>,
  PinAF<P<'H', 13>(), AF9>,
  PinAF<P<'I', 9>(), AF9>>;

using CAN2AF = AFMap<
  PinAF<P<'B', 5>(), AF3>,
  PinAF<P<'B', 6>(), AF8>,
  PinAF<P<'B', 12>(), AF10>,
  PinAF<P<'B', 13>(), AF10>>;

using I2C1AF = AFMap<
  PinAF<P<'B', 6>(), AF4>,
  PinAF<P<'B', 7>(), AF4>,
  PinAF<P<'B', 8>(), AF4>,
  PinAF<P<'B', 9>(), AF4>,
  PinAF<P<'G', 13>(), AF4>,
  PinAF<P<'G', 14>(), AF4>>;

using I2C2AF = AFMap<
  PinAF<P<'B', 10>(), AF4>,
  PinAF<P<'B', 11>(), AF4>,
  PinAF<P<'B', 13>(), AF4>,
  PinAF<P<'B', 14>(), AF4>,
  PinAF<P<'F', 0>(), AF4>,
  PinAF<P<'F', 1>(), AF4>,
  PinAF<P<'H', 4>(), AF4>,
  PinAF<P<'H', 5>(), AF4>>;
  
using I2C3AF = AFMap<
  PinAF<P<'A', 7>(), AF4>,
  PinAF<P<'B', 4>(), AF4>,
  PinAF<P<'C', 0>(), AF4>,
  PinAF<P<'C', 1>(), AF4>,
  PinAF<P<'C', 9>(), AF6>,
  PinAF<P<'G', 7>(), AF4>,
  PinAF<P<'G', 8>(), AF4>,
  PinAF<P<'H', 7>(), AF4>,
  PinAF<P<'H', 8>(), AF4>>;

using I2C4AF = AFMap<
  PinAF<P<'D', 12>(), AF4>,
  PinAF<P<'D', 13>(), AF4>,
  PinAF<P<'F', 14>(), AF4>,
  PinAF<P<'F', 15>(), AF4>,
  PinAF<P<'C', 9>(), AF6>,
  PinAF<P<'G', 7>(), AF4>,
  PinAF<P<'G', 8>(), AF4>,
  PinAF<P<'H', 7>(), AF4>,
  PinAF<P<'H', 8>(), AF4>>;

using TIM1AF = AFMap<
  PinAF<P<'E', 9>(), AF1>, 
  PinAF<P<'E', 11>(), AF1>, 
  PinAF<P<'E', 13>(), AF1>, 
  PinAF<P<'E', 14>(), AF1>>;

using TIM2AF = AFMap<
  PinAF<P<'A', 0>(), AF1>, 
  PinAF<P<'A', 1>(), AF1>, 
  PinAF<P<'A', 2>(), AF1>, 
  PinAF<P<'A', 3>(), AF1>>;

using TIM3AF = AFMap<
  PinAF<P<'A', 6>(), AF2>, 
  PinAF<P<'A', 7>(), AF2>, 
  PinAF<P<'B', 0>(), AF2>, 
  PinAF<P<'B', 1>(), AF2>>;

using TIM4AF = AFMap<
  PinAF<P<'D', 12>(), AF2>, 
  PinAF<P<'D', 13>(), AF2>, 
  PinAF<P<'D', 14>(), AF2>, 
  PinAF<P<'D', 15>(), AF2>>;

} // namespace pinmap
