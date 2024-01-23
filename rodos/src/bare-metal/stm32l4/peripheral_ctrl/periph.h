#pragma once

#include "peripheral_ctrl/gpio/enums.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"


namespace RODOS {

/**
 * represents a peripheral of the STM32L4
 * @tparam periphAddr    Base address of the peripheral. In memory organization chapter of the reference manual
 * @tparam PeriphStruct  A struct that represents the memory layout of the registers of the peripheral
 * @tparam PeriphRCC     A HWPeriphBitBand object representing the bits in the RCC registers needed to enable/reset
 *                      the peripheral. Register description RCC_AHB1RSTR to RCC_APB2ENR in the reference manual
 */
template <uint32_t periphAddr, typename PeriphStruct, typename PeriphRCC>
class Periph {
public:
    using PeriphStructType = PeriphStruct;

    static constexpr uint32_t         BASE_ADDR           = periphAddr;
    static constexpr uint32_t         RCC_PERIPH_RST_ADDR = PeriphRCC::ADDR;
    static inline PeriphStruct* const periphStruct        = reinterpret_cast<PeriphStruct*>(periphAddr);
    static inline HWRCCPeriph* const  rccPeriph           = PeriphRCC::asObject;
};

/**
 * @tparam periphAddr    See Periph
 * @tparam PeriphStruct  See Periph
 * @tparam PeriphRCC     See Periph
 * @tparam IRQn          Object representing the interrupt number of the peripheral
 *                       In the NVIC chapter of the reference manual
 * @tparam AFMap         Map with alternate functions of the pins. See chapter 4 of the datasheet
 */
template <uint32_t periphAddr, typename PeriphStruct, typename IRQn, typename PeriphRCC, typename AFMap>
class GPIOPeriph : public Periph<periphAddr, PeriphStruct, PeriphRCC> {
public:
    using IRQnType = typename IRQn::IRQnType;
    static constexpr GPIO::ALTERNATEFUNCTION getAlternateFunction(uint8_t pin)
    {
        return AFMap::getAlternateFunction(pin);
    }
    static constexpr IRQnType IRQ_N = IRQn();
};


}
