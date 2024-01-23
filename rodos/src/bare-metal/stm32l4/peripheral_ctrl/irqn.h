#pragma once

#include "stdint.h"

namespace RODOS {

template <uint8_t irqn, typename T = uint8_t>
struct IRQn {
    using IRQnType = T;
    constexpr operator T() const {
        return T(irqn);
    }
};

class CanIRQnStruct {
    const uint8_t CANx_TX;

public:
    constexpr CanIRQnStruct(uint8_t IRQ_TX)
      : CANx_TX(IRQ_TX) {}

    uint8_t getCANx_TX() { return CANx_TX; }
    uint8_t getCANx_RX0() { return static_cast<uint8_t>(CANx_TX + 1u); }
    uint8_t getCANx_RX1() { return static_cast<uint8_t>(CANx_TX + 2u); }
    uint8_t getCANx_SCE() { return static_cast<uint8_t>(CANx_TX + 3u); }
};

template <uint8_t IRQ_TX>
using CanIRQn = IRQn<IRQ_TX, CanIRQnStruct>;

}
