#pragma once

#include "peripheral_ctrl/gpio/enums.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"


namespace RODOS {

template <typename Head, typename... Elems>
class HeadOf : public Head {};

template <typename... Periphs>
class PeriphMap {
    using PeriphStructType = typename HeadOf<Periphs...>::PeriphStructType;
    static inline HWRCCPeriph* hwRCCPeriphs[] = { Periphs::rccPeriph... };
    static inline PeriphStructType* periphStructs[] = { Periphs::periphStruct... };

public:
    static inline HWRCCPeriph& getRCCPeriph(uint8_t periphId) {
        return *hwRCCPeriphs[periphId];
    }
    static inline PeriphStructType& getPeriphStruct(uint8_t periphId) {
        return *periphStructs[periphId];
    }
};

template <typename... Periphs>
class GPIOPeriphMap : public PeriphMap<Periphs...> {
    using GetAFFntPtr = GPIO::ALTERNATEFUNCTION (*)(uint8_t);
    static constexpr typename HeadOf<Periphs...>::IRQnType IRQns[] = { Periphs::IRQ_N... };
    static constexpr GetAFFntPtr GetAFFntPtrs[] = { Periphs::getAlternateFunction... };

  public:
    static inline constexpr GPIO::ALTERNATEFUNCTION getAlternateFunction(uint8_t periphId, uint8_t pin) {
        return GetAFFntPtrs[periphId](pin);
    }
    static inline constexpr typename HeadOf<Periphs...>::IRQnType getIRQn(uint8_t periphId) {
        return IRQns[periphId];
    }
    static inline constexpr uint8_t getNumPeriphs() {
        return sizeof...(Periphs);
    }
    static inline constexpr bool exists(uint8_t periphId) {
        return periphId < sizeof...(Periphs);
    }
};


}
