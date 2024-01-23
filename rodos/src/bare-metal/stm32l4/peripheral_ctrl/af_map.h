#pragma once

#include "aborting_error.h"
#include "gpio/enums.h"

namespace RODOS {

template <uint8_t pin, GPIO::ALTERNATEFUNCTION alternateFunction>
class PinAF {
  public:
    static constexpr uint8_t                 PIN                = pin;
    static constexpr GPIO::ALTERNATEFUNCTION ALTERNATE_FUNCTION = alternateFunction;
};

template <typename... AFs>
class AFMap {
    static constexpr uint8_t SIZE           = sizeof...(AFs);
    static constexpr uint8_t TABLE[2][SIZE] = { { AFs::PIN... }, { (uint8_t)AFs::ALTERNATE_FUNCTION... } };

  public:
    static constexpr GPIO::ALTERNATEFUNCTION getAlternateFunction(uint8_t pin) {
        for(int i = 0; i < SIZE; i++) {
            if(TABLE[0][i] == pin) {
                return (GPIO::ALTERNATEFUNCTION)TABLE[1][i];
            }
        }
        abortingError("invalid PIN configuration");
    }
};

}
