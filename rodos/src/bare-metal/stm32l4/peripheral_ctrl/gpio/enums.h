#pragma once

#include "rodos.h"

namespace RODOS {

namespace GPIO {
    enum class MODE : uint8_t {
        INPUT = 0,
        OUTPUT,
        ALTERNATE,
        ANALOG
    };
    enum class OUTPUTTYPE : uint8_t {
        PUSHPULL = 0,
        OPENDRAIN
    };
    enum class SPEED : uint8_t {
        LOW = 0,
        MEDIUM,
        FAST,
        HIGH
    };
    enum class PULL : uint8_t {
        NONE = 0,
        UP,
        DOWN
    };
    enum class ALTERNATEFUNCTION : uint8_t {
        AF0 = 0,
        AF1,
        AF2,
        AF3,
        AF4,
        AF5,
        AF6,
        AF7,
        AF8,
        AF9,
        AF10,
        AF11,
        AF12,
        AF13,
        AF14,
        AF15,
    };

} // namespace GPIO

}
