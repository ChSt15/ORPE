#pragma once

namespace RODOS {

inline uint8_t trailingZeros(uint32_t word){
    return static_cast<uint8_t>(__builtin_ctzl(word));
}

}
