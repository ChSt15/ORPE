#pragma once

#include "bit_utils.h"
#include "register.h"

namespace RODOS {

/**
 * class meant for registers that are divided into multiple
 * equally sized sections, where each section has a similar meaning
 * allows identical modifications to multiple such sections by setting a mask
 */
template <uint32_t VALUE_LENGTH>
class MaskedRegister : public Register {
    static_assert(VALUE_LENGTH>0&&VALUE_LENGTH<=32);
    static_assert((32%VALUE_LENGTH)==0);

    static constexpr uint32_t REG_MASK = (1ul << VALUE_LENGTH) - 1;

    static uint8_t findNextBitPos(uint32_t& bitMask)
    {
        uint8_t pos = trailingZeros(bitMask);
        bitMask &= ~(0x1ul << pos);
        return pos;
    }

public:

    inline void setMasked(uint32_t bitMask, uint32_t value)
    {
        while(bitMask!=0){
            uint8_t pos = findNextBitPos(bitMask);
            clearAndSetBits(REG_MASK<<(VALUE_LENGTH*pos), value<<(VALUE_LENGTH*pos));
        }
    }
};

template<>
inline void MaskedRegister<1>::setMasked(uint32_t bitMask, uint32_t value)
{
    clearAndSetBits(bitMask, value*bitMask);
}


}
