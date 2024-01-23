#pragma once
#include "gmock/gmock.h"
#include <cstdint>

namespace RODOS {

using ::testing::StrictMock;

template<typename _value_t>
class MMIOMock {
public:
    using value_t = _value_t;

    MOCK_METHOD(void, write, (value_t value));
    MOCK_METHOD(value_t, read, (), (const));
};

using MMIO = StrictMock<MMIOMock<uint32_t>>;
using MMIO8 = StrictMock<MMIOMock<uint8_t>>;
}
