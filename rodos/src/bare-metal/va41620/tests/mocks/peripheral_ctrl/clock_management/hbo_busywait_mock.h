#pragma once
#include "gmock/gmock.h"

namespace RODOS {

class HboBusywaitMock {
public:
    MOCK_METHOD(void, waitMicroseconds, (uint32_t microseconds));
};

extern HboBusywaitMock* globalHboBusywaitMock;

}
