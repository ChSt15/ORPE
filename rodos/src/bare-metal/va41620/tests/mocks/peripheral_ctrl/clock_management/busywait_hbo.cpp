#include "peripheral_ctrl/clock_management/busywait_hbo.h"
#include "peripheral_ctrl/clock_management/hbo_busywait_mock.h"
#include "peripheral_ctrl/clock_management/clkgen_setup.h"

namespace RODOS {

HboBusywaitMock* globalHboBusywaitMock{nullptr};

void waitMicrosecHBO(uint32_t microseconds){
    globalHboBusywaitMock->waitMicroseconds(microseconds);
}
}
