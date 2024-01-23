#pragma once
#include "cortex_m/register_access/register.h"
#include "cortex_m/register_access/reserved_bytes.h"

namespace RODOS {

namespace BasicTimer_CR1 {
    using CEN = RegSubValue<0>;
    using UDIS = RegSubValue<1>;
    using URS = RegSubValue<2>;
    using OPM = RegSubValue<3>;
    using APRE = RegSubValue<7>;
    using UIFREMAP = RegSubValue<11>;
}

namespace BasicTimer_DIER {
    using UIE = RegSubValue<0>;
    using UDE = RegSubValue<8>;
}
namespace BasicTimer_SR {
    using UIF = RegSubValue<0>;
}

struct BasicTimerStruct {
    Register CR1;
    Register CR2;
    ReservedWord RESERVED0;
    Register DIER;
    Register SR;
    Register EGR;
    ReservedWord RESERVED[3];
    Register CNT;
    Register PSC;
    Register ARR;
};
static_assert(sizeof(BasicTimerStruct)==0x30);
}
