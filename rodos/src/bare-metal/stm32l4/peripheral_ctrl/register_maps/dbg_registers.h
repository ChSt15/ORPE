#pragma once
#include "cortex_m/register_access/register.h"

namespace RODOS{

namespace DEMCR {
    using VC_CORERESET = RegSubValue<0>;
    using VC_MMERR = RegSubValue<4>;   
    using VC_NOCPERR = RegSubValue<5>;
    using VC_CHKERR = RegSubValue<6>;
    using VC_STATERR = RegSubValue<7>;
    using VC_BUSERR = RegSubValue<8>;
    using VC_INTERR = RegSubValue<9>;
    using VC_HARDERR = RegSubValue<10>;
    using MON_EN = RegSubValue<16>;
    using MON_PEND = RegSubValue<17>;
    using MON_STEP = RegSubValue<18>;
    using MON_REQ = RegSubValue<19>;
    using TRCENA = RegSubValue<24>;

}

struct DBGStruct {
    Register DHCSR;
    Register DCRSR;
    Register DCRDR;
    Register DEMCR;
};

static_assert(sizeof(DBGStruct)==0x10);
}
