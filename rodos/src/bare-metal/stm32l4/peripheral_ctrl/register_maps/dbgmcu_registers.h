#pragma once
#include "cortex_m/register_access/register.h"

namespace RODOS{

namespace DBGMCU_CR {
    using DBG_SLEEP = RegSubValue<0>;
    using DBG_STOP = RegSubValue<1>;
    using DBG_STANDBY = RegSubValue<2>;
    using TRACE_IOEN = RegSubValue<5>;
    using TRACE_MODE = RegSubValue<6>;
}

struct DBGMCUStruct {
    Register CR;
    Register APB1FZR1;
    Register APB1FZR2;
    Register APB2FZR;
};
static_assert(sizeof(DBGMCUStruct)==0x10);
}
