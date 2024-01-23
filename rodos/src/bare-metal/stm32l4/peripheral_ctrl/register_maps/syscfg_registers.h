#pragma once
#include "cortex_m/register_access/register.h"

namespace RODOS {

struct SYSCFGStruct {
    Register MEMRMP;
    Register CFGR1;
    Register EXTICR[4];
    Register SCSR;
    Register CFGR2;
    Register SWPR;
    Register SKR;
    Register SWPR2;
};
static_assert(sizeof(SYSCFGStruct)==0x2C);

}
