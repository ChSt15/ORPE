#pragma once
#include "cortex_m/register_access/register.h"
#include "cortex_m/register_access/reserved_bytes.h"

namespace RODOS {
struct EXTIStruct {
    Register IMR1;
    Register EMR1;
    Register RTSR1;
    Register FTSR1;
    Register SWIER1;
    Register PR1; 
    Register IMR2;
    Register EMR2;
    Register RTSR2;
    Register FTSR2;
    Register SWIER2;
    Register PR2;
};

}
