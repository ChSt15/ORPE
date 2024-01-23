#include "cortex_m/register_access/register.h"

namespace RODOS {

namespace FLASH_ACR {
    using LATENCY = RegSubValue<0,3>;
    using PRFTEN = RegSubValue<8>;
    using ICEN = RegSubValue<9>;
    using DCEN = RegSubValue<10>;
    using ICRST = RegSubValue<11>;
    using DCRST = RegSubValue<12>;
    using RUN_PD = RegSubValue<13>;
    using SLEEP_PD = RegSubValue<14>;
}

struct FlashStruct {
    Register ACR;
    Register PDKEYR;
    Register OPTKEYR;
    Register SR;
    Register CR;
    Register ECCR;
    Register OPTR;
    Register PCROP1SR;
    Register PCROP1ER;
    Register WRP1AR;
    Register WRP1BR;
    Register PCROP2SR;
    Register PCROP2ER;
    Register WRP2AR;
    Register WRP2BR;
};

}
