#include "vcore_range_manager.h"
#include "peripheral_ctrl/peripheral_defs.h"

namespace RODOS {
void VCORERangeManager::init(){
    using namespace PWR_CR1;
    pwr.CR1.set(VOS(voltageRange));
}
}


