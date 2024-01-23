#include "flash_manager.h"
#include "peripheral_ctrl/peripheral_defs.h"

namespace RODOS {

void FlashManager::setLatency(uint32_t latency){
    using namespace FLASH_ACR;
    do {
        flash.ACR.set(LATENCY(latency));
    }while(flash.ACR.read<LATENCY>()!=latency);
}

void FlashManager::initCache(){
    using namespace FLASH_ACR;
    flash.ACR.set(PRFTEN(1), ICEN(1), DCEN(1));
}

}
