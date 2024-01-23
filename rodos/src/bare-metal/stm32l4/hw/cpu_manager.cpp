#include "cpu_manager.h"
#include "peripheral_ctrl/peripheral_defs.h"

namespace RODOS {

void CPUManager::setupDebugging()
{
    using namespace DBGMCU_CR;
    dbgmcu.CR.set(DBG_SLEEP(1), DBG_STOP(1), DBG_STANDBY(1));
}

uint32_t CPUManager::getFirstUUIDWord(){
    return uid.UID[0].read();
}

}
