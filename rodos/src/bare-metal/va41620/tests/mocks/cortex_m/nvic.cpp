#include "cortex_m/nvic.h"
#include "nvic_mock.h"

namespace RODOS {

void NVIC::enableIRQ(uint8_t irq){
    testing::globalNVICMock->enableIRQ(irq);
}

void NVIC::disableIRQ(uint8_t irq){
    testing::globalNVICMock->disableIRQ(irq);
}

void NVIC::clearPendingIRQ(uint8_t irq){
    testing::globalNVICMock->clearPendingIRQ(irq);
}

void NVIC::setIRQPriority(uint8_t irq, uint8_t priority){
    testing::globalNVICMock->setIRQPriority(irq, priority);
}

void NVIC::setSystemHandlerPriority(SYSTEM_HANDLER irq, uint8_t priority){
    testing::globalNVICMock->setSystemHandlerPriority(irq, priority);
}

void NVIC::systemReset(){
    testing::globalNVICMock->systemReset();
}

void NVIC::setAllIrqsToDefaultPriority(){
    testing::globalNVICMock->setAllIrqsToDefaultPriority();
}


}
