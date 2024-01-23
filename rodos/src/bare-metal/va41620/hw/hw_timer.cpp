#include "hw_specific.h"
#include "cortex_m/systick_manager.h"
#include "cortex_m/nvic.h"

namespace RODOS {

int64_t Timer::microsecondsInterval = 0;

void Timer::init() {
    NVIC::setSystemHandlerPriority(NVIC::SYSTEM_HANDLER::PENDSV, NVIC::lowestPriority);
    NVIC::setSystemHandlerPriority(NVIC::SYSTEM_HANDLER::SYSTICK, NVIC::lowestPriority);
}

/**
* start timer 
*/
void Timer::start() 
{
    SysTickManager::start();
}

/**
* stop timer 
*/
void Timer::stop() 
{
    SysTickManager::stop();
}

/**
* set timer interval 
*/
void Timer::setInterval(const int64_t microsecondsInterval) {
    Timer::microsecondsInterval = microsecondsInterval;
    SysTickManager::setInterval(microsecondsInterval);
}

}
