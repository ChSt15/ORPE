/*
 * @file hw_timer.cpp
 * @date 2012/08/22
 * @author Michael Ruffer
 *
 * Copyright 2012 University of Wuerzburg
 *
 * @brief Timer for system time and preemption
 */
#include <emlib/inc/em_timer.h>
#include "rodos.h"
#include "hw_specific.h"

#include "platform-parameter.h"

#include "vendor-headers.h"

namespace RODOS {

static uint32_t timerClock = 1;

/***** Preemption Timer - Cortex "SysTick" counter *****
 *
 * Timer to force scheduler calls -> preemption
 * - cortex system counter is used -> "SysTick"
 * - 24Bit counter running at CPU_CLK/8 (19MHz/8 = 2,375MHz)
 * -> max. value: 0xFFF = 2^24 = 16.777.216
 * -> 1ms 	@ 2,375MHz: 2.375			@ 19MHz: 19.000
 * -> 10ms 	@ 2,375MHz: 23.750			@ 19MHz: 190.000	
 * -> 100ms @ 2,375MHz: 237.500			@ 19MHz: 1.900.000
 * -> 1s  	@ 2,375MHz: 2.375.000		@ 19MHz: 19.000.000 -> counter overflow !!!
 * -> 10s  	@ 2,375MHz: 23.750.000 -> counter overflow !!!
 */
extern long long timeToTryAgainToSchedule;
extern bool      isSchedulingEnabled;

extern "C" {

volatile int64_t nanoTime = 0;

/*
 * Interrupt Service Routine for "SysTick" counter
 *
 * !!! Don't use "naked" for the ISR, because the system crashes
 * -> maybe because not all registers are saved automatically
 * -> it works when the compiler puts prologue and epilogue in the ISR
 *    -> but this can be a problem when ISRs can interrupt each other
 * -> this can happen when they don't have the same priority !!!
 */
void SysTick_Handler();
void SysTick_Handler() {

    if(!isSchedulingEnabled) {
        return;
    }

    long long timeNow = NOW();
    TimeEvent::propagate(timeNow); // comment this out to improve performance, but: no time events any more

    if(timeNow < timeToTryAgainToSchedule) {
        return;
    }

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; // set SW-IRQ to call scheduler
}

/** \brief  System Tick Configuration

 This function initialises the system tick timer and its interrupt WITHOUT starting it
 Counter is in free running mode to generate periodical interrupts.

 \param [in]  ticks  Number of ticks between two interrupts
 \return          0  Function succeeded
 \return          1  Function failed
 */
static __INLINE uint32_t SysTick_Config_New(uint32_t ticks) {
    if(ticks > SysTick_LOAD_RELOAD_Msk) {
        return (1); /* Reload value impossible */
    }
    SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;       /* set reload register */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* set Priority for Cortex-M3 System Interrupts */
    SysTick->VAL = 0;                                            /* Load the SysTick Counter Value */
    return (0);                                                  /* Function successful */
}

static __INLINE uint32_t SysTick_IRQEnable(void) {
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    return (0);
}

static __INLINE uint32_t SysTick_IRQDisable(void) {
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);

    return (0);
}

static __INLINE uint32_t SysTick_Enable(void) {
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // if set clock source = CPU_CLK else CPU_CLK/8 (but this does not work on sf2)
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
    return (0);
}

static __INLINE uint32_t SysTick_Disable(void) {
    SysTick->CTRL = 0;

    return (0);
}

} // end extern "C"

/** 
 * the timer interval
 */
long long Timer::microsecondsInterval = PARAM_TIMER_INTERVAL;

void Timer::init() {
    Timer::microsecondsInterval = PARAM_TIMER_INTERVAL;
    timerClock                  = CMU_ClockFreqGet(cmuClock_HF);
    SysTick_Config_New((uint32_t )(((int64_t)timerClock * Timer::microsecondsInterval) / 1000000)); // initialization of systick timer, reload value: 190000-1 -> generates an irq every 10ms with 19MHz sys clock
}

/**
 * start timer
 */
void Timer::start() {
    SysTick_Config_New((uint32_t )(((int64_t)timerClock * Timer::microsecondsInterval) / 1000000)); // initialization of systick timer, reload value: 190000-1 -> generates an irq every 10ms with 19MHz sys clock
    SysTick_Enable();
}

/**
 * stop timer
 */
void Timer::stop() {
    SysTick_IRQDisable(); // only disable the interrupt for task swichting
}

/**
 * set timer interval
 */
void Timer::setInterval(const long long microsecondsInterval) {
    Timer::microsecondsInterval = microsecondsInterval;
}

/***** System Time ******
 *
 * timer is used to generate the system time
 * -> interrupt every 10ms
 * -> time resolution: 1/timerClock = 166,6ns
 * -> max. timerCount = timerClock/100 = 60 000 (IRQ every 10ms)
 */

//extern "C" {
/*
 * Interrupt Service Routine for Timer
 *
 * !!! Don't use "naked" for the ISR, because the system crashes
 * -> maybe because not all registers are saved automatically
 * -> it works when the compiler puts prologue and epilogue in the ISR
 *    -> but this can be a problem when ISRs can interrupt each other
 * -> this can happen when they don't have the same priority !!!
 */

int64_t TICK_SCALING;
uint8_t TICK_SCALING_BITS = 6;

int64_t hwGetNanoseconds(void) {



    uint32_t count      = 0;
    int64_t returnTime = 0;

    // -> current time = nanoTime + 1 000 000 000 * countRegister/tim2Clock

    // Disable Interrupt is no solution here (wrong values caused by missed interrupt)

    // Read nanoTime twice, to make sure it has not changed while reading counter value
    do {
        returnTime = nanoTime;
        count      = TIMER_CounterGet(TIMER0);
    } while(returnTime != nanoTime);

    /**
	 * Berechnung
	 * count ist der Timerwert zwischen den Aufrufen
	 * Alle 10 ms
	 *
	 */
    /** low precision
	 * - nanos = 166ns * count (for tim2Clock = 6MHz)
	 * - nanos = 11ns * count (for tim2Clock = 84MHz)
	 * ! startup time might be wrong due to integer overflow !
	 */
    //int nanos = 1000000000/timerClock * count;
    /** high precision
	 * - nanos = 125ns * count (for tim2Clock = 8MHz)
	 * - nanos = 8ns * count (for tim2Clock = 125MHz)
	 * - takes 4 times longer than low precision
	 */
    //int64_t nanos = ((int64_t)count * 1000000) / (timerClock / 1000);
     int64_t nanos =  (((returnTime << 16) | static_cast<int64_t>(count)) * TICK_SCALING) >> TICK_SCALING_BITS;

    return nanos;
}

void hwInitTime(void) {
    CMU_ClockEnable(cmuClock_TIMER0, true);
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.enable     = true,
    timerInit.debugRun   = true,
    timerInit.prescale   = timerPrescale8,
    timerInit.clkSel     = timerClkSelHFPerClk,
    timerInit.fallAction = timerInputActionNone,
    timerInit.riseAction = timerInputActionNone,
    timerInit.mode       = timerModeUp,
    timerInit.dmaClrAct  = false,
    timerInit.quadModeX4 = false,
    timerInit.oneShot    = false,
    timerInit.sync       = false,
    timerInit.count2x    = false;

    /* Enable overflow interrupt */
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);
    nanoTime = 0;
    NVIC_SetPriority(TIMER0_IRQn, TIMER_IRQ_PRIO);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIMER_TopSet(TIMER0, 0xFFFF);
    TIMER_Init(TIMER0, &timerInit);

    TICK_SCALING = (((1 << timerInit.prescale) * SECONDS) << TICK_SCALING_BITS) / CMU_ClockFreqGet(cmuClock_TIMER0);

    TIMER_Enable(TIMER0,true);
}

int64_t hwGetAbsoluteNanoseconds(void) {
    return hwGetNanoseconds(); // + timeAtStartup;
}

extern "C" {
void TIMER0_IRQHandler(void) {
    if(TIMER_IntGet(TIMER0) == TIMER_IF_OF){
       /* Clear flag for TIMER0 overflow interrupt */
       TIMER_IntClear(TIMER0, TIMER_IF_OF);
       nanoTime++;
    } else {
       /* Something different happened -> ToDo proper error handling */
       TIMER_IntClear(TIMER0, 0xFFFFFFFF & ~TIMER_IF_OF);
    }
}

}
} /* namespace RODOS */
