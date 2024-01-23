/**
 * @file hw_specific.cpp
 * @date 2008/04/23 7:33
 * @author Marcelo Alves, Jorge Oliveira
 *
 * Uni Wuerzburg
 *
 * @brief all hardware specific stuff
 *
 */

#include <math.h>

#include "rodos.h"
#include "platform-parameter.h"
#include "hw_specific.h"
#include "vendor-headers.h"

#include "bsp.h"
#include "rail-headers.h"

/* Constants required to set up the initial stack. */
#define INITIAL_XPSR                    ( 0x01000000 )
#define INITIAL_EXEC_RETURN             ( 0xfffffffd )  // Return to Thread mode using Process Stack Pointer

volatile long *contextT;

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

HAL_UART uart_stdout(UART_DEBUG);
extern long myNodeNr;

/** Responsible for hardware initialization */
void hwInit (void)
{
    //CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;
    //hfxoInit.ctuneSteadyState = 355;

    schedulerRunning = false;

    /* Make PendSV, SysTick and Timer0 (set in TIMER0_init) the same priroity */
    NVIC_SetPriority(PendSV_IRQn, PENDSV_IRQ_PRIO);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_IRQ_PRIO);
    /* Update SystemCoreClock (global variable that contains the system frequency) */
    SystemCoreClockUpdate();
    /* Initialize the chip */
    halInit();

    // Determine the Node Number from the first bytes of the chips UID
    // Note: the chip unique ID is 64bits. By truncating the number, it may no longer be unique.
    myNodeNr = static_cast<int32_t>(SYSTEM_GetUnique() & 0xFFFFFFFF);

    /* Initialize Timer for system time */
    /*
    CMU_HFXOInit(&hfxoInit);
    SystemHFXOClockSet(38400000UL);
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
    */
    uart_stdout.init(115200);
    PRINTF_CONDITIONAL(-1,"\033[H\033[J");
    //uart_stdout.config(UART_PARAMETER_ENABLE_DMA, 1);	// use UART with DMA

	return;
}

/** Create context on stack and return a pointer to it 
 *
 *  low registers:		R0 to R7
 *  high registers: 	R8 to R12
 *                      R13 (Stack Pointer)
 *                      R14 (Link Register)
 *                      R15 (Program Counter)
 *                      xPSR (Program Status Register)
 * 
 * long* stack = start of the thread's stack
 * void* object = thread's object pointer 
 * 
 * */
long* hwInitContext(long* stack, void* object) 
{	
    // Simulates what is done by HW automatically.	
    stack--; 
    *stack = INITIAL_XPSR;                          // xPSR
    stack--;
    *stack = (long) (threadStartupWrapper); 		// PC
    stack--;
    *stack = 0;                                     // LR
    stack -= 5;                                     // R12, R3, R2 and R1
    *stack = (long) object;                         // R0
        
	// These additional registers are also saved - see PendSV_Handler
	stack--;
    *stack = static_cast<long>INITIAL_EXEC_RETURN;					// this value is loaded to R14 (LR)
    stack -= 8;                                     // R11, R10, R9, R8, R7, R6, R5 and R4

	return stack;
}


/** Reset and reboot */
void hwResetAndReboot (void)
{
	NVIC_SystemReset();
	//hwInitWatchdog(0);
	return;
}

/**
  *  global interrupt disable
  */
void hwDisableInterrupts(){
    __disable_irq();
}

/**
  *  global interrupt enable
  */
void hwEnableInterrupts(){
    __enable_irq();
}


/*	Special for use with Host OS
 *	Always a stub when dealing with bare-metal targets
 
void sp_partition_yield (void)
{
	return;
}*/

/** Called to start Idle Thread */
void startIdleThread() __attribute__ (( naked ));
void startIdleThread() {
	__asm volatile(
			" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. Load Vector table offset register address*/
			" ldr r0, [r0] 			\n"	/* Load VTOR content (offset of the table base from the bottom of the memory map -> start of the stack) */
			" ldr r0, [r0] 			\n"	/* make R0 pointer to start of the stack */
			" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
			" cpsie i				\n" /* Globally enable interrupts. */
			" svc 0					\n" /* System call to start first task. */
			" nop					\n"
	);
}

extern "C" {
// - on interrupt the cortex m4 saves automatically the context of the interrupted function
// - it saves the registers xPSR,PC,LR,r12,r3,r2,r1,r0 on stack
// - that is why it's not necessary that the compiler adds code to save the context, so
//   we have to add the attribute "naked" to avoid this

/*
 * SVC_Handler is only used to start the first thread
 * - We do this procedure to load rest of context of first thread (idleThread) and switch to PSP
 * -> PSP (Process Stack Pointer) is used in threads
 * -> MSP (Main Stack Pointer) is used in exception handlers (so in scheduler)
 * - MSP is the default stack pointer after reset
 */
void SVC_Handler(void) __attribute__ (( naked ));
void SVC_Handler(void) 
{
	__asm volatile (
			"	ldr	r1, =contextT				\n" // after start "contextT" holds the stack pointer of first thread
			"	ldr r0, [r1]					\n"// load stack pointer of first thread (idleThread)
			"	ldmia r0!, {r4-r11, r14}		\n"// Pop the registers that are not automatically saved on exception entry and the critical nesting count.
			"	msr psp, r0						\n"// load stack pointer of first thread to PSP -> PSP is used after leaving ISR
			"	bx r14							\n"
			"									\n"
			"	.align 2						\n"
	);
}

/* When the ISR is entered the following registered are saved automatically:
 * FPSCR, S15 ... S0, xPSR, PC, LR, R12, R3, R2, R1, R0
 * -> the other registers (R11 ... R4, S31 ... S16) must be saved and restored manually
 * -> very helpful document: "PM0214 Programming manual - STM32F3xxx and STM32F4xxx Cortex-M4 programming manual" page 42
 */
void PendSV_Handler(void) __attribute__ (( naked ));
void PendSV_Handler(void) 
{
	__asm volatile
	(
			"	cpsid f								\n" // disable all interrupts
			"	mrs r0, psp							\n"// load stack pointer of interrupted thread in r0 -> r0 is used to save all not automatically saved core and fpu registers
			"										\n"
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
			"	tst r14, #0x10						\n" // Is the task using the FPU context?  If so, push high vfp registers.
			"	it eq								\n"
			"	vstmdbeq r0!, {s16-s31}				\n"// save FPU registers on stack of interrupted thread
#endif
			"										\n"
			"	stmdb r0!, {r4-r11, r14}			\n" // save core registers on stack of interrupted thread
			"										\n"
			"	bl schedulerWrapper					\n"
			"										\n"
			"	ldr r1, =contextT					\n"// "contextT" holds the stack pointer of next thread
			"	ldr r0, [r1]						\n"// load stack pointer of next thread
			"										\n"
			"	ldmia r0!, {r4-r11, r14}			\n"// Pop the core registers
			"										\n"
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
			"	tst r14, #0x10						\n" // Is the task using the FPU context?  If so, pop the high vfp registers too
			"	it eq								\n"
			"	vldmiaeq r0!, {s16-s31}				\n"
#endif
			"										\n"
			"	msr psp, r0							\n" // load psp with stack pointer of next thread -> PSP (process stack pointer) is used after leaving ISR
			"	cpsie f								\n"// enable all interrupts
			"	bx r14								\n"
			"										\n"
			"	.align 2							\n"
	);
}

void __asmSwitchToContext(long* context) 
{
	contextT = context;
}

void __asmSaveContextAndCallScheduler() 
{
	/* Set a PendSV-interrupt to request a context switch. */
	SCB ->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


int log2(int x)	//logarithm base 2
{
  int ans = 0;
  while(x>>=1) ans++;
  return ans;
}

} // end extern "C"

/*	Initialize and start HW-watchdog
 * 		- it is used the Watchdog Timer associated with the ocillator ULFRCO
 * 			- 1kHz
 *	Watchdog has fixed clock periods to be selected so: 
 * 		- Min period = 9ms
 *		- Max period = 262145ms
 * 	intervalMilliseconds won't be the actaul period time of the wdog
 */
void hwInitWatchdog(long intervalMilliseconds) {
    if (intervalMilliseconds < 0) {
        intervalMilliseconds = 0;
    }
  	int persel = 0;

  	// Enabling clock to the interface of the low energy modules (including the Watchdog)
  	CMU_ClockEnable(cmuClock_HFLE, true);

  	// Watchdog Initialize settings 
 	WDOG_Init_TypeDef wdogInit;

	wdogInit.clkSel 		= wdogClkSelULFRCO;			// Select 1kHZ WDOG oscillator
	wdogInit.debugRun		= false;					// WDOG not counting during debug halt
	wdogInit.em2Run			= true;						// WDOG counting when in EM2
	wdogInit.em3Run			= true;						// WDOG counting when in EM3
	wdogInit.em4Block		= false;					// EM4 can be entered
	wdogInit.enable 		= true;						// Start watchdog when init done
	wdogInit.lock 			= false;					// Do not lock WDOG configuration (if locked, reset needed to unlock)
 	wdogInit.resetDisable 	= false; 					// Do not disable reset
 	wdogInit.swoscBlock 	= false;            		// Do not block disabling LFRCO/LFXO in CMU
 	wdogInit.warnSel		= wdogWarnDisable;			// Disable warning interrupt
	wdogInit.winSel			= wdogIllegalWindowDisable;	// Disable illegal window interrupt

	//timeOut = (2^(3+persel) + 1)/freq
	persel = log2(intervalMilliseconds - 1) - 3;
	if(persel < 0) persel = 0x0;
	if(persel > 0xf) persel = 0xf;
	wdogInit.perSel 		= (WDOG_PeriodSel_TypeDef) persel; //selects one of the the periods (check em_wdog.h)

  	// Initializing watchdog with chosen settings 
  	WDOG_Init(&wdogInit);	
}

/*
 * trigger HW-watchdog -> must be done periodically with less than "intervalMilliseconds" ms
 */
void hwTriggerWatchdog() 
{
	//Feed the watchdog before it reaches the defined timeout period 
	WDOG_Feed();
}

#ifndef NO_RODOS_NAMESPACE
}
#endif
