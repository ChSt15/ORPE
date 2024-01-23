#include "hw/context_t.h"
namespace RODOS {

/**
 * Global variable holding the pointer to the Stack of the current Thread
 */
constinit volatile long* contextT __attribute__ ((used)) { };

/*
 * SVC_Handler is only used to start the first thread
 * - We do this procedure to load rest of context of first thread (idleThread) and switch to PSP
 * -> PSP (Process Stack Pointer) is used in threads
 * -> MSP (Main Stack Pointer) is used in exception handlers (so in scheduler)
 * - MSP is the default stack pointer after reset
 */
extern "C" void SVC_Handler(void)  __attribute__ (( naked ));
extern "C" void SVC_Handler(void) {
    asm volatile (
        "ldr r1, =contextT         \n"// after start "contextT" holds the stack pointer of first thread
        "ldr r0, [r1]              \n"// load stack pointer of first thread (idleThread)
        "ldmia r0!, {r4-r11, r14}  \n"// Pop the registers that are not automatically saved on exception entry and the critical nesting count.
        "msr psp, r0               \n"// load stack pointer of first thread to PSP -> PSP is used after leaving ISR
        "bx r14                    \n"
        "                          \n"
        ".align 2                  \n");
}

/* When the ISR is entered the following registered are saved automatically:
 * FPSCR, S15 ... S0, xPSR, PC, LR, R12, R3, R2, R1, R0
 * -> the other registers (R11 ... R4, S31 ... S16) must be saved and restored manually
 * -> very helpful document: "PM0214 Programming manual - STM32L4xxx Cortex-M4 programming manual" page 42
 */
extern "C" void PendSV_Handler(void)  __attribute__ (( naked ));
extern "C" void PendSV_Handler(void) {
    asm volatile
    (
        "cpsid f                       \n" // disable all interrupts
        "mrs r0, psp                   \n"// load stack pointer of interrupted thread in r0 -> r0 is used to save all not automatically saved core and fpu registers
        "                              \n"
        "tst r14, #0x10                \n" // Is the task using the FPU context?  If so, push high vfp registers.
        "it eq                         \n"
        "vstmdbeq r0!, {s16-s31}       \n"// save FPU registers on stack of interrupted thread
        "                              \n"
        "stmdb r0!, {r4-r11, r14}      \n" // save core registers on stack of interrupted thread
        "                              \n"
        "bl schedulerWrapper           \n"
        "                              \n"
        "ldr r1, =contextT             \n"// "contextT" holds the stack pointer of next thread
        "ldr r0, [r1]                  \n"// load stack pointer of next thread
        "                              \n"
        "ldmia r0!, {r4-r11, r14}      \n"// Pop the core registers
        "                              \n"
        "tst r14, #0x10                \n" // Is the task using the FPU context?  If so, pop the high vfp registers too
        "it eq                         \n"
        "vldmiaeq r0!, {s16-s31}       \n"
        "                              \n"
        "msr psp, r0                   \n" // load psp with stack pointer of next thread -> PSP (process stack pointer) is used after leaving ISR
        "cpsie f                       \n"// enable all interrupts
        "bx r14                        \n"
        "                              \n"
        ".align 2                      \n"
        ".ltorg                        \n");
}

}
