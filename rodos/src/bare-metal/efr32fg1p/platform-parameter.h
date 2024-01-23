
/**
 * @file platform-parameter.h
 * @date 2019/08/20
 * @author Marcelo Alves, Jorge Oliveira
 *
 * @brief configuration parameters ...
 *
 */

#pragma once

#include <stdint.h>
// ___________________  only defines, not in namespace

/** Version Number */
#define OSVERSION "EFR32FG1P Cortex-M4 V1.0"

/*************** System Configuration *********/

/** Memory for allocations, used for all stacks */
#undef XMALLOC_SIZE
#define XMALLOC_SIZE 4 * 1024

/** Default stack size (in bytes) for threads */
#undef DEFAULT_STACKSIZE
#define DEFAULT_STACKSIZE 2000

/** Default stack size (in bytes) for the scheduler */
#undef SCHEDULER_STACKSIZE
#define SCHEDULER_STACKSIZE 8

/** Time interval between timer interrupts in microseconds */
#undef PARAM_TIMER_INTERVAL
#define PARAM_TIMER_INTERVAL 1000


/** Default priority for newly created threads */
#undef DEFAULT_THREAD_PRIORITY
#define DEFAULT_THREAD_PRIORITY 100

/** User threads shall not have a priority higher than this */
#undef MAX_THREAD_PRIORITY
#define MAX_THREAD_PRIORITY 1000

/** High priority levels for priority ceiling */
#undef NETWORKREADER_PRIORITY
#define NETWORKREADER_PRIORITY (MAX_THREAD_PRIORITY + 1)

#undef CEILING_PRIORITY
#define CEILING_PRIORITY (NETWORKREADER_PRIORITY + 1)

/** Using a network, the maximal number of nodes attached */
#undef MAX_NUMBER_OF_NODES
#define MAX_NUMBER_OF_NODES 20

/** If using network it may have a limit for packets, eg. UDP */
#undef MAX_NETWORK_MESSAGE_LENGTH
#define MAX_NETWORK_MESSAGE_LENGTH 1536

/** We can onlx buffer a few UDP packets on the gecko */
#undef UDP_INCOMMIG_BUF_LEN          
#define UDP_INCOMMIG_BUF_LEN         25 //number of "NetworkMessage (1300 Bytes each)" in FIFO

//______________________________________________________________________________________________


#ifdef EFR32FG12P433F1024GM68
#define UART_DEBUG                              UART_IDX4,GPIO_028,GPIO_029
#else
#define UART_DEBUG                              UART_IDX0,GPIO_000,GPIO_001
//#define UART_DEBUG                              UART_IDX1,GPIO_042,GPIO_043 // EGSE TX RX Pins
#endif

#define TIMER_IRQ_PRIO                          0
#define RAIL_LIB_IRQ_PRIO                       1
#define SYSTICK_IRQ_PRIO                        7
#define PENDSV_IRQ_PRIO                         7

/***************************** Radio communicatio  parameter ************************************/

// WARNING!! All time intervals for RAIL  (GECKO lib) are in Microseconds !!!!
// =========

// Time interval (in microseconds) during which a single device (Master or Slave) is allowed to transmit DATA.
// This value must be able to accommodate the size of the DATA packet (see MAX_NETWORK_MESSAGE_LENGTH).
static const uint32_t DATA_TIMESLOT_INTERVAL = 7400;

// Time interval (in microseconds) during which a Master is allowed to transmit a POLL.
// This value must be able to accommodate the size of the POLL packet (see MAX_NUMBER_SLAVES).
static const uint32_t POLL_TIMESLOT_INTERVAL = 500;

// The master stops his packet reception mode a few microsends before the next poll is transmitted.
// used in HAL_RADIO_MASTER::listen(), in hal_radio_mstrslv.cpp
static const uint32_t LISTEN_INTERVAL = 500;

// The maximum number of Slaves. A slave’s ID must be between 0 and MAX_NUMBER_SLAVES-1.
static const uint32_t MAX_NUMBER_SLAVES = 50;

// The number of Slaves the Master polls by default. Can be changed in runtime using HAL_MASTER::config()
// Note: The value of this macro must not be higher than MAX_NUMBER_SLAVES.
static const uint32_t INIT_NUMBER_SLAVES = 5;

// Size of internal TX and RX FIFOs in bytes
static const int32_t TX_FIFO_SIZE = 2048;
static const int32_t RX_FIFO_SIZE = 4096;

// Length of RAIL packet header, in bytes, contains only the length
static const uint16_t PACKET_HEADER_LEN = 2;

