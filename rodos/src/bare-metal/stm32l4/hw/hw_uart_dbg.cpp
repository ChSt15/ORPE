#include "hw_hal/hw_hal_uart.h"
#include "hw/hw_uart_dbg.h"

namespace RODOS {
    HAL_UART uartStdout {UART_DEBUG_IDX, UART_DEBUG_TX, UART_DEBUG_RX};
}
