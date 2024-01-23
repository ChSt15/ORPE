#include "rodos.h"

#include "uart_router.h"

#define UART_BAUDRATE 115200

HAL_UART uart2(UART_IDX2);
HAL_UART uart5(UART_IDX5);
HAL_UART uart4(UART_IDX4);

LinkinterfaceUART linkinterfaceUART2(&uart2, UART_BAUDRATE);
LinkinterfaceUART linkinterfaceUART5(&uart5, UART_BAUDRATE);
LinkinterfaceUART linkinterfaceUART4(&uart4, UART_BAUDRATE);

Gateway gateway1(&linkinterfaceUART2, true);
Gateway gateway2(&linkinterfaceUART5, true);
Gateway gateway3(&linkinterfaceUART4, true);

UartRouter router(false, &gateway1, &gateway2, &gateway3);
