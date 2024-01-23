#include "rodos.h"
#include "gateway.h"

HAL_UART          uart2(UART_IDX2);
LinkinterfaceUART linkinterfaceUART2(&uart2, 115200);
Gateway           nameNotImportant(&linkinterfaceUART2, true);

