#pragma once

#include "rodos.h"

namespace RODOS {

constexpr GPIO_PIN getDefaultTxPin(const UART_IDX uartIdx)
{
    switch (uartIdx) {
    case UART_IDX0:
        return GPIO_103; //PG7
    case UART_IDX1:
        return GPIO_022; //PB6
    case UART_IDX2:
        return GPIO_053; //PD5
    case UART_IDX3:
        return GPIO_056; //PD8
    case UART_IDX4:
        return GPIO_042; //PC10
    case UART_IDX5:
        return GPIO_044; //PC12
    default:
        return GPIO_INVALID;
    }
}

constexpr GPIO_PIN getDefaultRxPin(const UART_IDX uartIdx)
{
    switch (uartIdx) {
    case UART_IDX0:
        return GPIO_104; //PG8
    case UART_IDX1:
        return GPIO_023; //PB7
    case UART_IDX2:
        return GPIO_054; //PD6
    case UART_IDX3:
        return GPIO_057; //PD9
    case UART_IDX4:
        return GPIO_043; //PC11
    case UART_IDX5:
        return GPIO_050; //PD2
    default:
        return GPIO_INVALID;
    }
}

constexpr GPIO_PIN getDefaultRtsPin(const UART_IDX uartIdx)
{
    switch (uartIdx) {
    case UART_IDX1:
        return GPIO_012; //PA12
    case UART_IDX2:
        return GPIO_052; //PD4
    default:
        return GPIO_INVALID;
    }
}

constexpr GPIO_PIN getDefaultCtsPin(const UART_IDX uartIdx)
{
    switch (uartIdx) {
    case UART_IDX1:
        return GPIO_011; //PA11
    case UART_IDX2:
        return GPIO_051; //PD3
    default:
        return GPIO_INVALID;
    }
}

}
