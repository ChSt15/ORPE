#pragma once

#include "rodos.h"

#include "hal/hal_uart.h"
#include "hw_hal_gpio.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"

#include "peripherals.h"

namespace RODOS {

extern "C" {
void USART1_IRQHandler();
void USART2_IRQHandler();
void USART3_IRQHandler();
void UART4_IRQHandler();
void UART5_IRQHandler();
void LPUART1_IRQHandler();
}

class HW_HAL_UART {
private:
    const UART_IDX idx;
    UARTStruct& uart;
    HAL_UART& hal_uart;

    int32_t rxErrCnt;

    GPIO_PIN tx;
    GPIO_PIN rx;
    GPIO_PIN rts;
    GPIO_PIN cts;

    BlockFifo<uint8_t, UART_BUF_SIZE> receiveBuffer;
    BlockFifo<uint8_t, UART_BUF_SIZE> transmitBuffer;

    void configurePin(GPIO_PIN pinIdx);
    void resetPin(GPIO_PIN pinIdx);

    HWRCCPeriph& getRCCPeriph() const;

    constexpr uint32_t getUartDiv(uint32_t baudrate);
    constexpr bool isLPUART();
    constexpr bool supportsHWFlowCtrl();

    void startFifoTX();
    bool isFifoTXRunning();
    void finishFifoTX();

    void resetPeriph();
    void resetHwCtrlPins();
public:
    HW_HAL_UART(HAL_UART& halUart, UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin);
    int32_t init(uint32_t baudrate);
    void reset();

    size_t read(void* buf, size_t size);
    int16_t getcharNoWait();
    size_t write(const void* buf, size_t size);
    int16_t putcharNoWait(uint8_t c);

    int32_t getRXBufLevel();
    int32_t getRXErrorCnt();
    bool isWriteFinished();
    bool isDataReady();
    void putcharFromInterrupt(char c);
    static void putcharFromInterrupt(UART_IDX idx, char c);

    void setBaudrate(uint32_t baudrate);
    int enableHWCtrlFlow();



    static constinit HW_HAL_UART* contextArray[mcu_specific::UARTS::getNumPeriphs()];
    void UARTIRQHandler();
};

}
