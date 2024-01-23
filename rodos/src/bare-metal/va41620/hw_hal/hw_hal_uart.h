#pragma once

#include "fifo.h"
#include "hal/hal_uart.h"
#include "hw_hal_gpio.h"
#include "peripheral_ctrl/register_maps/uart_registers.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "peripheral_ctrl/pin_config/enums.h"

namespace RODOS {

extern "C" {
void UART0_RX_IRQHandler();
void UART0_TX_IRQHandler();
void UART1_RX_IRQHandler();
void UART1_TX_IRQHandler();
void UART2_RX_IRQHandler();
void UART2_TX_IRQHandler();
}

constexpr uint8_t NUM_OF_UARTS = 3;

class VA41620_UART_IDX {
public:
    constexpr VA41620_UART_IDX(UART_IDX idx)
    : m_idx {static_cast<uint8_t>(idx)}
    {
        if(idx>=NUM_OF_UARTS){
            //this will only happen while constructors are running, so there'd be no use in printing a message
            __builtin_trap();
        }
    }

    constexpr operator uint8_t() const{
        return m_idx;
    }
    const uint8_t m_idx;
};

class HW_HAL_UART {
public:
    static constexpr size_t numUarts = 3;
    static constinit HW_HAL_UART* contextArray[3];

    HW_HAL_UART(HAL_UART& halUart, VA41620_UART_IDX uartIdx, pin_config::VA41620_PIN txPin, pin_config::VA41620_PIN rxPin, pin_config::VA41620_PIN rtsPin, pin_config::VA41620_PIN ctsPin);
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
    static void putcharFromInterrupt(VA41620_UART_IDX idx, char c);

    void setBaudrate(uint32_t baudrate);

    void rxIrqHandler();
    void txIrqHandler();
    void UARTIRQHandler();

    static uint8_t getRxIRQn(VA41620_UART_IDX idx);
    static uint8_t getTxIRQn(VA41620_UART_IDX idx);
private:
    const VA41620_UART_IDX m_idx;
    UARTStruct& m_uart;
    HAL_UART& m_hal_uart;

    int32_t m_rxErrCnt;

    pin_config::VA41620_PIN m_tx;
    pin_config::VA41620_PIN m_rx;
    pin_config::VA41620_PIN m_rts;
    pin_config::VA41620_PIN m_cts;

    BlockFifo<uint8_t, UART_BUF_SIZE> m_receiveBuffer;
    BlockFifo<uint8_t, UART_BUF_SIZE> m_transmitBuffer;

    uint8_t getRxIRQn();
    uint8_t getTxIRQn();

    void configurePin(pin_config::VA41620_PIN pinIdx);
    void resetPin(pin_config::VA41620_PIN pinIdx);

    static UARTStruct& getUartStruct(VA41620_UART_IDX idx);
    uint32_t getClockRate();
    uint32_t getClkScale(uint32_t baudrate);

    void startFifoTX();
    bool isFifoTXRunning();
    void finishFifoTX();

    void resetPeriph();
    void resetHwCtrlPins();
};

}
