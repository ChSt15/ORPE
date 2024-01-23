#include "hw_hal_uart.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/pin_config/pin_config.h"
#include "peripheral_ctrl/pin_config/pinmap.h"
#include "cortex_m/nvic.h"

namespace RODOS {

constinit HW_HAL_UART* HW_HAL_UART::contextArray[] {};

HW_HAL_UART::HW_HAL_UART(HAL_UART& halUart, VA41620_UART_IDX uartIdx, pin_config::VA41620_PIN txPin, pin_config::VA41620_PIN rxPin, pin_config::VA41620_PIN rtsPin, pin_config::VA41620_PIN ctsPin)
    : m_idx { uartIdx }
    , m_uart { getUartStruct(m_idx) }
    , m_hal_uart { halUart }
    , m_rxErrCnt { 0 }
    , m_tx { txPin }
    , m_rx { rxPin}
    , m_rts { rtsPin }
    , m_cts { ctsPin }
{
    contextArray[uartIdx] = this;
}

UARTStruct& HW_HAL_UART::getUartStruct(VA41620_UART_IDX idx){
    switch(int(idx)){
    case UART_IDX0:
        return *uart0;
    case UART_IDX1:
        return *uart1;
    case UART_IDX2:
        return *uart2;
    }
    __builtin_trap();
}

void HW_HAL_UART::rxIrqHandler(){
    using namespace UART_RXSTATUS;
    using namespace UART_FIFO_CLR;
    using namespace UART_DATA;

    RegisterCopy rxStatus = m_uart.RXSTATUS;
    if(rxStatus.read<RDAVL>()){
        do{
            uint8_t c = static_cast<uint8_t>(m_uart.DATA.read<VALUE>());
            if (!m_receiveBuffer.put(c)) {
                m_rxErrCnt++;
            }
        } while(m_uart.RXSTATUS.read<RDAVL>());
        m_hal_uart.upCallDataReady();
    }

    if(rxStatus.read<RXOVR>()){
        m_uart.FIFO_CLR.write(RXFIFO(1));
        m_rxErrCnt++;
    }
}

void HW_HAL_UART::txIrqHandler(){
    using namespace UART_TXSTATUS;
    using namespace UART_DATA;
    using namespace UART_IRQ_ENB;
    RegisterCopy txStatus = m_uart.TXSTATUS;
    if(isFifoTXRunning() && txStatus.read<WRRDY>()) {
        uint8_t c { 0 };
        while(m_uart.TXSTATUS.read<WRRDY>() && m_transmitBuffer.get(c)){
            m_uart.DATA.write(VALUE(static_cast<uint32_t>(c)));
        }
        if (m_transmitBuffer.isEmpty()) {
            finishFifoTX();
        }
    } else if (!txStatus.read<WRBUSY>()) {
        m_uart.IRQ_ENB.set(IRQ_TX_EMPTY(0));
        m_hal_uart.upCallWriteFinished();
    }
}

void HW_HAL_UART::startFifoTX(){
    using namespace UART_IRQ_ENB;
    m_uart.IRQ_ENB.set(IRQ_TX(1), IRQ_TX_EMPTY(0));
}

bool HW_HAL_UART::isFifoTXRunning(){
    using namespace UART_IRQ_ENB;
    return m_uart.IRQ_ENB.read<IRQ_TX>();
}

void HW_HAL_UART::finishFifoTX(){
    using namespace UART_IRQ_ENB;
    m_uart.IRQ_ENB.set(IRQ_TX(0), IRQ_TX_EMPTY(1));
}

uint32_t HW_HAL_UART::getClockRate(){
    switch(int(m_idx)){
    case UART_IDX0: [[fallthrough]];
    case UART_IDX1:
        return globalClockSetup.getAPB2Frequency();
    case UART_IDX2:
        return globalClockSetup.getAPB1Frequency();
    default:
        return 0;
    }
}

uint32_t HW_HAL_UART::getClkScale(uint32_t baudrate){
    uint32_t clockRate = getClockRate();
    constexpr uint32_t oversamplingFactor { 16 };
    uint32_t sampleRate = baudrate * oversamplingFactor;

    //the CLKSCALE register is effectively a Q18.6 fixed-point number
    //the official documentation describes a way to calculate the integer and fractional parts seperately
    //this implementation is equivalent except for rounding differences in the least significant bit
    constexpr uint8_t fixedPointDigits { 6 };
    uint64_t fixedPointClockRate = static_cast<uint64_t>(clockRate)<<fixedPointDigits;
    return static_cast<uint32_t>(fixedPointClockRate/sampleRate);
}

void HW_HAL_UART::setBaudrate(uint32_t baudrate){
    using namespace UART_TXSTATUS;
    using namespace UART_ENABLE;
    m_uart.ENABLE.write(RXENABLE(0), TXENABLE(0));
    while(!m_uart.TXSTATUS.read<WRBUSY>());
    m_uart.CLKSCALE.write(getClkScale(baudrate));
    m_uart.ENABLE.write(RXENABLE(1), TXENABLE(1));
}

uint8_t HW_HAL_UART::getTxIRQn(VA41620_UART_IDX idx){
    switch(int(idx)){
    case UART_IDX0:
        return 24;
    case UART_IDX1:
        return 26;
    case UART_IDX2:
        return 28;
    default:
        __builtin_trap();
    }
}

uint8_t HW_HAL_UART::getRxIRQn(VA41620_UART_IDX idx){
    return static_cast<uint8_t>(getTxIRQn(idx)+1);
}

uint8_t HW_HAL_UART::getTxIRQn(){
    return getTxIRQn(m_idx);
}

uint8_t HW_HAL_UART::getRxIRQn(){
    return getRxIRQn(m_idx);
}

int32_t HW_HAL_UART::init(uint32_t baudrate){
    using namespace UART_ENABLE;
    using namespace UART_IRQ_ENB;
    resetPeriph();

    sysconfigPeripheralBitband->UART_ENABLE[m_idx].write(uint32_t(1));

    configurePin(m_rx);
    configurePin(m_tx);

    m_uart.CLKSCALE.write(getClkScale(baudrate));

    m_uart.ENABLE.write(RXENABLE(1), TXENABLE(1));
    m_uart.IRQ_ENB.write(IRQ_RX(1), IRQ_RX_TO(1));
    NVIC::enableIRQ(getTxIRQn());
    NVIC::enableIRQ(getRxIRQn());

    return 0;
}

void HW_HAL_UART::configurePin(pin_config::VA41620_PIN pinIdx){
    pin_config::configurePin(pinIdx, pin_config::UART_PIN_MAPS.getFunction(m_idx, pinIdx));
}

void HW_HAL_UART::resetPin(pin_config::VA41620_PIN pinIdx){
    pin_config::resetPin(pinIdx);
}

void HW_HAL_UART::reset(){
    NVIC::disableIRQ(getTxIRQn());
    NVIC::disableIRQ(getRxIRQn());

    resetPeriph();

    resetPin(m_rx);
    resetPin(m_tx);

    m_transmitBuffer.clear();
    m_receiveBuffer.clear();
}

void HW_HAL_UART::resetPeriph(){
    sysconfigPeripheralBitband->UART_RESET[m_idx].write(uint32_t(0));
    sysconfigPeripheralBitband->UART_RESET[m_idx].write(uint32_t(1));
}

void HW_HAL_UART::resetHwCtrlPins(){
    resetPin(m_rts);
    resetPin(m_cts);
}

size_t HW_HAL_UART::read(void* buf, size_t size) {
    if(size <= 0){
        return 0;
    }
    if(buf==nullptr){
        return 0;
    }

    size_t readCnt = 0;
    uint8_t* p = m_receiveBuffer.getBufferToRead(readCnt);

    if(p!=nullptr) {
        if(readCnt > size) {
            readCnt = size;
        }
        memcpy(buf, p, readCnt);

        m_receiveBuffer.readConcluded(readCnt);
        return readCnt;
    } else {
        return 0;
    }
}

int16_t HW_HAL_UART::getcharNoWait() {
    uint8_t c = 0;
    if(m_receiveBuffer.get(c)){
        return c;
    } else {
        return -1;
    }
}

size_t HW_HAL_UART::write(const void* buf, size_t size) {
    if(size <= 0){
        return 0;
    }
    if(buf==nullptr){
        return 0;
    }

    size_t spaceinbuffer;
    uint8_t* p = m_transmitBuffer.getBufferToWrite(spaceinbuffer);

    if(p!=nullptr) {
        if(spaceinbuffer < size) {
            size = spaceinbuffer;
        }
        memcpy(p, buf, size);
        m_transmitBuffer.writeConcluded(size);

        startFifoTX();

        return size;
    } else {
        return 0;
    }
}

int16_t HW_HAL_UART::putcharNoWait(uint8_t c) {
    if(m_transmitBuffer.put(c)) {
        startFifoTX();
        return c;
    } else {
        return -1;
    }
}

int32_t HW_HAL_UART::getRXBufLevel(){
    return static_cast<int32_t>(m_receiveBuffer.getElementCount());
}

int32_t HW_HAL_UART::getRXErrorCnt(){
    const int32_t temp = m_rxErrCnt;
    m_rxErrCnt = 0;
    return temp;
}

bool HW_HAL_UART::isWriteFinished(){
    using namespace UART_TXSTATUS;
    return m_transmitBuffer.isEmpty() && !m_uart.TXSTATUS.read<WRBUSY>();
}

bool HW_HAL_UART::isDataReady(){
    return !m_receiveBuffer.isEmpty();
}

void HW_HAL_UART::putcharFromInterrupt(char c){
    using namespace UART_TXSTATUS;
    using namespace UART_DATA;
    while (!m_uart.TXSTATUS.read<WRRDY>());
    m_uart.DATA.write(VALUE(c));
}

void HW_HAL_UART::putcharFromInterrupt(VA41620_UART_IDX idx, char c){
    contextArray[idx]->putcharFromInterrupt(c);
}


template<VA41620_UART_IDX idx>
inline void UARTx_rxIrqHandler() {
    HW_HAL_UART::contextArray[idx]->rxIrqHandler();
    NVIC::clearPendingIRQ(HW_HAL_UART::getRxIRQn(idx));
}


template<VA41620_UART_IDX idx>
inline void UARTx_txIrqHandler() {
    HW_HAL_UART::contextArray[idx]->txIrqHandler();
    NVIC::clearPendingIRQ(HW_HAL_UART::getTxIRQn(idx));
}

extern "C" {
void UART0_RX_IRQHandler(){
    UARTx_rxIrqHandler<UART_IDX0>();
}
void UART0_TX_IRQHandler(){
    UARTx_txIrqHandler<UART_IDX0>();
}
void UART1_RX_IRQHandler(){
    UARTx_rxIrqHandler<UART_IDX1>();
}
void UART1_TX_IRQHandler(){
    UARTx_txIrqHandler<UART_IDX1>();
}
void UART2_RX_IRQHandler(){
    UARTx_rxIrqHandler<UART_IDX2>();
}
void UART2_TX_IRQHandler(){
    UARTx_txIrqHandler<UART_IDX2>();
}
} // end extern "C"

}
