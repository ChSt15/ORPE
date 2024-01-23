#include "hw_hal_uart.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "cortex_m/nvic.h"

namespace RODOS {

constinit HW_HAL_UART* HW_HAL_UART::contextArray[] {};

HW_HAL_UART::HW_HAL_UART(HAL_UART& halUart, UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin)
    : idx(uartIdx)
    , uart(mcu_specific::UARTS::getPeriphStruct(idx))
    , hal_uart(halUart)
    , rxErrCnt(0)
    , tx(txPin)
    , rx(rxPin)
    , rts(rtsPin)
    , cts(ctsPin)
{
    contextArray[uartIdx] = this;
}

void HW_HAL_UART::UARTIRQHandler(){
    using namespace UART_CR1;
    using namespace UART_ISR;
    using namespace UART_ICR;

    RegisterCopy isr = uart.ISR;

    if(isr.read<ORE>()){
        uart.ICR.write(ORECF(1));
        rxErrCnt++;
    }

    if(isr.read<RXNE>()) {
        uint8_t c = static_cast<uint8_t>(uart.RDR.read());
        if (!receiveBuffer.put(c)) {
            rxErrCnt++;
        }
        hal_uart.upCallDataReady();
    }

    if(isFifoTXRunning() && isr.read<TXE>()) {
        uint8_t c = 0;
        if (transmitBuffer.get(c)) {
            uart.TDR.write(static_cast<uint32_t>(c));
        } else {
            finishFifoTX();
        }
    } else if (isr.read<TC>()) {
        uart.CR1.set(TCIE(0));
        hal_uart.upCallWriteFinished();
    }
}

HWRCCPeriph& HW_HAL_UART::getRCCPeriph() const{
    return mcu_specific::UARTS::getRCCPeriph(idx);
}

void HW_HAL_UART::startFifoTX(){
    using namespace UART_CR1;
    uart.CR1.set(TXEIE(1), TCIE(0));
}

bool HW_HAL_UART::isFifoTXRunning(){
    using namespace UART_CR1;
    return uart.CR1.read<TXEIE>();
}

void HW_HAL_UART::finishFifoTX(){
    using namespace UART_CR1;
    uart.CR1.set(TXEIE(0), TCIE(1));
}

constexpr uint32_t HW_HAL_UART::getUartDiv(uint32_t baudrate){
    const uint32_t mult = isLPUART() ? 256 : 1;
    const uint32_t periphClock = (reinterpret_cast<uintptr_t>(&uart) == USART1Def::BASE_ADDR) ? PCLK::freq2 : PCLK::freq1;
    return (periphClock/baudrate)*mult;
}

constexpr bool HW_HAL_UART::isLPUART(){
    return reinterpret_cast<uintptr_t>(&uart) == LPUART1Def::BASE_ADDR;
}

constexpr bool HW_HAL_UART::supportsHWFlowCtrl(){
    const uintptr_t uartAddr = reinterpret_cast<uintptr_t>(&uart);
    const bool isUSART = (uartAddr == USART1Def::BASE_ADDR)
                      || (uartAddr == USART2Def::BASE_ADDR)
                      || (uartAddr == USART3Def::BASE_ADDR);
    const bool ctrlPinsValid = (cts != GPIO_INVALID) && (rts != GPIO_INVALID);
    return isUSART && ctrlPinsValid;
}

void HW_HAL_UART::setBaudrate(uint32_t baudrate){
    using namespace UART_CR1;
    using namespace UART_ISR;
    while(!uart.ISR.read<TC>());
    uart.CR1.set(UE(0));
    uart.BRR.write(getUartDiv(baudrate));
    uart.CR1.set(UE(1));
}

int32_t HW_HAL_UART::init(uint32_t baudrate){
    using namespace UART_CR1;
    using namespace UART_CR3;
    resetPeriph();

    getRCCPeriph().enableClock();

    configurePin(rx);
    configurePin(tx);

    uart.BRR.write(getUartDiv(baudrate));

    uart.CR1.set(UE(1), RE(1), TE(1), RXNEIE(1));
    NVIC::enableIRQ(mcu_specific::UARTS::getIRQn(idx));

    return 0;
}

int HW_HAL_UART::enableHWCtrlFlow(){
    using namespace UART_CR1;
    using namespace UART_CR3;
    if(supportsHWFlowCtrl()){
        configurePin(rts);
        configurePin(cts);
        uart.CR3.set(CTSE(1), RTSE(1));
        return 0;
    }else{
        return -1;
    }
}

void HW_HAL_UART::configurePin(GPIO_PIN pinIdx){
    GPIO::PinGroup pinGrp = GPIO::PinGroup(pinIdx);
    if (!pinGrp.isValid()) {
        RODOS_ERROR("HW_HAL_UART: PinGroup instance is not valid (no port)\n");
        return;
    }
    pinGrp.enableClocks();
    pinGrp.setMode(GPIO::MODE::ALTERNATE);
    pinGrp.setAlternateFunction(mcu_specific::UARTS::getAlternateFunction(idx, pinIdx));
    pinGrp.setOutputType(GPIO::OUTPUTTYPE::PUSHPULL);
    pinGrp.setSpeed(GPIO::SPEED::HIGH);
    pinGrp.setPull(GPIO::PULL::UP);
}

void HW_HAL_UART::resetPin(GPIO_PIN pinIdx){
    GPIO::PinGroup pinGrp = GPIO::PinGroup(pinIdx);
    pinGrp.reset();
}

void HW_HAL_UART::reset(){
    using namespace UART_CR3;
    NVIC::disableIRQ(mcu_specific::UARTS::getIRQn(idx));

    resetPeriph();

    resetPin(rx);
    resetPin(tx);
    if(uart.CR3.read<CTSE>()){
        resetHwCtrlPins();
    }

    transmitBuffer.clear();
    receiveBuffer.clear();
}

void HW_HAL_UART::resetPeriph(){
    HWRCCPeriph& periph = getRCCPeriph();
    periph.reset();
    periph.releaseReset();
}

void HW_HAL_UART::resetHwCtrlPins(){
    resetPin(rts);
    resetPin(cts);
}

size_t HW_HAL_UART::read(void* buf, size_t size) {
    if(size <= 0){
        return 0;
    }

    size_t readCnt = 0;
    uint8_t* p = receiveBuffer.getBufferToRead(readCnt);

    if(p!=nullptr) {
        if(readCnt > size) {
            readCnt = size;
        }
        memcpy(buf, p, readCnt);

        receiveBuffer.readConcluded(readCnt);
        return readCnt;
    } else {
        return 0;
    }
}

int16_t HW_HAL_UART::getcharNoWait() {
    uint8_t c = 0;
    if(receiveBuffer.get(c)){
        return c;
    } else {
        return -1;
    }
}

size_t HW_HAL_UART::write(const void* buf, size_t size) {
    if(size <= 0){
        return 0;
    }

    size_t spaceinbuffer;
    uint8_t* p = transmitBuffer.getBufferToWrite(spaceinbuffer);

    if(p!=nullptr) {
        if(spaceinbuffer < size) {
            size = spaceinbuffer;
        }
        memcpy(p, buf, size);
        transmitBuffer.writeConcluded(size);

        startFifoTX();

        return size;
    } else {
        return 0;
    }
}

int16_t HW_HAL_UART::putcharNoWait(uint8_t c) {
    if(transmitBuffer.put(c)) {
        startFifoTX();
        return c;
    } else {
        return -1;
    }
}

int32_t HW_HAL_UART::getRXBufLevel(){
    return static_cast<int32_t>(receiveBuffer.getElementCount());
}

int32_t HW_HAL_UART::getRXErrorCnt(){
    const int32_t temp = rxErrCnt;
    rxErrCnt = 0;
    return temp;
}

bool HW_HAL_UART::isWriteFinished(){
    using namespace UART_ISR;
    return transmitBuffer.isEmpty() && uart.ISR.read<TC>();
}

bool HW_HAL_UART::isDataReady(){
    return !receiveBuffer.isEmpty();
}

void HW_HAL_UART::putcharFromInterrupt(char c){
    using namespace UART_ISR;
    while (!uart.ISR.read<TXE>());
    uart.TDR.write(static_cast<uint32_t>(c));
}

void HW_HAL_UART::putcharFromInterrupt(UART_IDX idx, char c){
    contextArray[idx]->putcharFromInterrupt(c);
}


template<UART_IDX idx>
inline void UARTx_IRQHandler() {
    HW_HAL_UART::contextArray[idx]->UARTIRQHandler();
    NVIC::clearPendingIRQ(mcu_specific::UARTS::getIRQn(idx));
}

extern "C" {
void LPUART1_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX0)) {
        UARTx_IRQHandler<UART_IDX0>();
    }
}
void USART1_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX1)) {
        UARTx_IRQHandler<UART_IDX1>();
    }
}
void USART2_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX2)) {
        UARTx_IRQHandler<UART_IDX2>();
    }
}
void USART3_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX3)) {
        UARTx_IRQHandler<UART_IDX3>();
    }
}
void UART4_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX4)) {
        UARTx_IRQHandler<UART_IDX4>();
    }
}
void UART5_IRQHandler(){
    if constexpr (mcu_specific::UARTS::exists(UART_IDX5)) {
        UARTx_IRQHandler<UART_IDX5>();
    }
}
} // end extern "C"

}
