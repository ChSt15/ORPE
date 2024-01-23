/*
 * Copyright (c) 2019 Uni Wuerzburg, Wuerzburg

 * All rights reserved.
 *
 * @author Jorge Oliveira
 */

/* By driver enabled pins, "()"->also possible
 * - pins can be changed in HAL_UART::HAL_UART()
   -> Pins are used when selecting GPIO mode, and their LOC is used to route (check efr32fg1-datasheet page 129) 
   USART0 (VCOM) - 	Tx:  PA0 (LOC0)		Rx:  PA1 (LOC0)			VCOM Enable: PA5
            		CTS: PA2 (LOC30)	RTS: PA3 (LOC30)			          

   USART1 -  		Tx:  PB11 (LOC6)	Rx:  PB12 (LOC6)
            		CTS: PB13 (LOC4)	RTS: PB14 (LOC4)          
*/

#include "rodos.h"
#include "hal/hal_uart.h"
#include "hw_hal_gpio.h"

#include "dmadrv.h"

namespace RODOS {

    extern "C"
    {
    void LEUART0_IRQHandler();
    void USART0_RX_IRQHandler();
    void USART0_TX_IRQHandler();
    void USART1_RX_IRQHandler();
    void USART1_TX_IRQHandler();
    void USART2_RX_IRQHandler();
    void USART2_TX_IRQHandler();
    void USART3_RX_IRQHandler();
    void USART3_TX_IRQHandler();

    //void LDMA_IRQHandler();

    bool rxDMACallback(unsigned int channel,
                       unsigned int sequenceNo,
                       void *userParam);

    bool txDMACallback(unsigned int channel,
                       unsigned int sequenceNo,
                       void *userParam);
    }

    class HW_HAL_UART {

        friend class ReceiveTrigger;
        friend class HAL_UART;
        friend void UART_putchar(HAL_UART* uart, uint8_t c);

        friend void LEUART0_IRQHandler();
        friend void USART0_RX_IRQHandler();
        friend void USART0_TX_IRQHandler();
        friend void USART1_RX_IRQHandler();
        friend void USART1_TX_IRQHandler();
        friend void USART2_RX_IRQHandler();
        friend void USART2_TX_IRQHandler();
        friend void USART3_RX_IRQHandler();
        friend void USART3_TX_IRQHandler();

        //friend void LDMA_IRQHandler();
        friend bool rxDMACallback(unsigned int channel,
                                  unsigned int sequenceNo,
                                  void *userParam);

        friend bool txDMACallback(unsigned int channel,
                                  unsigned int sequenceNo,
                                  void *userParam);

        UART_IDX idx;
        bool hwFlowCtrl;
        uint32_t baudrate;
        HAL_UART* hal_uart;
        int uartRxError;


        USART_TypeDef *UARTx;

        LEUART_TypeDef *LEUARTx;

        GPIO_PIN tx;
        GPIO_PIN rx;
        GPIO_PIN rts;
        GPIO_PIN cts;

        int8_t txPinLoc;
        int8_t rxPinLoc;
        int8_t ctsPinLoc;
        int8_t rtsPinLoc;

        bool dmaRdEnable;
        bool dmaWrEnable;
        bool blockingWrEnable;
        bool blockingRdEnable;
        volatile bool DMATransmitRunning;
        volatile size_t DMATransmitRunningSize;
        volatile bool DMAReceiveRunning;
        volatile size_t DMAReceiveRunningSize;

//	LDMA_TransferCfg_t TxTransfer;
//	LDMA_Descriptor_t TxDesc;
//	LDMA_TransferCfg_t RxTransfer;
//	LDMA_Descriptor_t RxDesc;

        size_t DMAMaxReceiveSize;

        unsigned int rdDMACh;
        unsigned int wrDMACh;

        BlockFifo<uint8_t,UART_BUF_SIZE> receiveBuffer;
        BlockFifo<uint8_t,UART_BUF_SIZE> transmitBuffer;

        void DMATransmitFinishedHandler(int channel);
        void DMAReceiveFinishedHandler(int channel);


        //int DMAConfigure();
        void DMAStartTransmit(void* memoryBuffer, size_t len);
        void DMAStartReceive(void* memoryBuffer, size_t len);


        void SendTxBufWithDMA();
        void ReceiveIntoRxBufWithDMA();

        //There are 2 different interrupt vector groups: USART Reception, USART  Transmission
        void UART_RX_IRQHandler();
        void UART_TX_IRQHandler();
        void LEUART_RX_IRQHandler();
        void LEUART_TX_IRQHandler();

        int init(uint32_t baudrate);
        void initMembers(HAL_UART* halUart, UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin);

        int configRdDMA(size_t dmaRdSize);
        int configWrDMA();

        IRQn getUARTx_RX_IRQn();
        IRQn getUARTx_TX_IRQn();
        USART_TypeDef* getUARTx();
        CMU_Clock_TypeDef getUARTx_Clock();
        DMADRV_PeripheralSignal_t getUARTx_TX_dmaSignal();
        DMADRV_PeripheralSignal_t getUARTx_RX_dmaSignal();

        void putchar(uint8_t c);

    public:
        static int8_t getPinLoc(GPIO_PIN gpio, const uint8_t * lut); // return -1, if no "location" for "gpio" found, otherwise value between 0 ... 31
    };

/* pin location tables
 * - to save memory only one LUT for rx-,tx- and clk-pin is used
 *   -> this is possible because the 3 pin-location-tables are shifted by just one value
 *   -> each pin-location-table has 32 items
 *   -> we combined all 3 tables to one LUT with 34 items and use different starting points for each pin search
 */
#define PIN_LOCATION_TABLE_SIZE   32

    extern const uint8_t usart0_1PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5];
    extern const uint8_t usart2PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5];
    extern const uint8_t usart3PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5];

}
