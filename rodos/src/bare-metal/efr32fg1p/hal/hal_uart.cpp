/*
 * Copyright (c) 2019 Uni Wuerzburg, Wuerzburg

 * All rights reserved.
 *
 * @author Jorge Oliveira, Michael Ruffer
 */

/* USART default pins, when using constructor "HAL_UART(UART_IDX uartIdx)"
 *          TX    RX    CTS   RTS
 * USART0   PA0   PA1   PA2   PA3     VCOM Enable: PA5
 * USART1   PB11  PB12  PB13  PB14
 * USART2   PA5   PA6   PA7   PA8
 * USART3   PB6   PB7   PB8   PB9
 * USART4   PB11  PB23  --    --      LEUART Baud fixed to 115200
 */




/* ToDo:
 * hwFlowCtrl testen
 * enable VCOM somewhere else
 *    -> it is only necessary for specific hardware board "BRD4001A"
 */


#include "hw_hal_uart.h"
#include "vendor-headers.h"


#define UART_IDX_MIN        UART_IDX0
#define UART_IDX_MAX        UART_IDX4

namespace RODOS {

#define DMA_RD_TIMEOUT    10*MILLISECONDS

static volatile bool enableDMATriggerThread=false;

class ReceiveTrigger : StaticThread<> {
  public:
    ReceiveTrigger():StaticThread<>("UARTRecvTrigger"){}
  private:
    void run();
};

static ReceiveTrigger triggerthread;



HW_HAL_UART UART_contextArray[UART_IDX_MAX+1];

HAL_UART::HAL_UART(UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin)
{
    if ((uartIdx < UART_IDX_MIN) || (uartIdx > UART_IDX_MAX)) {
        context = nullptr;
        return;
    }
    // LEUART Pins are hardcoded and can not be set custom! Use HAL_UART(UART_IDX4) instead
    if(uartIdx == UART_IDX4){
        //FIXME: make pins changable
        context = &UART_contextArray[uartIdx];
        context->initMembers(this, uartIdx, GPIO_028, GPIO_029, GPIO_000, GPIO_000); // Hardcoded to PB12 PB13
        return;
    }
	context = &UART_contextArray[uartIdx];
	context->initMembers(this, uartIdx, txPin, rxPin, rtsPin, ctsPin);
}


HAL_UART::HAL_UART(UART_IDX uartIdx)
{
    if ((uartIdx < UART_IDX_MIN) || (uartIdx > UART_IDX_MAX)) {
      context = nullptr;
      return;
    }

	context = &UART_contextArray[uartIdx];

	switch (uartIdx){
	case UART_IDX0: //                      tx-PA0    rx-PA1    rts-PA3   cts-PA2
	    context->initMembers(this, uartIdx, GPIO_000, GPIO_001, GPIO_003, GPIO_002);
	    break;

	case UART_IDX1: //                      tx-PB11   rx-PB12   rts-PB14  cts-PB13
	    context->initMembers(this, uartIdx, GPIO_027, GPIO_028, GPIO_030, GPIO_029);
      break;

	case UART_IDX2: //                      tx-PA5    rx-PA6    rts-PA8   cts-PA7
	    context->initMembers(this, uartIdx, GPIO_005, GPIO_006, GPIO_008, GPIO_007);
	    break;

	case UART_IDX3: //                      tx-PB6    rx-PB7    rts-PB9   cts-PB8
	    context->initMembers(this, uartIdx, GPIO_022, GPIO_023, GPIO_025, GPIO_024);
	    break;
    case UART_IDX4:
        context->initMembers(this, uartIdx, GPIO_028, GPIO_029, GPIO_000, GPIO_000); // Hardcoded to PB12 PB13
        break;
	default:
	    break;
	}
}



extern "C"
{

void LEUART0_IRQHandler(){
    uint32_t flags = LEUART_IntGet(LEUART0);
    // RX portion of the interrupt handler
    if (flags & LEUART_IF_RXDATAV) {
        //RX stuff
        UART_contextArray[UART_IDX4].LEUART_RX_IRQHandler();
    }

    // TX portion of the interrupt handler
    if ((flags & LEUART_IF_TXC) || (flags & LEUART_IF_TXBL)) {
        //TX Stuff
        UART_contextArray[UART_IDX4].LEUART_TX_IRQHandler();
    }
    NVIC_ClearPendingIRQ(LEUART0_IRQn);
}

void USART0_RX_IRQHandler()
{
	UART_contextArray[UART_IDX0].UART_RX_IRQHandler();
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
}

void USART1_RX_IRQHandler()
{
	UART_contextArray[UART_IDX1].UART_RX_IRQHandler();
	NVIC_ClearPendingIRQ(USART1_RX_IRQn);
}

void USART2_RX_IRQHandler()
{
  UART_contextArray[UART_IDX2].UART_RX_IRQHandler();
  NVIC_ClearPendingIRQ(USART2_RX_IRQn);
}

void USART3_RX_IRQHandler()
{
  UART_contextArray[UART_IDX3].UART_RX_IRQHandler();
  NVIC_ClearPendingIRQ(USART3_RX_IRQn);
}



void USART0_TX_IRQHandler()
{
	UART_contextArray[UART_IDX0].UART_TX_IRQHandler();
	NVIC_ClearPendingIRQ(USART0_TX_IRQn);
}

void USART1_TX_IRQHandler()
{
	UART_contextArray[UART_IDX1].UART_TX_IRQHandler();
	NVIC_ClearPendingIRQ(USART1_TX_IRQn);
}

void USART2_TX_IRQHandler()
{
  UART_contextArray[UART_IDX2].UART_TX_IRQHandler();
  NVIC_ClearPendingIRQ(USART2_TX_IRQn);
}

void USART3_TX_IRQHandler()
{
  UART_contextArray[UART_IDX3].UART_TX_IRQHandler();
  NVIC_ClearPendingIRQ(USART3_TX_IRQn);
}


bool rxDMACallback(unsigned int channel,
                   unsigned int sequenceNo,
                   void *userParam) {
  (void) sequenceNo;
  ((HW_HAL_UART*)userParam)->DMAReceiveFinishedHandler(channel);

  return false;
}

bool txDMACallback(unsigned int channel,
                   unsigned int sequenceNo,
                   void *userParam) {
  (void) sequenceNo;
  ((HW_HAL_UART*)userParam)->DMATransmitFinishedHandler(channel);

  return false;
}


//void LDMA_IRQHandler(void)
//{
// 	uint32_t ch = 0;
// 	uint32_t mask = 0;
//  	// Get all pending and enabled interrupts.
//  	uint32_t pending = LDMA_IntGetEnabled();
//  	// Iterate over all LDMA channels.
//  	for (ch = 0; ch < DMA_CHAN_COUNT; ch++)
//  	{
//    	mask = 0x1u << ch;
//    	if (pending & mask)
//   	 	{
//      		// Clear interrupt flag.
//      		LDMA->IFC = mask;
//      		//
//      		switch(ch) {
//      		  case 0:
//      		 		UART_contextArray[UART_IDX0].DMATransmitFinishedHandler();
//      		 		break;
//      		 	case 1:
//      		 	  UART_contextArray[UART_IDX1].DMATransmitFinishedHandler();
//      		 	  break;
//      		 	case 2:
//      		 	  UART_contextArray[UART_IDX0].DMAReceiveFinishedHandler();
//      		 	  break;
//      		 	case 3:
//      		 	  UART_contextArray[UART_IDX1].DMAReceiveFinishedHandler();
//      		 	  break;
//      		 	default:
//      		 	  break;
//      		}
//   	 	}
//  	}
//}

} // end extern "C"


/*
 * USART
 * - all USART will be initialized in 8N1 mode
 */
int32_t HAL_UART::init(uint32_t iBaudrate) {

	if (context == nullptr) {return -1;}

	return context->init(iBaudrate);
}


int32_t HAL_UART::config(UART_PARAMETER_TYPE type, int32_t paramVal) {

	if (context == nullptr) {return -1;}

    if(context->idx == UART_IDX4) return -1; // Not implemented for idx4 (leuart)

	switch (type) {
		case UART_PARAMETER_BAUDRATE:
		  if ( (paramVal > 0) ){ //  && isWriteFinished()){    not working because after reset isWriteFinished returns false because of TXC-Bit
		      USART_Enable(context->UARTx, usartDisable);
		      USART_BaudrateAsyncSet(context->UARTx, 0, (uint32_t)paramVal, usartOVS16);
		      USART_Enable(context->UARTx, usartEnable);

		      context->baudrate = static_cast<uint32_t >(paramVal);
		      return 0;
		  } else {
		      return -1;
		  }

		case UART_PARAMETER_HW_FLOW_CONTROL:
			/* if rts and cts are set to GPIO_INVALID HW flow control is not available */
			if (paramVal > 0 && context->rts != GPIO_INVALID){ // enable HW Flow Control
			   // if (isWriteFinished()){     not working because after reset isWriteFinished returns false because of TXC-Bit
			        USART_Enable(context->UARTx, usartDisable);
			        HW_HAL_GPIO::configurePin(context->cts, gpioModeInput, 0);
			        HW_HAL_GPIO::configurePin(context->rts, gpioModePushPull, 1);

			        context->UARTx->ROUTELOC1 = (context->ctsPinLoc << _USART_ROUTELOC1_CTSLOC_SHIFT) |
			                                    (context->rtsPinLoc << _USART_ROUTELOC1_RTSLOC_SHIFT);

			        context->UARTx->ROUTEPEN &= ~(_USART_ROUTEPEN_RTSPEN_MASK | _USART_ROUTEPEN_CTSPEN_MASK);
			        context->UARTx->ROUTEPEN |= USART_ROUTEPEN_RTSPEN | USART_ROUTEPEN_CTSPEN;
			        USART_Enable(context->UARTx, usartEnable);
			        context->hwFlowCtrl = true;
//			    }else{
//			        return -1;
//			    }
			} else if(context->rts != GPIO_INVALID){  // disable HW Flow Control
//			    if (isWriteFinished()){
			        USART_Enable(context->UARTx, usartDisable);
			        context->UARTx->ROUTEPEN &= ~(_USART_ROUTEPEN_RTSPEN_MASK | _USART_ROUTEPEN_CTSPEN_MASK);
			        HW_HAL_GPIO::resetPin(context->rts);
			        HW_HAL_GPIO::resetPin(context->cts);
			        USART_Enable(context->UARTx, usartEnable);
			        context->hwFlowCtrl = false;
//			    }else{
//			        return -1;
//			    }
			}else{
			    return -1;
			}

			return 0;  // end case UART_PARAMETER_HW_FLOW_CONTROL

		case UART_PARAMETER_ENABLE_DMA:
      if (context->configRdDMA(paramVal) == 0){
          enableDMATriggerThread = true;
          return context->configWrDMA();
      }else{
          return -1;
      }

		case UART_PARAMETER_ENABLE_DMA_RD:
		    enableDMATriggerThread = true;
		    return context->configRdDMA(paramVal);


    case UART_PARAMETER_ENABLE_DMA_WR:
        return context->configWrDMA();


		case UART_PARAMETER_BLOCKING_WR:
		  if (paramVal > 0){
		      context->blockingWrEnable = true;
		  }else{
		      context->blockingWrEnable = false;
		  }
		  return 0;


    case UART_PARAMETER_BLOCKING_RD:
      if (paramVal > 0){
          context->blockingRdEnable = true;
      }else{
          context->blockingRdEnable = false;
      }
      return 0;

		default:
		  return -1;
	}
}


void HAL_UART::reset()
{
	if (context == nullptr) {return;}

    if(context->idx == UART_IDX4){
        LEUART_TypeDef *leuart = context->LEUARTx;
        NVIC_DisableIRQ(LEUART0_IRQn);
        LEUART_Reset(leuart);
        HW_HAL_GPIO::resetPin(context->rx);
        HW_HAL_GPIO::resetPin(context->tx);
    }else{
        USART_TypeDef *usart = context->UARTx;

        // Interrupt UART disable
        NVIC_DisableIRQ(context->getUARTx_RX_IRQn());
        NVIC_DisableIRQ(context->getUARTx_TX_IRQn());

        // reset interface
        USART_Reset(usart);

        HW_HAL_GPIO::resetPin(context->rx);
        HW_HAL_GPIO::resetPin(context->tx);

        if (context->hwFlowCtrl){
            if (context->rts != GPIO_INVALID) { HW_HAL_GPIO::resetPin(context->rts); }
            if (context->cts != GPIO_INVALID) { HW_HAL_GPIO::resetPin(context->cts); }
        }
    }
}


size_t HAL_UART::read(void* buf, size_t size)
{
    if (context == nullptr) {return 0;}
    if(size <=0) return 0;

    size_t readCnt = 0;
    uint8_t* rx = static_cast<uint8_t*>(buf);
    size_t freeSize = size;

    while (freeSize > 0) {
        size_t dataInBuffer = 0;
        uint8_t* p = context->receiveBuffer.getBufferToRead(dataInBuffer);
        //PRINTF("UART read rx buffer\n");

        if(p){
            if (dataInBuffer > freeSize) {dataInBuffer = freeSize;}

            readCnt += dataInBuffer;

            for (size_t i = 0; i < dataInBuffer; i++ ) {
                *rx = p[i];
                rx++;
                freeSize--;
            }
            context->receiveBuffer.readConcluded(dataInBuffer);

            if (context->dmaRdEnable){
                if (!context->DMAReceiveRunning) { // if DMA finished, restart it
                    context->ReceiveIntoRxBufWithDMA();
                }
            }
        }else{
            if(context->blockingRdEnable){
                //PRINTF("UART suspenduntildataready\n");
                this->suspendUntilDataReady();
            }
        }

        if(!context->blockingRdEnable){
            break;
        }
    }

    return readCnt;
}



size_t HAL_UART::write(const void* buf, size_t size)
{
    if (context == nullptr) {return 0;}
    if(size <= 0) return 0;

    const uint8_t* tx = static_cast<const uint8_t*>(buf);
    size_t writeCnt = 0;

    while (size > 0){
        size_t spaceInBuffer = 0;
        uint8_t* p = context->transmitBuffer.getBufferToWrite(spaceInBuffer);

        if(p) {
            if(spaceInBuffer > size) { spaceInBuffer=size; }

            writeCnt += spaceInBuffer;

            for (size_t i=0; i < spaceInBuffer; i++) {
                p[i] = *tx;
                tx++;
                size--;
            }

            context->transmitBuffer.writeConcluded(spaceInBuffer);
            if(context->idx == UART_IDX4){
                //LEUART_IntDisable(context->LEUARTx, LEUART_IEN_TXC);
            }else{
                //USART_IntDisable(context->UARTx, USART_IEN_TXC);
            }

            if(context->dmaWrEnable) {
                if(!context->DMATransmitRunning){
                    context->SendTxBufWithDMA();
                }
            } else {
                if(context->idx == UART_IDX4){
                    LEUART_IntEnable(context->LEUARTx, LEUART_IEN_TXBL);
                }else{
                    USART_IntEnable(context->UARTx, USART_IEN_TXBL);
                }
            }
        }else{
            if(context->blockingWrEnable){
                //PRINTF("UART suspendUntilWriteFinished\n");
                this->suspendUntilWriteFinished();
            }
        }

        if(!context->blockingWrEnable){
            break;
        }
    }

    return writeCnt;
}


int16_t HAL_UART::getcharNoWait()
{
    if (context == nullptr) {return -1;}

    uint8_t c = 0;
    bool tmpBlockingFlag = context->blockingRdEnable;

    context->blockingRdEnable = false;
    size_t retVal = read(&c,1);
    context->blockingRdEnable = tmpBlockingFlag;

    if(retVal == 1) {
        return c;
    }

    return -1;
}


int16_t HAL_UART::putcharNoWait(uint8_t c) {
  if (context == nullptr) {return -1;}

  bool tmpBlockingFlag = context->blockingWrEnable;

  context->blockingWrEnable = false;
  size_t retVal = write(&c,1);
  context->blockingWrEnable = tmpBlockingFlag;

	if(retVal == 1) {
	    return c;
	}

	return -1;
}


int32_t HAL_UART::status(UART_STATUS_TYPE type)
{
    if (context == nullptr) {return -1;}

	switch (type)
	{
		case UART_STATUS_RX_BUF_LEVEL:
			return static_cast<int32_t>(context->receiveBuffer.getElementCount());

		case UART_STATUS_RX_ERROR:
			int temp;
			temp = context->uartRxError;
			context->uartRxError = 0;
			return temp;

		// case UART_STATUS_TX_COMPLETE:
		//     return (USART_StatusGet(context->UARTx) & USART_STATUS_TXC);
		//...
		default: return -1;
	}
}


bool HAL_UART::isWriteFinished()
{
    if (context == nullptr) {return true;} // false would create an infinite loop
    if(context->idx == UART_IDX4){
        return context->transmitBuffer.isEmpty() && (LEUART_StatusGet(context->LEUARTx) & LEUART_STATUS_TXC);
    }else{
        return context->transmitBuffer.isEmpty() && (USART_StatusGet(context->UARTx) & USART_STATUS_TXC);
    }
}


bool HAL_UART::isDataReady()
{
    if (context == nullptr) {return false;}
    return !context->receiveBuffer.isEmpty();
}





/****************** HW_HAL_UART *******************/

void HW_HAL_UART::initMembers(HAL_UART* halUart, UART_IDX uartIdx, GPIO_PIN txPin, GPIO_PIN rxPin, GPIO_PIN rtsPin, GPIO_PIN ctsPin){
    idx = uartIdx;
    baudrate = 115200;
    hal_uart = halUart;
    dmaRdEnable = false;
    dmaWrEnable = false;
    rdDMACh = 9999;
    wrDMACh = 9999;
    blockingWrEnable = false;
    blockingRdEnable = false;
    DMATransmitRunning = false;
    DMAReceiveRunning = false;
    if(idx == UART_IDX4){
        LEUARTx = LEUART0;
    }else {
        hwFlowCtrl = UARTx->ROUTEPEN; //contains rts and cts bits
        UARTx = getUARTx();
    }


    tx = txPin;
    rx = rxPin;
    rts = rtsPin;
    cts = ctsPin;

    /* pin routing
     */
    switch(idx){
      case UART_IDX0:
        txPinLoc = HW_HAL_UART::getPinLoc(tx, &usart0_1PinLoc_LUT[0]);
        rxPinLoc = HW_HAL_UART::getPinLoc(rx, &usart0_1PinLoc_LUT[1]);
        ctsPinLoc = HW_HAL_UART::getPinLoc(cts, &usart0_1PinLoc_LUT[4]);
        rtsPinLoc = HW_HAL_UART::getPinLoc(rts, &usart0_1PinLoc_LUT[5]);
        break;

      case UART_IDX1:
        txPinLoc = HW_HAL_UART::getPinLoc(tx, &usart0_1PinLoc_LUT[0]);
        rxPinLoc = HW_HAL_UART::getPinLoc(rx, &usart0_1PinLoc_LUT[1]);
        ctsPinLoc = HW_HAL_UART::getPinLoc(cts, &usart0_1PinLoc_LUT[4]);
        rtsPinLoc = HW_HAL_UART::getPinLoc(rts, &usart0_1PinLoc_LUT[5]);
        break;

      case UART_IDX2:
        txPinLoc = HW_HAL_UART::getPinLoc(tx, &usart2PinLoc_LUT[0]);
        rxPinLoc = HW_HAL_UART::getPinLoc(rx, &usart2PinLoc_LUT[1]);
        ctsPinLoc = HW_HAL_UART::getPinLoc(cts, &usart2PinLoc_LUT[4]);
        rtsPinLoc = HW_HAL_UART::getPinLoc(rts, &usart2PinLoc_LUT[5]);
        break;

      case UART_IDX3:
        txPinLoc = HW_HAL_UART::getPinLoc(tx, &usart3PinLoc_LUT[0]);
        rxPinLoc = HW_HAL_UART::getPinLoc(rx, &usart3PinLoc_LUT[1]);
        ctsPinLoc = HW_HAL_UART::getPinLoc(cts, &usart3PinLoc_LUT[4]);
        rtsPinLoc = HW_HAL_UART::getPinLoc(rts, &usart3PinLoc_LUT[5]);
        break;

      case UART_IDX4:
          txPinLoc = 0; //Hardcoded PB12 in init, routing is static
          rxPinLoc = 0; // Hardcoded PB13 in init, routing is static
          ctsPinLoc = 0; //LEUART No cts pin
          rtsPinLoc = 0; // LEUART No rts pin
          break;
      default:
        //RODOS_ERROR("UART index out of range"); !!! RODOS_ERROR-call hangs constructor call !!!
        return;
    }

    if (rxPinLoc <0 || txPinLoc <0 || rtsPinLoc < 0 || ctsPinLoc < 0){
         //RODOS_ERROR("UART pin invalid!!!");  !!! RODOS_ERROR-call hangs constructor call !!!
     }
}


/*************DMA**************/
void ReceiveTrigger::run()
{
    while(!enableDMATriggerThread){
        suspendCallerUntil(NOW()+ 10 * SECONDS);
    }

    HW_HAL_UART *hwHalUart = NULL;

    TIME_LOOP(1*MILLISECONDS, DMA_RD_TIMEOUT) {
        for(int i=UART_IDX_MIN; i<=UART_IDX_MAX; i++){
            hwHalUart = &UART_contextArray[i];
            if(hwHalUart->dmaRdEnable){
                DMADRV_StopTransfer(hwHalUart->rdDMACh);
                hwHalUart->DMAReceiveFinishedHandler(hwHalUart->rdDMACh);
            }
        }
    }

}


void HW_HAL_UART::SendTxBufWithDMA()
{
	size_t len;
	uint8_t*  p = transmitBuffer.getBufferToRead(len);

	if(p)	{
		DMATransmitRunningSize = len;
		DMATransmitRunning = true;
		DMAStartTransmit(p, len);
	}
}

void HW_HAL_UART::DMAStartTransmit(void* memoryBuffer, size_t len) {
    void *txPort = (void *)&(UARTx->TXDATA);

    // Start transmit DMA.
    DMADRV_MemoryPeripheral(wrDMACh,
                            getUARTx_TX_dmaSignal(),
                            txPort,
                            memoryBuffer,
                            true,
                            len,
                            dmadrvDataSize1,
                            txDMACallback,
                            (void*)this);
}

void HW_HAL_UART::DMATransmitFinishedHandler(int channel) {
    size_t bytesNotTransfered = LDMA_TransferRemainingCount(channel);
    size_t bytesTransfered = DMATransmitRunningSize - bytesNotTransfered;

    transmitBuffer.readConcluded(bytesTransfered);

    if(transmitBuffer.isEmpty()) {
        DMATransmitRunning = false;
        USART_IntEnable(UARTx, USART_IEN_TXC);	// Enabled to check when the Transmission is complete (shift register is empty)
    }	else {
        SendTxBufWithDMA();
    }
}

void HW_HAL_UART::ReceiveIntoRxBufWithDMA() {

	size_t len;
	uint8_t* p = receiveBuffer.getBufferToWrite(len);

	if(p) {
		if(len > DMAMaxReceiveSize) {
			len = DMAMaxReceiveSize;
		}
		DMAReceiveRunningSize = len;
		DMAReceiveRunning = true;
		DMAStartReceive(p, len);
	}	else {
		DMAReceiveRunning = false; //Buffer Overflow, disable Receive until next read
		uartRxError++;
	}
}


void HW_HAL_UART::DMAStartReceive(void* memoryBuffer, size_t len) {
  //UARTx->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  void *rxPort = (void *)&(UARTx->RXDATA);

  // Start receive DMA.
  DMADRV_PeripheralMemory(rdDMACh,
                          getUARTx_RX_dmaSignal(),
                          memoryBuffer,
                          rxPort,
                          true,
                          len,
                          dmadrvDataSize1,
                          rxDMACallback,  // callback
                          (void*)this);   // userparameter
}

void HW_HAL_UART::DMAReceiveFinishedHandler(int channel){
	size_t bytesTransfered = DMAReceiveRunningSize - LDMA_TransferRemainingCount(channel);

	receiveBuffer.writeConcluded(bytesTransfered);

	if(bytesTransfered > 0) {
		hal_uart->upCallDataReady();
	}

	ReceiveIntoRxBufWithDMA();
}


void HW_HAL_UART::LEUART_RX_IRQHandler() {
    uint8_t c;

    if(LEUART_IntGet(LEUARTx) & LEUART_IF_RXDATAV){
        LEUART_IntClear(LEUARTx, LEUART_IF_RXDATAV);
        c = LEUART_RxDataGet(LEUARTx) & 0xFF;
        if(!receiveBuffer.put(c)){
            uartRxError++; //FIFO OVerflow
        }

        hal_uart->upCallDataReady();
    }
}

void HW_HAL_UART::LEUART_TX_IRQHandler() {
    uint8_t c;

    if(LEUART_IntGet(LEUARTx) & LEUART_IF_TXC) {				// Transmission is complete (shift register is empty)
        LEUART_IntDisable(LEUARTx, LEUART_IEN_TXC);
        LEUART_IntClear(LEUARTx, LEUART_IF_TXC);   // Clear flag

        hal_uart->upCallWriteFinished();
    }	else if (LEUART_IntGet(LEUARTx) & LEUART_IF_TXBL) {		// Ready to send (Empty TX buffer)
        if(transmitBuffer.get(c))	{
            LEUART_Tx(LEUARTx,c);
        }	else {
            // no more data, disable TXBL(transmitter idle/empty) and enable TXC(transmission complete)
            LEUART_IntDisable(LEUARTx, LEUART_IEN_TXBL);
            LEUART_IntClear(LEUARTx, LEUART_IF_TXBL);   // Clear flag
            LEUART_IntEnable(LEUARTx, LEUART_IEN_TXC);
            //hal_uart->upCallWriteFinished();
        }
    }
}

void HW_HAL_UART::UART_RX_IRQHandler()
{
	uint8_t c;

	//test overrun error and when DMA is enabled frame error, noise error and overrun error
/*	if (USART_GetFlagStatus(UARTx,USART_FLAG_ORE) ||
    	(isDMAEnabeld && (USART_GetITStatus(UARTx,USART_IT_FE) || USART_GetITStatus(UARTx,USART_IT_NE) || USART_GetITStatus(UARTx,USART_IT_ORE_ER))))
    {
        USART_ReceiveData(UARTx);
        uartRxError++;
    }
    else */
	  if(USART_IntGet(UARTx) & USART_IF_RXDATAV){
	      c = USART_RxDataGet(UARTx) & 0xFF;

	      if(!receiveBuffer.put(c)){
	          uartRxError++; //FIFO OVerflow
	      }

//	      USART_IntDisable(UARTx, USART_IEN_RXDATAV);
	      hal_uart->upCallDataReady();
	  }
}

void HW_HAL_UART::UART_TX_IRQHandler()
{
	  uint8_t c;

	  if(USART_IntGet(UARTx) & USART_IF_TXC) {				// Transmission is complete (shift register is empty)
	      USART_IntDisable(UARTx, USART_IEN_TXC);
	      //USART_IntClear(UARTx, USART_IntGet(UARTx));		// Clear flags
	      USART_IntClear(UARTx, USART_IF_TXC);   // Clear flag

	      hal_uart->upCallWriteFinished();
	  }	else if (USART_IntGet(UARTx) & USART_IF_TXBL) {		// Ready to send (Empty TX buffer)
	      if(transmitBuffer.get(c))	{
	          USART_Tx(UARTx,c);
	      }	else {
	          // no more data, disable TXBL(transmitter idle/empty) and enable TXC(transmission complete)
	          USART_IntDisable(UARTx, USART_IEN_TXBL);
	          //USART_IntClear(UARTx, USART_IntGet(UARTx));		// Clear flags
	          USART_IntClear(UARTx, USART_IF_TXBL);   // Clear flag

	          USART_IntEnable(UARTx, USART_IEN_TXC);
	          //hal_uart->upCallWriteFinished();
	      }
	  }else{
	      USART_IntClear(UARTx, USART_IntGet(UARTx));   // Clear flags
	  }
}

IRQn HW_HAL_UART::getUARTx_RX_IRQn()
{
    switch(idx){
   	case UART_IDX0: return USART0_RX_IRQn;
    case UART_IDX1: return USART1_RX_IRQn;
    case UART_IDX2: return USART2_RX_IRQn;
    case UART_IDX3: return USART3_RX_IRQn;
    default: return USART0_RX_IRQn;
    }
}

IRQn HW_HAL_UART::getUARTx_TX_IRQn()
{
    switch(idx){
   	case UART_IDX0: return USART0_TX_IRQn;
    case UART_IDX1: return USART1_TX_IRQn;
    case UART_IDX2: return USART2_TX_IRQn;
    case UART_IDX3: return USART3_TX_IRQn;
    default: return USART0_TX_IRQn;
    }
}

USART_TypeDef* HW_HAL_UART::getUARTx()
{
    switch(idx){
    case UART_IDX0: return USART0;
    case UART_IDX1: return USART1;
    case UART_IDX2: return USART2;
    case UART_IDX3: return USART3;
    default: return NULL;
    }
}

CMU_Clock_TypeDef HW_HAL_UART::getUARTx_Clock()
{
	switch(idx){
	  case UART_IDX0: return cmuClock_USART0;
    case UART_IDX1: return cmuClock_USART1;
    case UART_IDX2: return cmuClock_USART2;
    case UART_IDX3: return cmuClock_USART3;
    default: return cmuClock_USART0;
	}
}


#warning "spidrv.c uses dmadrvPeripheralSignal_USART1_TXBL"
// ToDo: check if correct signal is used
DMADRV_PeripheralSignal_t HW_HAL_UART::getUARTx_TX_dmaSignal()
{
    switch(idx){
   	case UART_IDX0: return dmadrvPeripheralSignal_USART0_TXEMPTY;
    case UART_IDX1: return dmadrvPeripheralSignal_USART1_TXEMPTY;
    case UART_IDX2: return dmadrvPeripheralSignal_USART2_TXEMPTY;
    case UART_IDX3: return dmadrvPeripheralSignal_USART3_TXEMPTY;
    default: return dmadrvPeripheralSignal_USART0_TXEMPTY;
    }
}

DMADRV_PeripheralSignal_t HW_HAL_UART::getUARTx_RX_dmaSignal()
{
    switch(idx){
   	case UART_IDX0: return dmadrvPeripheralSignal_USART0_RXDATAV;
    case UART_IDX1: return dmadrvPeripheralSignal_USART1_RXDATAV;
    case UART_IDX2: return dmadrvPeripheralSignal_USART2_RXDATAV;
    case UART_IDX3: return dmadrvPeripheralSignal_USART3_RXDATAV;
    default: return dmadrvPeripheralSignal_USART0_RXDATAV;
    }
}


int HW_HAL_UART::init(uint32_t baudrate) {
    if ((idx < UART_IDX_MIN) || (idx > UART_IDX_MAX)) {return -1;}

    if(idx == UART_IDX4){
        this->baudrate = 115200;
    }else{
        this->baudrate = baudrate;
    }

    if(idx == UART_IDX4){
       // Enable LEUART
       // Setting CMU_CTRL Wait State for HFLE Interface to allow freq higher 32 MHz (ref. p328)
       CMU->CTRL = (CMU->CTRL &~_CMU_CTRL_WSHFLE_MASK) | CMU_CTRL_WSHFLE;
       //Enable LE (low energy) clocks
       CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules
       CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_HFCLKLE); // Set a reference clock

       // Enable clocks for LEUART0
       CMU_ClockEnable(cmuClock_LEUART0, true);

       // set pin modes for USART TX and RX pins and VCOM enable
       HW_HAL_GPIO::configurePin(this->rx, gpioModeInput, 0);
       HW_HAL_GPIO::configurePin(this->tx, gpioModePushPull, 1);

       // Initialize the LEUART0 module
       LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
       init.baudrate = 115200;
       LEUART_Init(LEUART0, &init);

       // Enable LEUART0 RX/TX pins on PB[13:12]
       LEUART0->ROUTELOC0 = LEUART_ROUTELOC0_RXLOC_LOC7 | LEUART_ROUTELOC0_TXLOC_LOC7; // Hardcoded to PB12 PB13
       LEUART0->ROUTEPEN  = LEUART_ROUTEPEN_RXPEN | LEUART_ROUTEPEN_TXPEN;

       // Enable USART Interrupts
       LEUART_IntEnable(LEUARTx, LEUART_IEN_RXDATAV);
       NVIC_EnableIRQ(LEUART0_IRQn);
    }else{
        CMU_ClockEnable(getUARTx_Clock(), true); // Enable oscillator to USART module

#ifndef EFR32FG12P433F1024GM68
        /* enable VCOM for Debug outputs (USART0 on BRD4001A): set PA5 high */
        if (idx == UART_IDX0){
            HW_HAL_GPIO::configurePin(GPIO_005, gpioModePushPull, 1);
        }
#endif

        // set pin modes for USART TX and RX pins and VCOM enable
        HW_HAL_GPIO::configurePin(this->rx, gpioModeInput, 0);
        HW_HAL_GPIO::configurePin(this->tx, gpioModePushPull, 1);

        USART_InitAsync_TypeDef Uis = USART_INITASYNC_DEFAULT;
        Uis.baudrate = baudrate;  			// set baudrate
        USART_InitAsync(UARTx, &Uis);		// init USART

        // Route pins
        UARTx->ROUTELOC0 =  (rxPinLoc << _USART_ROUTELOC0_RXLOC_SHIFT) |
                           (txPinLoc << _USART_ROUTELOC0_TXLOC_SHIFT);

        UARTx->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

        // Enable USART Interrupts
        NVIC_EnableIRQ(getUARTx_RX_IRQn());
        NVIC_EnableIRQ(getUARTx_TX_IRQn());
        USART_IntEnable(UARTx, USART_IEN_RXDATAV);
    }

    return 0;
}


int HW_HAL_UART::configRdDMA(size_t dmaRdSize){
    DMADRV_Init();

    if (rdDMACh == 9999){
        if (DMADRV_AllocateChannel(&rdDMACh, NULL) != ECODE_EMDRV_DMADRV_OK) {
            rdDMACh = 9999;
            return -1;
        }
    }

    if(dmaRdSize > UART_BUF_SIZE){
        DMAMaxReceiveSize = UART_BUF_SIZE;
    }else{
        DMAMaxReceiveSize = dmaRdSize;
    }
    dmaRdEnable = true;
    USART_IntDisable(UARTx, USART_IEN_RXDATAV);
    ReceiveIntoRxBufWithDMA();
    return 0;
}

int HW_HAL_UART::configWrDMA(){
  DMADRV_Init();

  if (wrDMACh == 9999){
      if (DMADRV_AllocateChannel(&wrDMACh, NULL) != ECODE_EMDRV_DMADRV_OK) {
          wrDMACh = 9999;
          return -1;
      }
  }
  dmaWrEnable = true;
  return 0;
}


void HW_HAL_UART::putchar(uint8_t c)
{
    if(idx == UART_IDX4){
        while(!(LEUART_StatusGet(LEUARTx) & LEUART_STATUS_TXC)) {}
        LEUART_Tx(LEUARTx, c);
    }else{
        while(!(USART_StatusGet(UARTx) & USART_STATUS_TXC)) {}
        USART_Tx(UARTx,c);
    }

}


void UART_putchar(HAL_UART* uart, uint8_t c)
{
	for(int i=UART_IDX_MIN;i<=UART_IDX_MAX;i++){
		if(UART_contextArray[i].hal_uart == uart){
			UART_contextArray[i].putchar(c);
			return;
		}
	}
}



int8_t HW_HAL_UART::getPinLoc(GPIO_PIN gpio, const uint8_t * lut){
    int i = 0;

    while(i < PIN_LOCATION_TABLE_SIZE){
        if(gpio == lut[i]){
            return i;
        }
        i++;
    }

    return -1;
}



/* pin location tables
 * - to save memory only one LUT for rx-, tx-, rts-, cts-, cs- and clk-pin is used
 *   -> this is possible because the 6 pin-location-tables are shifted by just one value
 *   -> each pin-location-table has 32 items
 *   -> we combined all 6 tables to one LUT with 37 items and use different starting points for each pin search
 */
const uint8_t usart0_1PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5] = {
    GPIO_000, GPIO_001, GPIO_002, GPIO_003, GPIO_004, GPIO_005, // PA1 ... PA5
    GPIO_027, GPIO_028, GPIO_029, GPIO_030, GPIO_031, // PB11 ... PB15
    GPIO_038, GPIO_039, GPIO_040, GPIO_041, GPIO_042, GPIO_043, // PC06 ... PC11
    GPIO_057, GPIO_058, GPIO_059, GPIO_060, GPIO_061, GPIO_062, GPIO_063, // PD09 ... PD15
    GPIO_080, GPIO_081, GPIO_082, GPIO_083, GPIO_084, GPIO_085, GPIO_086, GPIO_087, // PF00 ... PF07
    GPIO_000, GPIO_001, GPIO_002, GPIO_003, GPIO_004 // PA00 ... PA04 -> repeat first five elements to use LUT for all six pins
};

const uint8_t usart2PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5] = {
    GPIO_005, GPIO_006, GPIO_007, GPIO_008, GPIO_009, // PA5 ... PA9
    GPIO_128, GPIO_129, GPIO_130, GPIO_131, // PI0 ... PI3
    GPIO_022, GPIO_023, GPIO_024, GPIO_025, GPIO_026, // PB06 ... PB10
    GPIO_080, GPIO_081, GPIO_083, GPIO_084, GPIO_085, GPIO_086, GPIO_087, // PF00 ... PF07 (except PF02!)
    GPIO_088, GPIO_089, GPIO_090, GPIO_091, GPIO_092, GPIO_093, GPIO_094, GPIO_095, // PF08 ... PF15
    GPIO_160, GPIO_161, GPIO_162, // PK00 ... PK02
    GPIO_005, GPIO_006, GPIO_007, GPIO_008, GPIO_009 // PA05 ... PA09 -> repeat first five elements to use LUT for all six pins
};

const uint8_t usart3PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+5] = {
    GPIO_056, GPIO_057, GPIO_058, GPIO_059, GPIO_060, GPIO_061, GPIO_062, GPIO_063, // PD08 ... PD15
    GPIO_130, GPIO_131, // PI2 ... PI3
    GPIO_022, GPIO_023, GPIO_024, GPIO_025, GPIO_026, GPIO_027, // PB06 ... PB11
    GPIO_158, GPIO_159, // PJ14 ... PJ15
    GPIO_032, GPIO_033, GPIO_034, GPIO_035, GPIO_036, GPIO_037, // PC00 ... PC05
    GPIO_091, GPIO_092, GPIO_093, GPIO_094, GPIO_095, // PF11 ... PF15
    GPIO_160, GPIO_161, GPIO_162, // PK00 ... PK02
    GPIO_056, GPIO_057, GPIO_058, GPIO_059, GPIO_060 // PD08 ... PD12 -> repeat first five elements to use LUT for all six pins
};


}
