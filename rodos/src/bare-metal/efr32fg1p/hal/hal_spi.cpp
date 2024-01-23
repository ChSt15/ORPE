/**
 * @file    hal_spi.h
 *
 * @date    01.07.2021
 * @author  Michael Ruffer
 *
 * @brief   Simple API to configure and use SPI interfaces
 */

#include <new>
#include "rodos.h"
#include "hal/hal_spi.h"
#include "hw_hal_gpio.h"
#include "hw_hal_uart.h" // necessary for pin location table


#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
//#include "em_usart.h"
#include "spidrv.h"

namespace RODOS {
/** TODO:
 * - Konflikt LDMA_IRQHandler mit hal_uart auflösen
 * - Semaphore für USART/SPI-Instanzen
 * - bei DMA Transfer wird der Sendepuffer nicht kopiert
 *   -> man muss sicherstellen, dass der Puffer bis zum Ende gültig ist!!!
 * - read/write not "busy-waiting" -> interrupts & suspend
 * - slave-mode
 * - use of autoCS-pin
 */

  /* driver features
   * - default polarity/phase = 0/0 -> clk is idle low, sample on rising edge
   * - TI mode not supported
   * - pin configuration with constructor
   * - baudrate and mode configuration with config()
   * - default: blocking & DMA disabled
   * - not supported yet: DMA disabled & non-blocking
   */

  /* default pin configurations
   * SPI0: SCK = PA2 (LOC0), MISO = PA1 (LOC0), MOSI = PA0 (LOC0)
   * SPI1: SCK = PA5 (LOC3), MISO = PA4 (LOC3), MOSI = PA3 (LOC3)
   * SPI2: SCK = PF5 (LOC16), MISO = PF4 (LOC16), MOSI = PF3 (LOC16)
   * SPI3: SCK = PC2 (LOC18), MISO = PC1 (LOC18), MOSI = PC0 (LOC18)
   */

  // chip selects on "innocube eval board for radiation tests"
  //CS = PC10(GPIO_042)  -TO COMUNICATE WITH RAM
  //CS = PC11(GPIO_043)  -TO COMUNICATE WITH FLASH1
  //CS = PB9 (GPIO_025)  -TO COMUNICATE WITH FLASH2
  //CS = PB10(GPIO_026)  -TO COMUNICATE WITH FLASH3
  //
  //CS = PC3    - TO COMUNICATE WITH LSM9DS1  -   accelerometer and gyroscope
  //CS = PC4   - TO COMUNICATE WITH LSM9DS1  -     magnetometer



#define SPI_IDX_MIN SPI_IDX0
#define SPI_IDX_MAX SPI_IDX3

class HW_HAL_SPI {
  public:
    SPI_IDX idx;
    bool initialized;
    USART_TypeDef *SPIx;
    HAL_SPI* halSpi;

    uint32_t baudrate;

    SPIDRV_HandleData_t spidrv_handle_data;
    SPIDRV_Handle_t spidrv_handle;

    GPIO_PIN GPIO_Pin_MISO;
    GPIO_PIN GPIO_Pin_MOSI;
    GPIO_PIN GPIO_Pin_SCK;
    GPIO_PIN GPIO_Pin_NSS;

    int8_t misoPinLoc;
    int8_t mosiPinLoc;
    int8_t sckPinLoc;

    bool flagTxComplete;
    bool flagRxComplete;

    uint8_t * remainingBuf;
    uint32_t remainingSize;

    bool dmaEnable;
    bool blockingEnable;

  public:
    HW_HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin);
    HW_HAL_SPI(SPI_IDX idx);

    HW_HAL_SPI() {
      // this constructor is called to init SPIcontextArray-objects
      // !!! don't set member variables -> this might overwrite values already set by constructor call of placement new
    }

    ~HW_HAL_SPI(){}

    void waitOnTransferReady();

    int32_t setBaudrate(uint32_t baudrate);
    int32_t enableClocks();

    void initMembers(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin = GPIO_INVALID);

    int DMAinit();

    static HW_HAL_SPI* getHW_HAL_SPI(struct SPIDRV_HandleData *handle);

    /* writeRead() related functions */
    static void startTransmitRemaining(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred);
    static void startReceiveRemaining(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred);
    static void txComplete(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred);
    static void rxComplete(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred);
    static void txRxComplete(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred);
};



HW_HAL_SPI::HW_HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin) { 
  RODOS_ASSERT(idx >= SPI_IDX_MIN); // SPI index out of range
  RODOS_ASSERT(idx <= SPI_IDX_MAX); // SPI index out of range
  initMembers(idx, sckPin, misoPin, mosiPin, nssPin);
}



HW_HAL_SPI::HW_HAL_SPI(SPI_IDX idx) {

	switch(idx){
	  case SPI_IDX0: // USART0
	    initMembers(idx, GPIO_002, GPIO_001, GPIO_000 ); // SCK = PA2 (LOC0), MISO = PA1 (LOC0), MOSI = PA0 (LOC0)
	    break;

	  case SPI_IDX1: // USART1
	    initMembers(idx, GPIO_005, GPIO_004, GPIO_003 );  // SCK = PA5 (LOC3), MISO = PA4 (LOC3), MOSI = PA3 (LOC3)
	    break;

	  case SPI_IDX2: // USART2
	    initMembers(idx,GPIO_085 ,GPIO_084, GPIO_083 );   // SCK = PF5 (LOC16), MISO = PF4 (LOC16), MOSI = PF3 (LOC16)
	    break;

	  case SPI_IDX3: // USART3
	    initMembers(idx, GPIO_034, GPIO_033, GPIO_032 );  // SCK = PC2 (LOC18), MISO = PC1 (LOC18), MOSI = PC0 (LOC18)
	    break;

	default:
	  //RODOS_ERROR("SPI index out of range");  !!! RODOS_ERROR-call hangs constructor call !!!
	  break;
	}
}



void HW_HAL_SPI::initMembers(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin) {

  this->flagTxComplete = true;
  this->flagRxComplete = true;
  this->initialized = false;
	this->idx = idx;
	this->dmaEnable = false;
	this->blockingEnable = true;

  SPIx = NULL;
  spidrv_handle = &spidrv_handle_data;

	switch(idx){
	  case SPI_IDX0:
	    SPIx = USART0;
	    break;

	  case SPI_IDX1:
	    SPIx = USART1;
	    break;

	  case SPI_IDX2:
      SPIx = USART2;
  		break;

	  case SPI_IDX3:
      SPIx = USART3;
  		break;

	  default:
	    //RODOS_ERROR("SPI index out of range");  !!! RODOS_ERROR-call hangs constructor call !!!
	    return;
	}

  GPIO_Pin_SCK    = sckPin;
	GPIO_Pin_MISO   = misoPin;
	GPIO_Pin_NSS    = nssPin;
	GPIO_Pin_MOSI   = mosiPin;

  /* pin routing
   * - to save memory only one LUT for rx-,tx- and clk-pin is used
   *   -> this is possible because the 3 pin-location-tables are shifted by just one value
   *   -> each pin-location-table has 32 items
   *   -> we combined all 3 tables to one LUT with 34 items and use different starting points for each pin search
   */
  switch(idx){
    case SPI_IDX0:
      mosiPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MOSI, &usart0_1PinLoc_LUT[0]);
      misoPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MISO, &usart0_1PinLoc_LUT[1]);
      sckPinLoc  = HW_HAL_UART::getPinLoc(GPIO_Pin_SCK, &usart0_1PinLoc_LUT[2]);
      break;

    case SPI_IDX1:
      mosiPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MOSI, &usart0_1PinLoc_LUT[0]);
      misoPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MISO, &usart0_1PinLoc_LUT[1]);
      sckPinLoc  = HW_HAL_UART::getPinLoc(GPIO_Pin_SCK, &usart0_1PinLoc_LUT[2]);
      break;

    case SPI_IDX2:
      mosiPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MOSI, &usart2PinLoc_LUT[0]);
      misoPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MISO, &usart2PinLoc_LUT[1]);
      sckPinLoc  = HW_HAL_UART::getPinLoc(GPIO_Pin_SCK, &usart2PinLoc_LUT[2]);
      break;

    case SPI_IDX3:
      mosiPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MOSI, &usart3PinLoc_LUT[0]);
      misoPinLoc = HW_HAL_UART::getPinLoc(GPIO_Pin_MISO, &usart3PinLoc_LUT[1]);
      sckPinLoc  = HW_HAL_UART::getPinLoc(GPIO_Pin_SCK, &usart3PinLoc_LUT[2]);
      break;

    default:
      //RODOS_ERROR("SPI index out of range");  !!! RODOS_ERROR-call hangs constructor call !!!
      return;
  }

  if (misoPinLoc <0 || mosiPinLoc <0 || sckPinLoc < 0){
      //RODOS_ERROR("SPI pin invalid!!!");  !!! RODOS_ERROR-call hangs constructor call !!!
  }
}



int32_t HW_HAL_SPI::setBaudrate(uint32_t baudrate){

    if (SPIx == NULL){return -1;}

    SPIDRV_SetBitrate(spidrv_handle, baudrate);

    return 0;
}



int32_t HW_HAL_SPI::enableClocks(){
#if defined(_CMU_HFPERCLKEN0_MASK)
    CMU_ClockEnable(cmuClock_HFPER, true);
#endif

    CMU_ClockEnable(cmuClock_GPIO, true);

    switch(idx){
      case SPI_IDX0:
        CMU_ClockEnable(cmuClock_USART0, true);
        break;

      case SPI_IDX1:
        CMU_ClockEnable(cmuClock_USART1, true);
        break;

      case SPI_IDX2:
        CMU_ClockEnable(cmuClock_USART2, true);
        break;

      case SPI_IDX3:
        CMU_ClockEnable(cmuClock_USART3, true);
        break;

      default:
        RODOS_ERROR("SPI index out of range");
        return -1;
    }

    return 0;
}



static HW_HAL_SPI SPIcontextArray[SPI_IDX_MAX+1];

HW_HAL_SPI* HW_HAL_SPI::getHW_HAL_SPI(struct SPIDRV_HandleData *handle){
  HW_HAL_SPI* hwHalSpi = NULL;

  if (handle->peripheral.usartPort == USART0) {hwHalSpi = &SPIcontextArray[0];}
  if (handle->peripheral.usartPort == USART1) {hwHalSpi = &SPIcontextArray[1];}
  if (handle->peripheral.usartPort == USART2) {hwHalSpi = &SPIcontextArray[2];}
  if (handle->peripheral.usartPort == USART3) {hwHalSpi = &SPIcontextArray[3];}

  return hwHalSpi;
}



void HW_HAL_SPI::startTransmitRemaining(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred){
    (void)&handle;
    (void)itemsTransferred;
    HW_HAL_SPI* spiContext = HW_HAL_SPI::getHW_HAL_SPI(handle);

    if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
        if (spiContext) {SPIDRV_MTransmit(handle, spiContext->remainingBuf, spiContext->remainingSize, HW_HAL_SPI::txRxComplete);}
    }
}



void HW_HAL_SPI::startReceiveRemaining(struct SPIDRV_HandleData *handle, Ecode_t transferStatus, int itemsTransferred){
    (void)&handle;
    (void)itemsTransferred;
    HW_HAL_SPI* spiContext = HW_HAL_SPI::getHW_HAL_SPI(handle);

    if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
        if (spiContext) {SPIDRV_MReceive(handle, spiContext->remainingBuf, spiContext->remainingSize, HW_HAL_SPI::txRxComplete);}
    }
}



void HW_HAL_SPI::txComplete(struct SPIDRV_HandleData *handle,
                      Ecode_t transferStatus,
                      int itemsTransferred){
    (void)&handle;
    (void)itemsTransferred;
    HW_HAL_SPI* spiContext = HW_HAL_SPI::getHW_HAL_SPI(handle);

    if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
        if (spiContext) spiContext->flagTxComplete = true;
        spiContext->halSpi->upCallWriteFinished();
    }
}



void HW_HAL_SPI::rxComplete(struct SPIDRV_HandleData *handle,
                      Ecode_t transferStatus,
                      int itemsTransferred){
    (void)&handle;
    (void)itemsTransferred;
    HW_HAL_SPI* spiContext = HW_HAL_SPI::getHW_HAL_SPI(handle);

    if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
        if (spiContext) spiContext->flagRxComplete = true;
        spiContext->halSpi->upCallWriteFinished();
        spiContext->halSpi->upCallReadFinished();
    }
}



void HW_HAL_SPI::txRxComplete(struct SPIDRV_HandleData *handle,
                      Ecode_t transferStatus,
                      int itemsTransferred){
    (void)&handle;
    (void)itemsTransferred;
    HW_HAL_SPI* spiContext = HW_HAL_SPI::getHW_HAL_SPI(handle);

    if (transferStatus == ECODE_EMDRV_SPIDRV_OK) {
        if (spiContext) spiContext->flagTxComplete = true;
        if (spiContext) spiContext->flagRxComplete = true;
        spiContext->halSpi->upCallWriteFinished();
        spiContext->halSpi->upCallReadFinished();
    }
}

int HW_HAL_SPI::DMAinit(){
    SPIDRV_Init_t initData;

    initData.port              = SPIx;//SL_SPIDRV_EXP_PERIPHERAL;
    initData.portLocationTx    = mosiPinLoc;//SL_SPIDRV_EXP_TX_LOC;
    initData.portLocationRx    = misoPinLoc;//SL_SPIDRV_EXP_RX_LOC;
    initData.portLocationClk   = sckPinLoc;//SL_SPIDRV_EXP_CLK_LOC;
    initData.portLocationCs    = 0;//SL_SPIDRV_EXP_CS_LOC;
    initData.bitRate           = baudrate;//SL_SPIDRV_EXP_BITRATE;
    initData.frameLength       = 8;
    initData.dummyTxValue      = 0;
    initData.type              = spidrvMaster;//SL_SPIDRV_EXP_TYPE;
    initData.bitOrder          = spidrvBitOrderMsbFirst;//SL_SPIDRV_EXP_BIT_ORDER;
    initData.clockMode         = spidrvClockMode0;//SL_SPIDRV_EXP_CLOCK_MODE;
    initData.csControl         = spidrvCsControlApplication;//SL_SPIDRV_EXP_CS_CONTROL;
    initData.slaveStartMode    = spidrvSlaveStartImmediate;//SL_SPIDRV_EXP_SLAVE_START_MODE;

    Ecode_t retVal = SPIDRV_Init(spidrv_handle, &initData);

    if ( retVal == ECODE_OK){
        initialized = true;
        return 0;
    }

    return retVal;
}
/************************* end HW_HAL_SPI *************************************/






/********************************  HAL_SPI ************************************/



HAL_SPI::HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin) {
	if (idx < SPI_IDX_MIN || idx > SPI_IDX_MAX) {
		RODOS_ERROR("SPI index out of range");
	} else {
		context = new (&SPIcontextArray[idx]) HW_HAL_SPI(idx, sckPin, misoPin, mosiPin, nssPin);
		context->halSpi = this;
	}
}



HAL_SPI::HAL_SPI(SPI_IDX idx) {
	if (idx < SPI_IDX_MIN || idx > SPI_IDX_MAX) {
		RODOS_ERROR("SPI index out of range");
	} else {
		context = new (&SPIcontextArray[idx]) HW_HAL_SPI(idx);
		context->halSpi = this;
	}
}



/** init SPI interface
 * @param baudrate
 * @return 0 on success
 */
int32_t HAL_SPI::init(uint32_t baudrate, bool slave, bool tiMode) {       
    (void)tiMode;
    (void)slave;

    context->enableClocks();

    context->baudrate = baudrate;

    USART_InitSync_TypeDef initData = USART_INITSYNC_DEFAULT;
    initData.baudrate      = baudrate;
    initData.master        = true;
    initData.autoCsEnable  = false;
    initData.clockMode     = usartClockMode0; // don't change this value -> use config()
    initData.msbf          = true;
    initData.enable        = usartEnable;
    // USART_InitSync resets complete USART module
    // -> special register-settings (like "ROUTELOC") must
    //    be set afterwards!!!
    USART_InitSync(context->SPIx, &initData);

    context->SPIx->ROUTELOC0 =  (context->misoPinLoc << _USART_ROUTELOC0_RXLOC_SHIFT) |
                                (context->mosiPinLoc << _USART_ROUTELOC0_TXLOC_SHIFT) |
                                (context->sckPinLoc << _USART_ROUTELOC0_CLKLOC_SHIFT);

  	context->SPIx->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

    /* configure GPIOs */
  	GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCK), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCK), gpioModePushPull, 0);
  	GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_MOSI), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_MOSI), gpioModePushPull, 0);
  	GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_MISO), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_MISO), gpioModeInput, 0); // "out"-value sets filter on/off

    context->initialized = true;
    return 0;
}



/* writeRead() */
int32_t HAL_SPI::writeRead(const void* sendBuf, size_t len, void* recBuf, size_t maxLen) {
	  if(!context->initialized) return -1;
	
	  if (context->dmaEnable){
	      Ecode_t retVal = ECODE_OK;

	      if (len <= 0 || maxLen <= 0){
	          return -1;
	      }

	      context->flagRxComplete = false;
	      context->flagTxComplete = false;

	      if (len > maxLen){
	          context->remainingBuf = &((uint8_t*)sendBuf)[maxLen];
	          context->remainingSize = len - maxLen;
	          retVal = SPIDRV_MTransfer(context->spidrv_handle, sendBuf, recBuf, maxLen, HW_HAL_SPI::startTransmitRemaining);
	      }else if (len < maxLen){
	          context->remainingBuf = &((uint8_t*)recBuf)[len];
	          context->remainingSize = maxLen - len;
	          retVal = SPIDRV_MTransfer(context->spidrv_handle, sendBuf, recBuf, len, HW_HAL_SPI::startReceiveRemaining);
	      }else{ // len == maxLen
	          retVal = SPIDRV_MTransfer(context->spidrv_handle, sendBuf, recBuf, len, HW_HAL_SPI::txRxComplete);
	      }

	      if(context->blockingEnable){
	          suspendUntilWriteFinished(NOW()+100*MILLISECONDS);
	      }

	      return retVal;
	  }else{
        uint8_t *pTx = (uint8_t*)sendBuf;
        uint8_t *pRx = (uint8_t*)recBuf;
        uint8_t rxVal = 0;
        size_t rxCnt = maxLen;

        /* transmit tx buffer and receive at the same time */
        unsigned int i = 0;
        for(i = 0; i < len; i++){
            rxVal = USART_SpiTransfer(context->SPIx, *pTx);
            pTx++;
            if (rxCnt) {
                *pRx = rxVal;
                pRx++;
                rxCnt--;
            }
        }

        /* generate SCLK by sending dummy bytes to complete read request */
        while (rxCnt){
            *pRx = USART_SpiTransfer(context->SPIx, 0);
            pRx++;
            rxCnt--;
        }

        context->flagRxComplete = true;
        context->flagTxComplete = true;
        upCallWriteFinished();
        upCallReadFinished();

        return maxLen;
	  }
}



int32_t HAL_SPI::write(const void* sendBuf, size_t len) {

    if(!context->initialized) return -1;

    if (context->dmaEnable){
        context->flagTxComplete = false;
        SPIDRV_MTransmit(context->spidrv_handle, sendBuf, len, HW_HAL_SPI::txComplete);

        if(context->blockingEnable){
            suspendUntilWriteFinished(NOW()+100*MILLISECONDS);
            return len;
        }
        return 0;
    }else{
        uint8_t *pTx = (uint8_t*)sendBuf;

        /* transmit tx buffer */
        unsigned int i = 0;
        for(i = 0; i < len; i++){
            USART_SpiTransfer(context->SPIx, *pTx);
            pTx++;
        }

        context->flagTxComplete = true;
        upCallWriteFinished();

        return len;
    }
}


int32_t HAL_SPI::read(void* recBuf, size_t maxLen) {

	  if(!context->initialized) return -1;
 
	  if (context->dmaEnable){
	      context->flagRxComplete = false;
	      SPIDRV_MReceive(context->spidrv_handle, recBuf, maxLen, HW_HAL_SPI::rxComplete);

	      if(context->blockingEnable){
	          suspendUntilReadFinished(NOW()+100*MILLISECONDS);
	          return maxLen;
	      }

	      return 0;
	  }else{
	      uint8_t *pRx = (uint8_t*)recBuf;
	      size_t rxCnt = maxLen;

	      /* generate SCLK by sending dummy bytes */
	      while (rxCnt){
	          *pRx = USART_SpiTransfer(context->SPIx, 0);
	          pRx++;
	          rxCnt--;
	      }

	      context->flagRxComplete = true;
	      upCallReadFinished();

	      return maxLen;
	  }
}


int32_t HAL_SPI::config(SPI_PARAMETER_TYPE type, int32_t value){
    if(!context->initialized) return -1;

    switch (type){
    case SPI_PARAMETER_BAUDRATE:
        if ( (value > 0) && isWriteFinished() && isReadFinished()){
            USART_Enable(context->SPIx, usartDisable);
            context->setBaudrate(static_cast<uint32_t>(value));
            USART_Enable(context->SPIx, usartEnable);
            return 0;
        }
        return -1;

    case SPI_PARAMETER_MODE:
      // MODE:CPOL/CPHA  0:0/0   1:0/1   2:1/0   3:1/1
      if (value >= 0 && value <= 3){
          USART_Enable(context->SPIx, usartDisable);

          uint32_t cr1 = context->SPIx->CTRL;
          cr1 = cr1 & ~(_USART_CTRL_CLKPHA_MASK | _USART_CTRL_CLKPOL_MASK); // clear CPOL & CPHA
          uint32_t cpha = value & 1;
          uint32_t cpol = (value >> 1) & 1;
          cr1 = cr1 | (cpha << _USART_CTRL_CLKPHA_SHIFT) | (cpol << _USART_CTRL_CLKPOL_SHIFT);   // set CPHA & CPOL
          context->SPIx->CTRL = cr1;

          GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCK), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCK), gpioModePushPull, cpol);

          USART_Enable(context->SPIx, usartEnable);
          return 0;
      }else{
          return -1;
      }

    case SPI_PARAMETER_DMA:
      if (value > 0){
          context->DMAinit();
          context->dmaEnable = true;
      }else{
          init();
          context->dmaEnable = false;
      }
      return 0;

    case SPI_PARAMETER_BLOCKING:
      if (value > 0){
          context->blockingEnable = true;
      }else{
          context->blockingEnable = false;
      }
      return 0;

    default:
        return -1;
    }
}

void HAL_SPI::reset(){
    if (context->SPIx == NULL) return;

    USART_Reset(context->SPIx);
    context->initialized = false;

    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCK), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCK), gpioModeDisabled, 0);
    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_MOSI), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_MOSI), gpioModeDisabled, 0);
    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_MISO), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_MISO), gpioModeDisabled, 0);

    /* disable peripheral clock */
    switch(context->idx){
      case SPI_IDX0:
        CMU_ClockEnable(cmuClock_USART0, false);
        break;

      case SPI_IDX1:
        CMU_ClockEnable(cmuClock_USART1, false);
        break;

      case SPI_IDX2:
        CMU_ClockEnable(cmuClock_USART2, false);
        break;

      case SPI_IDX3:
        CMU_ClockEnable(cmuClock_USART3, false);
        break;

      default:
        RODOS_ERROR("SPI index out of range");
        return;
    }
}

int32_t HAL_SPI::status(SPI_STATUS_TYPE type) {
    if(!context->initialized) return -1;

    switch (type){
    case SPI_STATUS_BAUDRATE:
        //return static_cast<int32_t>(context->baudrate);
        return USART_BaudrateGet(context->SPIx);
    default:
        return -1;
    }
}

bool HAL_SPI::isWriteFinished(){
	return context->flagTxComplete;
}

bool HAL_SPI::isReadFinished(){
	return context->flagRxComplete;
}

}


