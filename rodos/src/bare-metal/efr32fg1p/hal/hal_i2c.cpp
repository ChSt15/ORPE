#include <new>
#include "hal/hal_i2c.h"
#include "rodos.h"
#include "timemodel.h"

//#include "hw_hal_gpio.h"
#include "hw_hal_i2c.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"

namespace RODOS {

  /* ToDo:
   * - test writeRead()
   * - timeout for while-loops?
   * - slave mode
   * - interrupts?
   */

//#ifndef I2C_TIMEOUT_IN_NS
//#define I2C_TIMEOUT_IN_NS  		(2*MILLISECONDS)
//#endif
//#ifndef I2C_LONG_TIMEOUT_IN_NS
//#define I2C_LONG_TIMEOUT_IN_NS  	(10*MILLISECONDS)
//#endif

HW_HAL_I2C HW_HAL_I2C::I2C_contextArray[I2C_IDX_MAX+1];

/*****************************************************************************************************************/
/*
 * class HW_HAL_I2C
 */

/* pin location tables
 * - to save memory only one LUT for scl- and sda-pin is used
 *   -> this is possible because the 2 pin-location-tables are shifted by just one value
 *   -> each pin-location-table has 32 items
 *   -> we combined all 2 tables to one LUT with 33 items and use different starting points for each pin search
 */
#define PIN_LOCATION_TABLE_SIZE   32

const uint8_t i2c0PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+1] = {
    GPIO_000, GPIO_001, GPIO_002, GPIO_003, GPIO_004, GPIO_005, // PA1 ... PA5
    GPIO_027, GPIO_028, GPIO_029, GPIO_030, GPIO_031, // PB11 ... PB15
    GPIO_038, GPIO_039, GPIO_040, GPIO_041, GPIO_042, GPIO_043, // PC06 ... PC11
    GPIO_057, GPIO_058, GPIO_059, GPIO_060, GPIO_061, GPIO_062, GPIO_063, // PD09 ... PD15
    GPIO_080, GPIO_081, GPIO_082, GPIO_083, GPIO_084, GPIO_085, GPIO_086, GPIO_087, // PF00 ... PF07
    GPIO_000 // PA00 -> repeat first element to use LUT for both pins
};

const uint8_t i2c1PinLoc_LUT[PIN_LOCATION_TABLE_SIZE+1] = {
    GPIO_006, GPIO_007, GPIO_008, GPIO_009, // PA6 ... PA9
    GPIO_130, GPIO_131, // PI2,PI3
    GPIO_022, GPIO_023, GPIO_024, GPIO_025, GPIO_026, // PB6 ... PB10
    GPIO_158, GPIO_159, // PJ14,PJ15
    GPIO_032, GPIO_033, GPIO_034, GPIO_035, GPIO_036, GPIO_037, // PC00 ... PC05
    GPIO_042, GPIO_043, // PC10 ... PC11
    GPIO_088, GPIO_089, GPIO_090, GPIO_091, GPIO_092, GPIO_093, GPIO_094, GPIO_095, // PF08 ... PF15
    GPIO_160, GPIO_161, GPIO_162, // PK00 ... PK02
    GPIO_006 // PA06 -> repeat first element to use LUT for both pins
};

void HW_HAL_I2C::initMembers(GPIO_PIN sclPin, GPIO_PIN sdaPin){
  isMaster = false;
  initialized = false;

  switch(idx) {
  case I2C_IDX0:
    I2Cx = I2C0;
    break;
  case I2C_IDX1:
    I2Cx = I2C1;
    break;
  default:
    break;
  }

  GPIO_Pin_SCL = sclPin;
  GPIO_Pin_SDA = sdaPin;
}

int8_t HW_HAL_I2C::getPinLoc(GPIO_PIN gpio, const uint8_t * lut){
    int i = 0;

    while(i < PIN_LOCATION_TABLE_SIZE){
        if(gpio == lut[i]){
            return i;
        }
        i++;
    }

    return -1;
}

int32_t HW_HAL_I2C::enableClocks(){
#if defined(_CMU_HFPERCLKEN0_MASK)
    CMU_ClockEnable(cmuClock_HFPER, true);
#endif

    CMU_ClockEnable(cmuClock_GPIO, true);

    switch(idx){
      case I2C_IDX0:
        CMU_ClockEnable(cmuClock_I2C0, true);
        break;

      case I2C_IDX1:
        CMU_ClockEnable(cmuClock_I2C1, true);
        break;

      default:
        RODOS_ERROR("I2C index out of range");
        return -1;
    }

    return 0;
}
/*****************************************************************************************************************/





/*****************************************************************************************************************/
HAL_I2C::HAL_I2C(I2C_IDX idx) {
    if (idx < I2C_IDX_MIN || idx > I2C_IDX_MAX) {
        RODOS_ERROR("I2C index out of range");
    } else {
        context = new (&HW_HAL_I2C::I2C_contextArray[idx]) HW_HAL_I2C(idx); // placement new to avoid dynamic memory allocation
    }

    switch (idx){
      case I2C_IDX0:
        context->initMembers(GPIO_042, GPIO_043); // SCL = PC10, SDA = PC11 (EvalBoard-exp-hdr: pin 15/16)
        break;
      case I2C_IDX1:
        context->initMembers(GPIO_022, GPIO_023); // SCL = PB06, SDA = PB07 (EvalBoard-exp-hdr: pin 12/14)
        break;
      default:
        break;
    }
}

HAL_I2C::HAL_I2C(I2C_IDX idx, GPIO_PIN sclPin, GPIO_PIN sdaPin) {
    if (idx < I2C_IDX_MIN || idx > I2C_IDX_MAX) {
      RODOS_ERROR("I2C index out of range");
    } else {
        context = new (&HW_HAL_I2C::I2C_contextArray[idx]) HW_HAL_I2C(idx); // placement new to avoid dynamic memory allocation
        context->initMembers(sclPin, sdaPin);
    }
}



int32_t HAL_I2C::init(uint32_t speed) {

    context->enableClocks();

    //RODOS_ASSERT_IFNOT_RETURN((speed > 0 && speed <= 20000000), -1);
    //context->I2C_SPEED = speed;

    /* configure GPIOs */
    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCL), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCL), gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SDA), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SDA), gpioModeWiredAndPullUpFilter, 1);

     /* In some situations, after a reset during an I2C transfer, the slave
    device may be left in an unknown state. Send 9 clock pulses to
    set slave in a defined state. */
    int i = 0;
    for (i = 0; i < 9; i++) {
        GPIO_PinOutSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCL), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCL));
        GPIO_PinOutClear(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCL), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCL));
    }

    for (i = 0; i < 20; i++) {
        GPIO_PinOutSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCL), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCL));
    }

    /* pin routing */
    int8_t sclPinLoc = 0;
    int8_t sdaPinLoc = 0;

    switch(context->idx){
      case I2C_IDX0:
        sdaPinLoc = context->getPinLoc(context->GPIO_Pin_SDA, &i2c0PinLoc_LUT[0]);
        sclPinLoc = context->getPinLoc(context->GPIO_Pin_SCL, &i2c0PinLoc_LUT[1]);
        break;

      case I2C_IDX1:
        sdaPinLoc = context->getPinLoc(context->GPIO_Pin_SDA, &i2c1PinLoc_LUT[0]);
        sclPinLoc = context->getPinLoc(context->GPIO_Pin_SCL, &i2c1PinLoc_LUT[1]);
        break;

      default:
        RODOS_ERROR("I2C index out of range");
        return -1;
    }

    if (sclPinLoc <0 || sdaPinLoc <0){
        RODOS_ERROR("I2C pin invalid");
        I2C_Reset(context->I2Cx);
        return -1;
    }

    context->I2Cx->ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;

    // Route GPIO pins to I2C module
    context->I2Cx->ROUTELOC0 =  (sclPinLoc << _I2C_ROUTELOC0_SCLLOC_SHIFT) |
                                (sdaPinLoc << _I2C_ROUTELOC0_SDALOC_SHIFT);

    // Initializing the I2C
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT; // Using default settings
    i2cInit.freq = speed;
    I2C_Init(context->I2Cx, &i2cInit);

    context->isMaster = true;
    context->initialized = true;

    //PRINTF("I2C peripheral clock: %d\n", CMU_ClockFreqGet(cmuClock_I2C0));
    //PRINTF("I2C clock: %d\n", I2C_BusFreqGet(context->I2Cx));

    return 0;
}


void HAL_I2C::reset() {
    context->isMaster = false;
    context->initialized = false;

    if (context->I2Cx == NULL) return;

    I2C_Reset(context->I2Cx);

    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SCL), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SCL), gpioModeDisabled, 0);
    GPIO_PinModeSet(HW_HAL_GPIO::getEFR32Port(context->GPIO_Pin_SDA), HW_HAL_GPIO::getEFR32Pin(context->GPIO_Pin_SDA), gpioModeDisabled, 0);
}


bool HAL_I2C::isWriteFinished(){
    return true;
}


bool HAL_I2C::isReadFinished(){
    return true;
}


int32_t HAL_I2C::write(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize) {
  if (!context->initialized) return -1;
 	if (txBufSize == 0) return 0;

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initializing I2C transfer
  i2cTransfer.addr          = addr<<1;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = (uint8_t*)txBuf;
  i2cTransfer.buf[0].len    = txBufSize;
  i2cTransfer.buf[1].data   = NULL;
  i2cTransfer.buf[1].len    = 0;

  result = I2C_TransferInit(context->I2Cx, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(context->I2Cx);
  }

  if(result != i2cTransferDone) {
      PRINTF("I2C write error: %d\n", result);
      return -1;
  }


  upCallWriteFinished();

  return txBufSize;

//	if (context->IsMaster) {
//	    int32_t retVal = context->mstrWrite(addr,txBuf,txBufSize);
//	    upCallWriteFinished();
//	    return retVal;
//	} else {
//	    upCallWriteFinished();
//		return HAL_I2C_ERR_NOT_MASTER;
//	}
}


int32_t HAL_I2C::read(const uint8_t addr, uint8_t* rxBuf, uint32_t rxBufSize) {
  if (!context->initialized) return -1;
	if (rxBufSize == 0) return 0;

	// Transfer structure
	  I2C_TransferSeq_TypeDef i2cTransfer;
	  I2C_TransferReturn_TypeDef result;

	  // Initializing I2C transfer
	  i2cTransfer.addr          = addr<<1;
	  i2cTransfer.flags         = I2C_FLAG_READ; // must write target address before reading
	  i2cTransfer.buf[0].data   = rxBuf;
	  i2cTransfer.buf[0].len    = rxBufSize;
	  i2cTransfer.buf[1].data   = NULL;
	  i2cTransfer.buf[1].len    = 0;

	  result = I2C_TransferInit(context->I2Cx, &i2cTransfer);

	  // Sending data
	  while (result == i2cTransferInProgress)
	  {
	    result = I2C_Transfer(context->I2Cx);
	  }

	  if(result != i2cTransferDone)
	  {
	      PRINTF("I2C read error: %d\n", result);
	      return -1;
	  }

	  upCallReadFinished();

	  return rxBufSize;

//	if (context->IsMaster) {
//		int32_t retVal = context->mstrRead(addr,rxBuf,rxBufSize);
//	    upCallReadFinished();
//	    return retVal;
//	} else {
//	    upCallReadFinished();
//		return HAL_I2C_ERR_NOT_MASTER;
//	}
}


int32_t HAL_I2C::writeRead(const uint8_t addr, const uint8_t* txBuf, uint32_t txBufSize, uint8_t* rxBuf, uint32_t rxBufSize) {
  if (!context->initialized) return -1;

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initializing I2C transfer
  i2cTransfer.addr          = addr<<1;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading
  i2cTransfer.buf[0].data   = (uint8_t*)txBuf;
  i2cTransfer.buf[0].len    = txBufSize;
  i2cTransfer.buf[1].data   = rxBuf;
  i2cTransfer.buf[1].len    = rxBufSize;

  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }

  if(result != i2cTransferDone)
  {
      PRINTF("I2C writeRead: %d\n", result);
      return -1;
  }

  upCallReadFinished();
  return rxBufSize;

//  int32_t txRetVal = 0;
//  int32_t rxRetVal = 0;
//	if (context->IsMaster){
//		txRetVal = context->mstrWriteNoStop(addr, txBuf, txBufSize);
//		if (txRetVal < 0) {
//		    upCallReadFinished();
//			return txRetVal;
//		}
//
//		/* send Re-Start  */
//		I2C_GenerateSTART(context->I2Cx, ENABLE);
//
//		/* Test on EV5 and clear it */
//		TTime startTime = NOW();
//		while (I2C_CheckEvent(context->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
//			if ((NOW()-startTime) > I2C_TIMEOUT_IN_NS){
//			    upCallReadFinished();
//				return HAL_I2C_ERR_START;
//			}
//		}
//
//		rxRetVal = context->mstrReadNoStart(addr, rxBuf, rxBufSize);
//		if (rxRetVal < 0) {
//		    upCallReadFinished();
//			return rxRetVal;
//		}
//	}else{
//	    upCallReadFinished();
//		return HAL_I2C_ERR_NOT_MASTER;
//	}
//
//	upCallReadFinished();
//	return rxRetVal;
}


} // namespace RODOS

