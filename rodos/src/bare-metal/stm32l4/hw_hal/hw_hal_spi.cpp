#include "hw_hal_spi.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripheral_ctrl/gpio/pin_group.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"
#include "rodos.h"

#include "peripherals.h"

namespace RODOS {

using namespace SPI_CR1;
using namespace SPI_CR2;
using namespace SPI_SR;

HW_HAL_SPI::HW_HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin)
  : idx(idx), spi(getSPIStruct(idx)), misoPin(misoPin), mosiPin(mosiPin), sckPin(sckPin), nssPin(nssPin) {
}

SPIStruct& HW_HAL_SPI::getSPIStruct(SPI_IDX idx) {
    return mcu_specific::SPIS::getPeriphStruct(idx);
}

int32_t HW_HAL_SPI::init(uint32_t baudrate, bool tiMode) {
    getRCCPeriph().enableClock();

    initSPIPin(misoPin);
    initSPIPin(mosiPin);
    initSPIPin(sckPin);
    if(nssPin != GPIO_INVALID) {
        initSPIPin(nssPin, GPIO::PULL::UP);
    }
    dummy_byte = 0x00;
    
    if(!setBaudrate(baudrate)) {
        return -1;
    }
    spi.CR1.set(MSTR(1));
    spi.CR2.set(FRF(tiMode), SSOE(1), FRXTH(1));

    return 0;
}

constexpr uint32_t getPrescalerFromCR1BR(uint8_t cr1BR){
    return 2ul << cr1BR;
}

void HW_HAL_SPI::printClockSetupError(uint32_t peripheralClock){
    RODOS_ERROR("SPI baudrate must be the APB frequency divided by a power of two\n");
    PRINTF("Note: APB frequency is %lu\n", static_cast<long unsigned>(peripheralClock));
}

bool HW_HAL_SPI::setBaudrate(uint32_t baudrate) {
    const uint32_t peripheralClock = (idx == SPI_IDX0) ? PCLK::freq1 : PCLK::freq2;
    const uint32_t requiredPrescaler = peripheralClock / baudrate;
    if((peripheralClock % baudrate) != 0) {
        printClockSetupError(peripheralClock);
        return false;
    }

    uint8_t cr1BR = 0;
    constexpr uint8_t maxCR1BR = 7u;
    while(getPrescalerFromCR1BR(cr1BR) != requiredPrescaler) {
        if(++cr1BR > maxCR1BR) {
            printClockSetupError(peripheralClock);
            return false;
        }
    }
    this->baudrate = baudrate;
    spi.CR1.set(BR(cr1BR));
    return true;
}

bool HW_HAL_SPI::setMode(uint8_t mode){
    constexpr uint8_t maxSPIMode = 3u;
    if(mode>maxSPIMode){
        return false;
    }
    spi.CR1.set(MODE(mode));
    return true;
}

bool HW_HAL_SPI::setMosiIdl(bool high){
    if(high){
        initSPIPin(mosiPin, GPIO::PULL::UP);
        dummy_byte = 0xff;
    }else{
        initSPIPin(mosiPin, GPIO::PULL::DOWN);
        dummy_byte = 0x00;
    }
    return true;
}

void HW_HAL_SPI::initSPIPin(GPIO_PIN pinIdx, GPIO::PULL pull) {
    GPIO::PinGroup pinGrp = GPIO::PinGroup(pinIdx);
    if(!pinGrp.isValid()) {
        RODOS_ERROR("HW_HAL_SPI: PinGroup instance is not valid (no port)\n");
        return;
    }
    pinGrp.enableClocks();
    pinGrp.setMode(GPIO::MODE::ALTERNATE);
    pinGrp.setAlternateFunction(getAlternateFunction(pinIdx));
    pinGrp.setPull(pull);
    pinGrp.setOutputType(GPIO::OUTPUTTYPE::PUSHPULL);
    pinGrp.setSpeed(GPIO::SPEED::HIGH);
}

HWRCCPeriph& HW_HAL_SPI::getRCCPeriph() {
    return mcu_specific::SPIS::getRCCPeriph(idx);
}

GPIO::ALTERNATEFUNCTION HW_HAL_SPI::getAlternateFunction(GPIO_PIN pin) {
    return mcu_specific::SPIS::getAlternateFunction(idx, (uint8_t)pin);
}

void HW_HAL_SPI::emptyRXFIFO() {
    using namespace SPI_SR;
    while(spi.SR.read<FRLVL>() > 0) {
        spi.DR.read8();
    }
}

void HW_HAL_SPI::waitUntilDataTransmitted() {
    using namespace SPI_SR;
    while(spi.SR.read<BSY>() > 0);
}

void HW_HAL_SPI::reset(){
    GPIO::PinGroup(misoPin).reset();
    GPIO::PinGroup(mosiPin).reset();
    GPIO::PinGroup(sckPin).reset();
    if(nssPin != GPIO_INVALID) {
        GPIO::PinGroup(nssPin).reset();
    }
    getRCCPeriph().reset();
    getRCCPeriph().releaseReset();
}

bool HW_HAL_SPI::writeRead(const void* sendBuf, size_t len, void* recBuf, size_t maxLen) {
    return writeReadTrig(sendBuf, len, recBuf, maxLen, 0, 0, false, false);
}

bool HW_HAL_SPI::writeReadTrig(const void* sendBuf, size_t len, void* recBuf, size_t maxLen, 
            const uint8_t trigger, const int64_t timeout, bool notTrig, bool syncd){
    using namespace SPI_SR;
    if((sendBuf==nullptr&&len!=0) || (recBuf==nullptr&&maxLen!=0) || ((timeout == 0) == syncd)){        
        return false;
    }
    const uint8_t* txBuf = static_cast<const uint8_t*>(sendBuf);
    uint8_t*       rxBuf = static_cast<uint8_t*>(recBuf);

    spi.CR1.set(SPE(1));
    uint8_t rxB = (uint8_t)(trigger+(uint8_t)1)%256;       //init rxB to a different value than the trigger
    bool error = false;
    int64_t timeout_end = NOW() + timeout;
    size_t bytesToTransmit = (len>maxLen) ? len : maxLen;
    bytesToTransmit = (bytesToTransmit>INT32_MAX) ? INT32_MAX : bytesToTransmit;
    {
        PRIORITY_CEILER_IN_SCOPE();
        if(syncd){                                      
            //send dummy bytes as long as the sync condition is not met
            bool stop=false;
            do{
                RegisterCopy sr = spi.SR;
                if(NOW() > timeout_end || sr.read<FRE>() || sr.read<MODF>()){
                    error = true;
                    break;
                }
                if(sr.read<TXE>())
                    spi.DR.write8(dummy_byte);
                if(sr.read<RXNE>())
                    rxB = spi.DR.read8();
                stop=(notTrig != (trigger == rxB));
            }while(!stop);
        }
        while(bytesToTransmit>0 || maxLen>0) {

            RegisterCopy sr = spi.SR;
            if(sr.read<FRE>() || sr.read<MODF>()){
                error = true;
                break;
            }
            if(sr.read<TXE>() && bytesToTransmit>0) {
                if(len > 0) {
                    len--;
                    spi.DR.write8(*(txBuf++));
                } else {
                    spi.DR.write8(dummy_byte);
                }
                bytesToTransmit--;
            }
            if(sr.read<RXNE>() && maxLen>0) {
                maxLen--;
                *(rxBuf++) = spi.DR.read8();
            }
        }
    }
    waitUntilDataTransmitted();
    emptyRXFIFO();
    spi.CR1.set(SPE(0));

    return !error;
}

}
