#include "rodos.h"
#include "hal/hal_spi.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"
#include "peripheral_ctrl/gpio/pin_group.h"

#include "peripherals.h"

namespace RODOS {

class HW_HAL_SPI {
private:
    const SPI_IDX idx;
    bool initialized;
    SPIStruct& spi;

    const HAL_SPI* owner;

    uint8_t dummy_byte = 0x00;

    GPIO_PIN misoPin;
    GPIO_PIN mosiPin;
    GPIO_PIN sckPin;
    GPIO_PIN nssPin;

    GPIO::ALTERNATEFUNCTION getAlternateFunction(GPIO_PIN pin);
    HWRCCPeriph& getRCCPeriph();
    void initSPIPin(GPIO_PIN pinIdx, GPIO::PULL pull = GPIO::PULL::NONE);

    void waitUntilDataTransmitted();
    void emptyRXFIFO();

    static SPIStruct& getSPIStruct(const SPI_IDX idx);
    static void printClockSetupError(uint32_t peripheralClock);
public:
    uint32_t baudrate;

    HW_HAL_SPI(SPI_IDX idx, GPIO_PIN sckPin, GPIO_PIN misoPin, GPIO_PIN mosiPin, GPIO_PIN nssPin);
    int32_t init(uint32_t baudrate, bool tiMode);
    bool setMode(uint8_t mode);
    bool setBaudrate(uint32_t baudrate);
    bool setMosiIdl(bool high);
    void reset();


    bool writeRead(const void* sendBuf, size_t txLen, void* recBuf, size_t rxLen);

    /**
     *  @brief  Write or/and read from the SPI connection. 
     *          This function provides the possibility to wait until
     *          a certain Byte has / hasn't been read from the miso line. 
     *          This is useful, when waiting for a busy signal to end or 
     *          the client to be able to receive new commands.
     *  @param  sendBuf the buffer, where the data to be send is stored
     *  @param  txLen length of the data in the sendBuffer [in Byte]
     *  @param  recBuf the buffer, where the recieved data will be stored
     *  @param  rxLen length of the data to be read [in Byte]
     *  @param  trigger the Byte received on the MISO line, that signals 
     *          the start of a message / when to start sending
     *  @param  timeout when to stop waiting for the sign-Byte
     *  @param  notTrig if set to true, start recording/sending
     *          after the first byte that doesn't match sign
     *          if set to false, start sending/recording with the first
     *          byte that matches sign
     *  @param  syncd whether to use the synchronize feature of this
     *          function (default: true)
     */ 
    bool writeReadTrig( const void* sendBuf, size_t txLen, void* recBuf, size_t rxLen, 
                        const uint8_t trigger, const int64_t timeout, bool notTrig, bool syncd=true);

};
}
