#include "peripheral_test_fixture.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "peripheral_ctrl/pin_config/pin_config_mock.h"
#include "cortex_m/nvic_mock.h"

namespace RODOS::testing {

using ::testing::StrictMock;

void PeripheralTestFixture::SetUp(){
    globalPinConfigMock = new StrictMock<PinConfigMock>;
    globalNVICMock = new StrictMock<NVICMock>;

    RODOS::adc = new RODOS::ADCStruct;
    RODOS::i2cs = new RODOS::I2CStruct[3];
    RODOS::cans = new RODOS::CANStruct[2];
    RODOS::uart0 = new RODOS::UARTStruct;
    RODOS::uart1 = new RODOS::UARTStruct;
    RODOS::uart2 = new RODOS::UARTStruct;
    RODOS::spis = new RODOS::SPIStruct[4];
    RODOS::ioconfig = new RODOS::IOCONFIGStruct;
    RODOS::tim0to15 = new RODOS::TIMStruct[16];
    RODOS::tim16to23 = new RODOS::TIMStruct[8];
    RODOS::clkgen = new RODOS::CLKGENStruct;
    RODOS::sysconfig = new RODOS::SYSCONFIGStruct;
    RODOS::sysconfigPeripheralBitband = new RODOS::SYSCONFIGPeripheralBitbandStruct;
    RODOS::gpioStructs = new RODOS::GPIOStruct[RODOS::numGpioStructs];
}

void PeripheralTestFixture::TearDown(){
    delete globalPinConfigMock;
    delete globalNVICMock;

    delete RODOS::adc;
    delete [] RODOS::i2cs;
    delete [] RODOS::cans;
    delete RODOS::uart0;
    delete RODOS::uart1;
    delete RODOS::uart2;
    delete RODOS::ioconfig;
    delete RODOS::clkgen;
    delete RODOS::sysconfig;
    delete RODOS::sysconfigPeripheralBitband;
    delete [] RODOS::gpioStructs;
    delete [] RODOS::spis;
    delete [] RODOS::tim0to15;
    delete [] RODOS::tim16to23;
}

}
