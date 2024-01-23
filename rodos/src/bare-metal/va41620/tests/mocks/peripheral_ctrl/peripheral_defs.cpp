#include "peripheral_defs.h"

namespace RODOS {

ADCStruct* adc {nullptr};
I2CStruct* i2cs {nullptr};
CANStruct* cans {nullptr};
SPIStruct* spis {nullptr};
UARTStruct* uart0 {nullptr};
UARTStruct* uart1 {nullptr};
UARTStruct* uart2 {nullptr};
IOCONFIGStruct* ioconfig {nullptr};
TIMStruct* tim0to15 {nullptr};
TIMStruct* tim16to23 {nullptr};
CLKGENStruct* clkgen {nullptr};
SYSCONFIGStruct* sysconfig {nullptr};
GPIOStruct* gpioStructs {nullptr};
SYSCONFIGPeripheralBitbandStruct* sysconfigPeripheralBitband {nullptr};

}
