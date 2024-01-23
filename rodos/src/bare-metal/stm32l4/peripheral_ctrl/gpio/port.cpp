#include "port.h"
#include "peripheral_ctrl/register_maps/pwr_registers.h"
#include "peripherals.h"

namespace RODOS {

namespace GPIO {
constexpr uint32_t GPIO_PERIPH_BASE_OFFSET = 4;

uint8_t Port::getPortNum()
{
    return static_cast<uint8_t>(gpio-gpios);
}

HWRCCPeriph& Port::getRCCPeriph()
{
    uint32_t hwRccPeriphRstAddr = GPIOADef::RCC_PERIPH_RST_ADDR + (getPortNum() * GPIO_PERIPH_BASE_OFFSET);
    return *reinterpret_cast<HWRCCPeriph*>(hwRccPeriphRstAddr);
}

bool Port::needsVddIO2()
{
    return gpio==&gpios[6];
}

void Port::enableClock()
{
    if (needsVddIO2()) {
        PWRDef::rccPeriph->enableClock();
        pwr.CR2.set<PWR_CR2::IOSV>(1);
    }
    HWRCCPeriph& periph = getRCCPeriph();
    periph.enableClock();
    periph.releaseReset();
}

}


}
