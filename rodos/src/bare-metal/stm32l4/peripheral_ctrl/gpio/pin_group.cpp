#include "pin_group.h"
#include "peripheral_ctrl/peripheral_rcc/hw_rcc_periph.h"

namespace RODOS {

namespace GPIO {


    PinGroup::PinGroup(GPIO_PIN pinIdx, uint8_t numPins)
        : port(pinIdx)
    {
        if(pinIdx != GPIO_INVALID){
            calcPinMask(static_cast<uint8_t>(pinIdx % 16), numPins);
        }
    }

    void PinGroup::reset()
    {
        setMode(MODE::ANALOG);
        setSpeed(SPEED::LOW);
        setPull(PULL::NONE);
        setOutputType(OUTPUTTYPE::PUSHPULL);
        setAlternateFunction(ALTERNATEFUNCTION::AF0);
        port.gpio->BRR.setMasked(pinMask, 1);
    }

    void PinGroup::setOutput(uint16_t val)
    {
        uint16_t pinMaskSet = static_cast<uint16_t>((val << (trailingZeros(pinMask))) & pinMask);
        uint16_t pinMaskReset = static_cast<uint16_t>((((uint16_t) ~val) << (trailingZeros(pinMask))) & pinMask);

        //direct register write to both set and reset the pins at once
        uint32_t registerValue = ((uint32_t)pinMaskSet) | (((uint32_t)pinMaskReset) << 16);
        port.gpio->BSRR.write(registerValue);
    }

    uint16_t PinGroup::read()
    {
        uint16_t portData;
        if (getMode() == MODE::OUTPUT) {
            portData = (uint16_t) port.gpio->ODR.read();
        } else {
            portData = (uint16_t) port.gpio->IDR.read();
        }
        return static_cast<uint16_t>((portData & this->pinMask) >> trailingZeros(pinMask));
    }

    void PinGroup::setAlternateFunction(ALTERNATEFUNCTION af)
    {
        uint32_t pinMaskLower = pinMask & 0x00FF;
        uint32_t pinMaskHigher = (pinMask & 0xFF00) >> 8;
        port.gpio->AFRL.setMasked(pinMaskLower, (uint32_t)af);
        port.gpio->AFRH.setMasked(pinMaskHigher, (uint32_t)af);
    }

    uint16_t PinGroup::calcPinMask(uint8_t firstPin, uint32_t numOfPins)
    {
        if (numOfPins + firstPin > 16) {
            // pin-group exceeds port boundary ! only the pins up to most significant pin of port will be set
            pinMask = static_cast<uint16_t>(0xFFFF << firstPin);
            return false;
        } else {
            pinMask = static_cast<uint16_t>(0xFFFFu >> (16 - numOfPins));
            pinMask = static_cast<uint16_t>(pinMask << firstPin);
            return true;
        }
    }
}

}
