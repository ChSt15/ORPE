#pragma once

#include "rodos.h"

#include "port.h"

namespace RODOS {

namespace GPIO {
    class PinGroup {
        Port port;
        uint16_t pinMask;
        MODE currentMode = MODE::INPUT;

    public:
        PinGroup(GPIO_PIN pinIdx, uint8_t numPins = 1);
        uint16_t calcPinMask(uint8_t firstPin, uint32_t numOfPins);

        void reset();
        void setOutput(uint16_t val);
        uint16_t read();

        void setAlternateFunction(ALTERNATEFUNCTION af);

        inline void enableClocks()
        {
            port.enableClock();
        }

        inline void setMode(MODE mode)
        {
            currentMode = mode;
            port.gpio->MODER.setMasked(pinMask, (uint32_t)mode);
        }

        inline void setOutputType(OUTPUTTYPE outputType)
        {
            port.gpio->OTYPER.setMasked(pinMask, (uint32_t)outputType);
        }

        inline void setSpeed(SPEED speed)
        {
            port.gpio->OSPEEDR.setMasked(pinMask, (uint32_t)speed);
        }

        inline void setPull(PULL pull)
        {
            port.gpio->PUPDR.setMasked(pinMask, (uint32_t)pull);
        }

        inline MODE getMode()
        {
            return currentMode;
        }

        inline bool isSinglePin()
        {
            return (pinMask>>trailingZeros(pinMask))==1;;
        }

        inline uint8_t getPinPos()
        {
            return trailingZeros(pinMask);
        }

        inline bool isValid()
        {
            return port.isValid() && pinMask != 0;
        }

        inline bool setNumPins(uint32_t numPins)
        {
            return calcPinMask(trailingZeros(pinMask), numPins);
        }

        inline GPIO_PIN asRODOSPin()
        {
            return (GPIO_PIN)(getPinPos() + port.getPortNum() * 16);
        }

        inline Port& getPort()
        {
            return port;
        }
    };
}

}

