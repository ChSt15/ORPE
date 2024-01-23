#include "hal/hal_spi.h"
#include "rodos-debug.h"
#include "thread.h"

class SpiTestThread : RODOS::StaticThread<> {
    RODOS::HAL_SPI m_spi { RODOS::SPI_IDX1, RODOS::GPIO_083, RODOS::GPIO_084, RODOS::GPIO_085, RODOS::GPIO_100 };

    void init() override {
        m_spi.init(12'500'000, false, false);
    }

    void run() override {
        constexpr uint8_t RDID_OPCODE = 0b1001'1111;
        constexpr uint8_t EXPECTED_ID[9] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x25, 0x08};
        uint8_t rxBuffer[10]{};
        m_spi.writeRead(&RDID_OPCODE, sizeof(RDID_OPCODE), rxBuffer, sizeof(rxBuffer));
        RODOS::printCharsHex("Expected ID: ", EXPECTED_ID, sizeof(EXPECTED_ID));
        RODOS::printCharsHex("Read ID:     ", rxBuffer+1, sizeof(EXPECTED_ID));
    }
};

SpiTestThread testThread;
