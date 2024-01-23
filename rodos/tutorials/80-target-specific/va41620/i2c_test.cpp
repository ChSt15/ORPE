#include "thread.h"
#include "hal/hal_i2c.h"
#include "rodos-debug.h"

/**
 * @brief reads the WHO_AM_I register of the LIS2DE12TR on the VA41620 GPIO board
 */
class I2CTestThread : RODOS::StaticThread<> {
    RODOS::HAL_I2C m_i2c { RODOS::I2C_IDX0 };

    void init() override {
        constexpr uint32_t I2C_SPEED { 400'000 };
        m_i2c.init(I2C_SPEED);
    }

    void run() override {
        constexpr uint32_t LIS2DE12_I2C_ADDRESS { 0b0011001 };
        constexpr uint8_t WHO_AM_I_REGISTER_ADDRESS { 0x0F };
        constexpr uint8_t WHO_AM_I_EXPECTED_VALUE { 0b00110011 };

        uint8_t rxBuffer{};
        int32_t i2cReturnValue { m_i2c.writeRead(LIS2DE12_I2C_ADDRESS, &WHO_AM_I_REGISTER_ADDRESS, sizeof(WHO_AM_I_REGISTER_ADDRESS), &rxBuffer, sizeof(rxBuffer)) };

        RODOS::PRINTF("I2C return value on writeRead: %ld\n", static_cast<long int>(i2cReturnValue));
        RODOS::PRINTF("Expected ID: %02x\n", WHO_AM_I_EXPECTED_VALUE);
        RODOS::PRINTF("Got ID:      %02x\n", rxBuffer);
    }
};

I2CTestThread testThread;
