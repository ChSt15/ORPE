#pragma once

#include <cstdint>
#include "hal/hal_i2c.h"
#include "peripheral_ctrl/register_maps/i2c_registers.h"
#include "clkgen_config.h"

namespace RODOS {

class HW_HAL_I2C {
public:
    static constexpr auto NUM_I2CS { 3 };

    HW_HAL_I2C(I2C_IDX idx);

    void init(uint32_t speed) const;
    int32_t write(const uint8_t addr, const uint8_t *txBuf, uint32_t txSize, bool stop) const;
    int32_t read(const uint8_t addr, uint8_t *rxBuf, uint32_t rxSize) const;
private:
    I2CStruct& m_i2c;
    const I2C_IDX m_idx;

    static constexpr auto MAX_TRANSFER_SIZE { 0x7FE };

    //For calculation details see Programmer's manual section 12.3.2
    static constexpr int8_t FAST_MODE_CLK_MULTIPLIER { 25 };
    static constexpr int32_t BUS_FREQUENCY { globalClockSetup.getAPB1Frequency() };
    static constexpr int32_t FREQUENCY_BEFORE_CLKSCALE { BUS_FREQUENCY/FAST_MODE_CLK_MULTIPLIER };
    static constexpr int32_t MAX_I2C_FAST_MODE_SPEED { 400'000 };
    static constexpr int32_t MAX_SPEED { FREQUENCY_BEFORE_CLKSCALE>MAX_I2C_FAST_MODE_SPEED ? MAX_I2C_FAST_MODE_SPEED : FREQUENCY_BEFORE_CLKSCALE };

    void setClockLowTimeout(uint32_t speed) const;
    uint8_t calculateClkscaleValue(uint32_t speed) const;
    bool hasClockLowTimeoutOccured() const;
    void resetClockLowTimeoutState() const;
};

}
