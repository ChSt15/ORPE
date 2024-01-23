#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "hal/hal_i2c.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "clkgen_config.h"
#include "tests/peripheral_test_fixture.h"

#include <cstring>

namespace {

class I2CTest : public RODOS::testing::PeripheralTestFixture {};

using ::testing::Expectation;
using ::testing::ExpectationSet;
using ::testing::_;
using ::testing::Return;

TEST_F(I2CTest, TestInit){
    using namespace RODOS::I2C_CLKSCALE;
    using namespace RODOS::I2C_CTRL;
    RODOS::I2C_IDX idx { RODOS::I2C_IDX0 };
    uint32_t usedSpeed { 400'000 };
    RODOS::HAL_I2C i2c { idx };

    Expectation clockInitExpectation { EXPECT_CALL(RODOS::sysconfigPeripheralBitband->I2C_ENABLE[idx], write(1)) };
    uint32_t FAST_MODE_MULTIPLIER { 25 };
    uint32_t expectedClkScaleValue { (RODOS::globalClockSetup.getAPB1Frequency()/FAST_MODE_MULTIPLIER/usedSpeed) - 1 };
    EXPECT_CALL(RODOS::i2cs[idx].CLKSCALE, write(VALUE(expectedClkScaleValue) | FASTMODE(1)))
        .After(clockInitExpectation);
    EXPECT_CALL(RODOS::i2cs[idx].CLKTOLIMIT, write(_))
        .After(clockInitExpectation)
        .WillOnce([&](int clktolimit){
            int64_t timeoutApb1Cycles { clktolimit*expectedClkScaleValue };
            double timeoutSeconds { static_cast<double>(timeoutApb1Cycles)/static_cast<double>(RODOS::globalClockSetup.getAPB1Frequency()) };
            double expectedTimeoutSeconds { 1e-3 };
            double allowedTimeoutErrorSeconds { expectedTimeoutSeconds/2 };
            ASSERT_NEAR(timeoutSeconds, expectedTimeoutSeconds, allowedTimeoutErrorSeconds);
        });
    EXPECT_CALL(RODOS::i2cs[idx].CTRL, write(ENABLE(1) | ALGFILTER(1)))
        .After(clockInitExpectation);
    i2c.init(usedSpeed);
}

TEST_F(I2CTest, TestWrite){
    using namespace RODOS::I2C_CMD;
    using namespace RODOS::I2C_STATUS;
    using namespace RODOS::I2C_FIFO_CLR;
    using namespace RODOS::I2C_IRQ_ENB;
    using namespace RODOS::I2C_ADDRESS;

    RODOS::I2C_IDX idx { RODOS::I2C_IDX2 };
    RODOS::HAL_I2C i2c { idx };

    uint8_t testedData[] { 1, 2, 3, 4, 3, 2, 9, 4, 3, 2, 3, 1, 5, 8, 3, 5, 6, 2, 1, 6, 7, 3, 66, 23, 53, 63, 121, 32, 24, 244, 42, 53, 123, 213, 123  };
    uint8_t testedAddress { 0b101010 };

    ExpectationSet setupExpectations;
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].FIFO_CLR, write(static_cast<uint32_t>(TXFIFO(1))));
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].WORDS, write(sizeof(testedData)));
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].ADDRESS, write(ADDRESS(testedAddress) | DIRECTION(I2C_DIRECTION_TX)));

    Expectation cmdExpectation  { 
        EXPECT_CALL(RODOS::i2cs[idx].CMD, write(START(1) | STOP(1)))
        .After(setupExpectations)
    };

    uint8_t currentWriteIndex { 0 };
    constexpr uint8_t TOTAL_FIFO_CAPACITY { 16 };
    uint8_t freeSpaceInFifo { TOTAL_FIFO_CAPACITY };
    constexpr uint8_t STATUS_READS_PER_SENT_BYTE { 3 };
    uint8_t statusReadsUntilNextByte { STATUS_READS_PER_SENT_BYTE };

    EXPECT_CALL(RODOS::i2cs[idx].DATA, write(_))
        .Times(sizeof(testedData))
        .After(cmdExpectation)
        .WillRepeatedly([&](uint32_t data){
            ASSERT_GT(freeSpaceInFifo, 0);
            freeSpaceInFifo--;
            ASSERT_EQ(data, testedData[currentWriteIndex++]);
        });
    EXPECT_CALL(RODOS::i2cs[idx].STATUS, read())
        .After(cmdExpectation)
        .WillRepeatedly([&]{
            if(statusReadsUntilNextByte==0){
                statusReadsUntilNextByte = STATUS_READS_PER_SENT_BYTE;
                if(freeSpaceInFifo<TOTAL_FIFO_CAPACITY){
                    freeSpaceInFifo++;
                }
            }
            statusReadsUntilNextByte--;
            return IDLE { (freeSpaceInFifo==TOTAL_FIFO_CAPACITY && currentWriteIndex==sizeof(testedData)) ? 1u : 0u } | TXNFULL { freeSpaceInFifo>0 ? 1u : 0u };
        });
    EXPECT_CALL(RODOS::i2cs[idx].IRQ_RAW, read())
        .WillRepeatedly(Return(0));
    EXPECT_CALL(RODOS::i2cs[idx].TXCOUNT, read())
        .WillOnce([&]{ return currentWriteIndex; });

    ASSERT_EQ(i2c.write(testedAddress, testedData, sizeof(testedData)), sizeof(testedData));
}


TEST_F(I2CTest, TestRead){
    using namespace RODOS::I2C_CMD;
    using namespace RODOS::I2C_STATUS;
    using namespace RODOS::I2C_FIFO_CLR;
    using namespace RODOS::I2C_IRQ_ENB;
    using namespace RODOS::I2C_ADDRESS;

    RODOS::I2C_IDX idx { RODOS::I2C_IDX1 };
    RODOS::HAL_I2C i2c { idx };

    uint8_t testedData[] { 1, 2, 7, 4, 3, 2, 9, 4, 3, 2, 3, 1, 5, 8, 3, 5, 6, 2, 1, 6, 7, 3, 66, 23, 53, 63, 121, 1, 24, 244, 42, 53, 123, 213, 123  };
    uint8_t testedAddress { 0b11010 };

    ExpectationSet setupExpectations;
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].FIFO_CLR, write(static_cast<uint32_t>(RXFIFO(1))));
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].WORDS, write(sizeof(testedData)));
    setupExpectations += EXPECT_CALL(RODOS::i2cs[idx].ADDRESS, write(ADDRESS(testedAddress) | DIRECTION(I2C_DIRECTION_RX)));

    Expectation cmdExpectation  { 
        EXPECT_CALL(RODOS::i2cs[idx].CMD, write(START(1) | STOP(1)))
        .After(setupExpectations)
    };

    uint8_t currentReadIndex { 0 };
    constexpr uint8_t TOTAL_FIFO_CAPACITY { 16 };
    uint8_t filledSpaceInFifo { TOTAL_FIFO_CAPACITY };
    constexpr uint8_t STATUS_READS_PER_RECEIVED_BYTE { 3 };
    uint8_t statusReadsUntilNextByte { STATUS_READS_PER_RECEIVED_BYTE };

    EXPECT_CALL(RODOS::i2cs[idx].DATA, read())
        .Times(sizeof(testedData))
        .After(cmdExpectation)
        .WillRepeatedly([&]()->uint32_t {
            EXPECT_GT(filledSpaceInFifo, 0);
            filledSpaceInFifo--;
            return testedData[currentReadIndex++];
        });
    EXPECT_CALL(RODOS::i2cs[idx].STATUS, read())
        .After(cmdExpectation)
        .WillRepeatedly([&]{
            if(statusReadsUntilNextByte==0){
                statusReadsUntilNextByte = STATUS_READS_PER_RECEIVED_BYTE;
                if(filledSpaceInFifo<TOTAL_FIFO_CAPACITY){
                    filledSpaceInFifo++;
                }
            }
            statusReadsUntilNextByte--;
            return IDLE { (filledSpaceInFifo==0 && currentReadIndex==sizeof(testedData)) ? 1u : 0u } | RXNEMPTY { filledSpaceInFifo>0 ? 1u : 0u };
        });
    EXPECT_CALL(RODOS::i2cs[idx].IRQ_RAW, read())
        .WillRepeatedly(Return(0));
    EXPECT_CALL(RODOS::i2cs[idx].RXCOUNT, read())
        .WillOnce([&]{ return currentReadIndex; });

    uint8_t readBuffer[sizeof(testedData)] {};
    ASSERT_EQ(i2c.read(testedAddress, readBuffer, sizeof(testedData)), sizeof(testedData));
    ASSERT_EQ(std::memcmp(readBuffer, testedData, sizeof(testedData)), 0);
}


TEST_F(I2CTest, TestWriteRead){
    using namespace RODOS::I2C_CMD;
    using namespace RODOS::I2C_STATUS;
    using namespace RODOS::I2C_FIFO_CLR;
    using namespace RODOS::I2C_IRQ_ENB;
    using namespace RODOS::I2C_ADDRESS;

    RODOS::I2C_IDX idx { RODOS::I2C_IDX2 };
    RODOS::HAL_I2C i2c { idx };

    uint8_t testedTxData[] { 1, 2, 3, 4, 3, 2, 9, 4, 3, 2, 3, 1, 5, 8, 3, 5, 6, 2, 1, 6, 7, 3, 66, 23, 53, 63, 121, 32, 24, 244, 42, 53, 123, 213, 123  };
    uint8_t testedRxData[] { 1, 2, 43, 13, 7, 4, 3, 2, 9, 4, 3, 2, 3, 1, 5, 8, 3, 5, 6, 2, 1, 6, 7, 3, 12, 23, 53, 63, 121, 1, 24, 244, 42, 53, 123, 213, 123  };
    uint8_t testedAddress { 0b1010 };

    ExpectationSet txSetupExpectations;
    txSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].FIFO_CLR, write(static_cast<uint32_t>(TXFIFO(1))));
    txSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].WORDS, write(sizeof(testedTxData)));
    txSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].ADDRESS, write(ADDRESS(testedAddress) | DIRECTION(I2C_DIRECTION_TX)));

    Expectation txCmdExpectation  { 
        EXPECT_CALL(RODOS::i2cs[idx].CMD, write(static_cast<uint32_t>(START(1))))
        .After(txSetupExpectations)
    };

    uint8_t currentWriteIndex { 0 };
    constexpr uint8_t TOTAL_FIFO_CAPACITY { 16 };
    uint8_t freeSpaceInTxFifo { TOTAL_FIFO_CAPACITY };
    constexpr uint8_t STATUS_READS_PER_SENT_BYTE { 3 };
    uint8_t statusReadsUntilNextTransmittedByte { STATUS_READS_PER_SENT_BYTE };

    ExpectationSet txExpectations;
    txExpectations += EXPECT_CALL(RODOS::i2cs[idx].DATA, write(_))
        .Times(sizeof(testedTxData))
        .After(txCmdExpectation)
        .WillRepeatedly([&](uint32_t data){
            ASSERT_GT(freeSpaceInTxFifo, 0);
            freeSpaceInTxFifo--;
            ASSERT_EQ(data, testedTxData[currentWriteIndex++]);
        });
    txExpectations += EXPECT_CALL(RODOS::i2cs[idx].STATUS, read())
        .After(txCmdExpectation)
        .WillRepeatedly([&]{
            if(statusReadsUntilNextTransmittedByte==0){
                statusReadsUntilNextTransmittedByte = STATUS_READS_PER_SENT_BYTE;
                if(freeSpaceInTxFifo<TOTAL_FIFO_CAPACITY){
                    freeSpaceInTxFifo++;
                }
            }
            statusReadsUntilNextTransmittedByte--;
            return WAITING { (freeSpaceInTxFifo==TOTAL_FIFO_CAPACITY && currentWriteIndex==sizeof(testedTxData)) ? 1u : 0u } | TXNFULL { freeSpaceInTxFifo>0 ? 1u : 0u };
        });
    EXPECT_CALL(RODOS::i2cs[idx].IRQ_RAW, read())
        .WillRepeatedly(Return(0));
    txExpectations += EXPECT_CALL(RODOS::i2cs[idx].TXCOUNT, read())
        .WillOnce([&]{ return currentWriteIndex; });


    ExpectationSet rxSetupExpectations;
    rxSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].FIFO_CLR, write(static_cast<uint32_t>(RXFIFO(1))))
        .After(txExpectations);
    rxSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].WORDS, write(sizeof(testedRxData)))
        .After(txExpectations);
    rxSetupExpectations += EXPECT_CALL(RODOS::i2cs[idx].ADDRESS, write(ADDRESS(testedAddress) | DIRECTION(I2C_DIRECTION_RX)))
        .After(txExpectations);

    Expectation rxCmdExpectation  { 
        EXPECT_CALL(RODOS::i2cs[idx].CMD, write(START(1) | STOP(1)))
            .After(rxSetupExpectations)
    };

    uint8_t currentReadIndex { 0 };
    uint8_t filledSpaceInRxFifo { TOTAL_FIFO_CAPACITY };
    constexpr uint8_t STATUS_READS_PER_RECEIVED_BYTE { 3 };
    uint8_t statusReadsUntilNextReceivedByte { STATUS_READS_PER_RECEIVED_BYTE };

    EXPECT_CALL(RODOS::i2cs[idx].DATA, read())
        .Times(sizeof(testedRxData))
        .After(rxCmdExpectation)
        .WillRepeatedly([&]()->uint32_t {
            EXPECT_GT(filledSpaceInRxFifo, 0);
            filledSpaceInRxFifo--;
            return testedRxData[currentReadIndex++];
        });
    EXPECT_CALL(RODOS::i2cs[idx].STATUS, read())
        .After(rxCmdExpectation)
        .WillRepeatedly([&]{
            if(statusReadsUntilNextReceivedByte==0){
                statusReadsUntilNextReceivedByte = STATUS_READS_PER_RECEIVED_BYTE;
                if(filledSpaceInRxFifo<TOTAL_FIFO_CAPACITY){
                    filledSpaceInRxFifo++;
                }
            }
            statusReadsUntilNextReceivedByte--;
            return IDLE { (filledSpaceInRxFifo==0 && currentReadIndex==sizeof(testedRxData)) ? 1u : 0u } | RXNEMPTY { filledSpaceInRxFifo>0 ? 1u : 0u };
        });
    EXPECT_CALL(RODOS::i2cs[idx].RXCOUNT, read())
        .WillOnce([&]{ return currentReadIndex; });

    uint8_t readBuffer[sizeof(testedRxData)] {};
    ASSERT_EQ(i2c.writeRead(testedAddress, testedTxData, sizeof(testedTxData), readBuffer, sizeof(testedRxData)), sizeof(testedRxData));
    ASSERT_EQ(std::memcmp(readBuffer, testedRxData, sizeof(testedRxData)), 0);
}


}
