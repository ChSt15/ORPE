#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "hal/hal_spi.h"
#include "clkgen_config.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "peripheral_ctrl/pin_config/pin_config_mock.h"
#include "peripheral_ctrl/pin_config/enums.h"
#include "tests/peripheral_test_fixture.h"

#include <cstring>
#include <array>
#include <algorithm>
#include <random>

namespace {
using ::testing::ExpectationSet;
using ::testing::Expectation;
using ::testing::_;
using ::testing::Mock;

using namespace RODOS::testing;

class SPITest : public PeripheralTestFixture {};


TEST_F(SPITest, VerifyInit){
    using namespace RODOS::SPI_CTRL0;
    using namespace RODOS::SPI_CLKPRESCALE;
    using namespace RODOS::SPI_CTRL1;
    constexpr RODOS::SPI_IDX idx { RODOS::SPI_IDX0 };
    constexpr RODOS::GPIO_PIN sckPin { RODOS::GPIO_031 };
    constexpr RODOS::GPIO_PIN misoPin { RODOS::GPIO_032 };
    constexpr RODOS::GPIO_PIN mosiPin { RODOS::GPIO_033 };
    constexpr RODOS::GPIO_PIN nssPin { RODOS::GPIO_027 };
    constexpr uint32_t attemptedFrequency { 1'000'000 };
    RODOS::HAL_SPI spi { idx, sckPin, misoPin, mosiPin, nssPin };

    Expectation clockSetupExpectation { EXPECT_CALL(RODOS::sysconfigPeripheralBitband->SPI_ENABLE[idx], write(1)) };

    RODOS::UInt32BitField fakeCTRL0Register;
    RODOS::UInt32BitField fakeCLKPRESCALERegister;

    EXPECT_CALL(RODOS::spis[idx].CTRL0, read())
        .After(clockSetupExpectation)
        .WillRepeatedly([&]{return fakeCTRL0Register.read();});
    EXPECT_CALL(RODOS::spis[idx].CTRL0, write(_))
        .After(clockSetupExpectation)
        .WillRepeatedly([&](uint32_t x){fakeCTRL0Register.write(x);});
    EXPECT_CALL(RODOS::spis[idx].CLKPRESCALE, write(_))
        .After(clockSetupExpectation)
        .WillRepeatedly([&](uint32_t x){fakeCLKPRESCALERegister.write(x);});

    constexpr uint8_t EXPECTED_NSS_INDEX { 3 };
    EXPECT_CALL(RODOS::spis[idx].CTRL1, write(SS(EXPECTED_NSS_INDEX) | BLOCKMODE(1) | BMSTALL(1) | ENABLE(1)))
        .After(clockSetupExpectation);

    constexpr auto expectedFunction { RODOS::pin_config::PIN_FUNCTION::FUN1 };
    constexpr auto expectedPull { RODOS::pin_config::PULL::OFF };
    constexpr auto expectedOutputType { RODOS::pin_config::OUTPUT_TYPE::PUSHPULL };
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(sckPin), expectedFunction, expectedOutputType, expectedPull));
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(misoPin), expectedFunction, expectedOutputType, expectedPull));
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(mosiPin), expectedFunction, expectedOutputType, expectedPull));
    EXPECT_CALL(*globalPinConfigMock, configurePin(RODOS::pin_config::VA41620_PIN(nssPin), expectedFunction, expectedOutputType, expectedPull));

    spi.init(attemptedFrequency);

    uint32_t clkprescaleValue { fakeCLKPRESCALERegister.read<VALUE>() };
    uint32_t scrdvValue { fakeCTRL0Register.read<SCRDV>() };

    double achievedFrequency { static_cast<double>(RODOS::globalClockSetup.getSysclkFrequency()/4)/(clkprescaleValue*(scrdvValue+1)) };
    double frequencyTolerance { attemptedFrequency/10.0 };
    ASSERT_NEAR(achievedFrequency, attemptedFrequency, frequencyTolerance);

    uint32_t wordSize { fakeCTRL0Register.read<SIZE>() };
    ASSERT_EQ(wordSize+1, 8);
}

enum class SPI_TRANSFER_FUNCTION {
    READ,
    WRITE,
    WRITEREAD
};

template<SPI_TRANSFER_FUNCTION function, size_t txSize, size_t rxSize>
void testSpiWriteRead(RODOS::SPI_IDX idx, const std::array<std::byte, txSize>& txBuffer, const std::array<std::byte, rxSize>& rxData){
    using namespace RODOS::SPI_STATUS;
    using namespace RODOS::SPI_DATA;
    using namespace RODOS::SPI_FIFO_CLR;
    RODOS::HAL_SPI spi { idx };

    constexpr auto STATUS_READS_PER_BYTE_TRANSMITTED { 4 };
    constexpr auto STATUS_READS_PER_BYTE_RECEIVED { 2 };
    int8_t statusReadsUntilNextByteTransmitted { STATUS_READS_PER_BYTE_TRANSMITTED };
    int8_t statusReadsUntilNextByteReceived { STATUS_READS_PER_BYTE_RECEIVED };

    size_t txIndex { 0 };
    [[maybe_unused]] size_t rxIndex { 0 };

    constexpr size_t FIFO_SIZE { 16 };
    size_t currentBytesInRxFifo { 0 };
    size_t currentBytesInTxFifo { 0 };
    size_t totalBytesReceived { 0 };
    size_t totalBytesTransmitted { 0 };

    constexpr std::size_t TOTAL_TRANSMISSION_SIZE { std::max(rxSize, txSize) };
    std::array<std::byte, TOTAL_TRANSMISSION_SIZE> txData;
    std::byte expectedDummyByte { 0xFF };
    for(size_t i=0; i<TOTAL_TRANSMISSION_SIZE; i++){
        if constexpr (txSize>0){
            if(i<txSize){
                txData[i] = txBuffer[i];
            }else{
                txData[i] = expectedDummyByte;
            }
        }else{
            txData[i] = expectedDummyByte;
        }
    }

    Expectation fifoClearExpectation { EXPECT_CALL(RODOS::spis[idx].FIFO_CLR, write(TXFIFO(1) | RXFIFO(1))) };

    EXPECT_CALL(RODOS::spis[idx].STATUS, read())
        .After(fifoClearExpectation)
        .WillRepeatedly([&]{
            const bool isStillTransmitting { totalBytesTransmitted!=TOTAL_TRANSMISSION_SIZE };
            const bool isStillReceiving { [&]{
                if constexpr(rxSize==0){
                    return false;
                }else{
                    return totalBytesReceived<rxSize;
                }
            }()};
            bool busy { isStillTransmitting || isStillReceiving };

            if(currentBytesInTxFifo>0){
                statusReadsUntilNextByteTransmitted--;
                if(statusReadsUntilNextByteTransmitted<=0){
                    totalBytesTransmitted++;
                    currentBytesInTxFifo--;
                    statusReadsUntilNextByteTransmitted = STATUS_READS_PER_BYTE_TRANSMITTED;
                }
            }
            if(currentBytesInRxFifo<FIFO_SIZE && totalBytesTransmitted>totalBytesReceived){
                statusReadsUntilNextByteReceived--;
                if(statusReadsUntilNextByteReceived<=0){
                    totalBytesReceived++;
                    currentBytesInRxFifo++;
                    statusReadsUntilNextByteReceived = STATUS_READS_PER_BYTE_RECEIVED;
                }
            }
            return RNE(currentBytesInRxFifo>0) | TNF(currentBytesInTxFifo<FIFO_SIZE) | BUSY(busy);
        });

    if constexpr (rxSize>0){
        EXPECT_CALL(RODOS::spis[idx].DATA, read())
            .Times(rxSize)
            .After(fifoClearExpectation)
            .WillRepeatedly([&]()->uint32_t{
                EXPECT_GT(currentBytesInRxFifo, 0);
                currentBytesInRxFifo--;
                return std::to_integer<uint32_t>(rxData[rxIndex++]);
            });
    }
    Expectation allButLastByteWriteExpectation {
        EXPECT_CALL(RODOS::spis[idx].DATA, write(_))
            .Times(TOTAL_TRANSMISSION_SIZE-1)
            .After(fifoClearExpectation)
            .WillRepeatedly([&](uint32_t x){
                constexpr uint32_t DATA_BITMASK { 0xFF };
                EXPECT_EQ(x&~DATA_BITMASK, 0);
                EXPECT_GT(FIFO_SIZE, currentBytesInTxFifo);
                EXPECT_EQ(std::to_integer<uint32_t>(txData[txIndex]), x);
                currentBytesInTxFifo++;
                txIndex++;
            })
    };
    EXPECT_CALL(RODOS::spis[idx].DATA, write(_))
        .After(allButLastByteWriteExpectation)
        .WillOnce([&](uint32_t x){
            RODOS::UInt32BitField data { x };
            EXPECT_EQ(data.read<BMSTOP>(), 1);
            EXPECT_GT(FIFO_SIZE, currentBytesInTxFifo);
            EXPECT_EQ(std::to_integer<uint32_t>(txData[txIndex]), data.read<VALUE>());
            currentBytesInTxFifo++;
        });

    std::array<std::byte, rxSize> rxBuffer{};
    switch(function){
    case SPI_TRANSFER_FUNCTION::WRITEREAD:
        spi.writeRead(txBuffer.data(), txSize, rxBuffer.data(), rxSize);
        break;
    case SPI_TRANSFER_FUNCTION::READ:
        spi.read(rxBuffer.data(), rxSize);
        break;
    case SPI_TRANSFER_FUNCTION::WRITE:
        spi.write(txBuffer.data(), txSize);
        break;
    }

    ASSERT_EQ(rxBuffer, rxData);

    Mock::VerifyAndClear(&RODOS::spis[idx].DATA);
    Mock::VerifyAndClear(&RODOS::spis[idx].STATUS);
    Mock::VerifyAndClear(&RODOS::spis[idx].FIFO_CLR);
}

template<size_t size>
void fillByteArrayWithRandomBytes(std::array<std::byte, size>& array){
    static std::random_device rd;
    static std::mt19937 mt { rd() };
    std::uniform_int_distribution<uint8_t> byteDistribution;
    std::generate(array.begin(), array.end(), [&](){return std::byte { byteDistribution(mt) };});
}


TEST_F(SPITest, testWriteRead){
    RODOS::SPI_IDX idx { RODOS::SPI_IDX2 };

    std::array<std::byte, 100> testTxData1;
    fillByteArrayWithRandomBytes(testTxData1);
    std::array<std::byte, 50> testRxData1;
    fillByteArrayWithRandomBytes(testRxData1);
    testSpiWriteRead<SPI_TRANSFER_FUNCTION::WRITEREAD>(idx, testTxData1, testRxData1);

    std::array<std::byte, 50> testTxData2;
    fillByteArrayWithRandomBytes(testTxData2);
    std::array<std::byte, 100> testRxData2;
    fillByteArrayWithRandomBytes(testRxData2);
    testSpiWriteRead<SPI_TRANSFER_FUNCTION::WRITEREAD>(idx, testTxData2, testRxData2);

    std::array<std::byte, 100> testTxData3;
    fillByteArrayWithRandomBytes(testTxData3);
    std::array<std::byte, 100> testRxData3;
    fillByteArrayWithRandomBytes(testRxData3);
    testSpiWriteRead<SPI_TRANSFER_FUNCTION::WRITEREAD>(idx, testTxData3, testRxData3);
}

TEST_F(SPITest, testRead){
    RODOS::SPI_IDX idx { RODOS::SPI_IDX0 };

    std::array<std::byte, 0> testTxData1;
    std::array<std::byte, 100> testRxData1;
    fillByteArrayWithRandomBytes(testRxData1);
    testSpiWriteRead<SPI_TRANSFER_FUNCTION::READ>(idx, testTxData1, testRxData1);
}

TEST_F(SPITest, testWrite){
    RODOS::SPI_IDX idx { RODOS::SPI_IDX0 };

    std::array<std::byte, 100> testTxData1;
    fillByteArrayWithRandomBytes(testTxData1);
    std::array<std::byte, 0> testRxData1;
    testSpiWriteRead<SPI_TRANSFER_FUNCTION::WRITE>(idx, testTxData1, testRxData1);
}

}
