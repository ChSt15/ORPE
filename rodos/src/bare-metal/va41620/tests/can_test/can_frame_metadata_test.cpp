#include "gtest/gtest.h"
#include "hw_hal/can/can_frame_metadata.h"
#include <random>

namespace {

TEST(CanFrameMetadataTest, testRoundtrip){
    constexpr size_t testedIdsPerCombination { 1'000 };
    std::random_device rd;
    std::mt19937 mt { rd() };
    std::uniform_int_distribution<uint32_t> extendedIdDistribution(0, 0x1fff'ffff);
    std::uniform_int_distribution<uint32_t> basicIdDistribution(0, 0x7ff);
    for(bool ide : { true, false }){
        std::uniform_int_distribution<uint32_t>& idDistribution { ide ? extendedIdDistribution : basicIdDistribution };
        for(bool rtr : {true, false}){
            for(size_t i = 0; i<testedIdsPerCombination; i++){
                uint32_t id { idDistribution(mt) };
                RODOS::VA41620CanFrameMetadata metadata { id, ide, rtr };
                ASSERT_EQ(metadata.getId(), id);
                ASSERT_EQ(metadata.getRTR(), rtr);
                ASSERT_EQ(metadata.getIde(), ide);
            }
        }
    }
}

}
