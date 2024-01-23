#include "gtest/gtest.h"
#include "hw_hal/can/can_filter.h"

#include <random>
namespace {

using RODOS::CanFilter;
using RODOS::VA41620CanFrameMetadata;


TEST(CanFilterTest, TestRtrAndIdeFlagRecognition){
    VA41620CanFrameMetadata basicFrame { 0, false, false };
    VA41620CanFrameMetadata extendedFrame { 0, true, false };
    VA41620CanFrameMetadata basicRtrFrame { 0, false, true };
    VA41620CanFrameMetadata extendedRtrFrame { 0, true, true };
    for (bool rtr : { true, false }){
        for (bool ide : { true, false }){
            constexpr uint32_t idMask { 0x1fff'ffff };
            CanFilter filter {0, idMask, ide, rtr, nullptr};
            ASSERT_EQ(filter.matches(basicFrame)      , (!rtr)&&(!ide));
            ASSERT_EQ(filter.matches(extendedFrame)   , (!rtr)&&( ide));
            ASSERT_EQ(filter.matches(basicRtrFrame)   , ( rtr)&&(!ide));
            ASSERT_EQ(filter.matches(extendedRtrFrame), ( rtr)&&( ide));
        }
    }
}

TEST(CanFilterTest, TestRandomExtendedIdMatching){
    std::random_device rd;
    std::mt19937 mt { rd() };
    std::uniform_int_distribution<uint32_t> idDistribution {0, 0x1fff'ffff};

    //note: very large number because of low probability of random filter matching random number
    constexpr size_t numValuesToTest { 10'000'000 };
    for(size_t i = 0; i<numValuesToTest; i++){
        uint32_t filterIdMask { idDistribution(mt) };
        uint32_t filterId { idDistribution(mt) };
        CanFilter filter { filterId, filterIdMask, true, false, nullptr};
        uint32_t testedId { idDistribution(mt) };
        VA41620CanFrameMetadata testedMetadata { testedId, true, false };
        bool shouldMatch { ((testedId&~filterIdMask)^(filterId&~filterIdMask)) == 0 };
        ASSERT_EQ(filter.matches(testedMetadata), shouldMatch)
            << "Error with Mask " << filterIdMask
            << " and ID " << filterId
            << " and tested ID "
            << testedId;
    }
}

TEST(CanFilterTest, TestRandomBasicIdMatching){
    std::random_device rd;
    std::mt19937 mt { rd() };
    std::uniform_int_distribution<uint32_t> idDistribution {0, 0x7ff};

    constexpr size_t numValuesToTest { 1'000'000 };
    for(size_t i = 0; i<numValuesToTest; i++){
        uint32_t filterIdMask { idDistribution(mt) };
        uint32_t filterId { idDistribution(mt) };
        CanFilter filter { filterId, filterIdMask, false, false, nullptr};
        uint32_t testedId { idDistribution(mt) };
        VA41620CanFrameMetadata testedMetadata { testedId, false, false };
        bool shouldMatch { ((testedId&~filterIdMask)^(filterId&~filterIdMask)) == 0 };
        ASSERT_EQ(filter.matches(testedMetadata), shouldMatch)
            << "Error with Mask " << filterIdMask
            << " and ID " << filterId
            << " and tested ID "
            << testedId;
    }
}

}
