#pragma once
#include "gtest/gtest.h"

namespace RODOS::testing {

class PeripheralTestFixture : public ::testing::Test {
protected:
    void SetUp() override;
    void TearDown() override;
};

}
