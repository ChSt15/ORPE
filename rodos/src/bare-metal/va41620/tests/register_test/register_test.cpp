#include "cortex_m/register_access/register.h"
#include "gtest/gtest.h"

namespace {

TEST(TestRegister, WriteRegisterDoesSetValue) {
    RODOS::Register reg;

    EXPECT_CALL(static_cast<RODOS::MMIO&>(reg), write(3u));
    reg.write(3u);
}

}
