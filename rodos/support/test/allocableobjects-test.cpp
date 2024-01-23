#include "../support-libs/allocableobjects.h"

#include <gtest/gtest.h>

struct TestObject {
    int a, b, c;
};

TEST(AllocableObjects, init) {
    const uint32_t                                       NUM_OF_ELEMENTS = 8;
    RODOS::AllocableObjects<TestObject, NUM_OF_ELEMENTS> pool;

    pool.init();
    EXPECT_EQ(pool.getNumOfFreeItems(), NUM_OF_ELEMENTS);
}

TEST(AllocableObjects, sharedPtr) {
    RODOS::AllocableObjects<TestObject, 2> pool;

    auto allocResult = pool.alloc();
    ASSERT_TRUE(allocResult.isOk());
    auto alloc1 = allocResult.val;

    allocResult = pool.alloc();
    ASSERT_TRUE(allocResult.isOk());

    auto alloc2a = allocResult.val;
    {
        auto alloc2b = allocResult.val;

        allocResult = pool.alloc();
        ASSERT_FALSE(allocResult.isOk());
    } // one alloc2 reference remaining
    allocResult = pool.alloc();
    ASSERT_FALSE(allocResult.isOk());

    alloc2a.clear();
    // both alloc2 references destroyed, object becomes free in the pool

    allocResult = pool.alloc();
    ASSERT_TRUE(allocResult.isOk());
}
