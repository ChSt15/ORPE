#include "rodos.h"
#include "random.h"

struct TestData {
    size_t   length = 64;
    uint8_t  msg[64];
    uint16_t checksum;
};

extern Topic<TestData> topic1, topic2;