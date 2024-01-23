#include "rodos.h"
#include "demo_topics.h"

class Receiver1 : public SubscriberReceiver<TestData> {
public:
    Receiver1() : SubscriberReceiver<TestData>(topic1, "Receiver1") {}

    void put(TestData &data) override {
        uint16_t csum = checkSum(data.msg, data.length);
        if (data.checksum == csum)
            PRINTF("received: 0x%04x\n", data.checksum);
        else
            PRINTF("wrong checksum: expected 0x%04x, got 0x%04x\n", data.checksum, csum);
    }
} receiver1;
