#include "rodos.h"
#include "demo_topics.h"

class Receiver2 : public SubscriberReceiver<TestData> {
public:
    Receiver2() : SubscriberReceiver<TestData>(topic2, "Receiver2") {}

    void put(TestData &data) override {
        uint16_t csum = checkSum(data.msg, data.length);
        if (data.checksum == csum)
            PRINTF("received: 0x%04x\n", data.checksum);
        else
            PRINTF("wrong checksum: expected 0x%04x, got 0x%04x\n", data.checksum, csum);
    }
} receiver2;
