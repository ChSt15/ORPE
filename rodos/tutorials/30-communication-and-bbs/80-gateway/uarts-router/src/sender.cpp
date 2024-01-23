#include <stddef.h>

#include "rodos.h"
#include "demo_topics.h"

class SenderTrd : public StaticThread<> {
protected:
    TestData data;

public:
    SenderTrd() : StaticThread<>("Sender") { }

    void generateRandomMessage() {
        for (size_t j = 0; j < this->data.length; j++) {
            this->data.msg[j] = static_cast<uint8_t>(uint64Rand());
        }
        this->data.checksum = checkSum(this->data.msg, static_cast<size_t>(this->data.length));
    }

    void run() override {
        setRandSeed(static_cast<uint64_t>(getNodeNumber()));

        TIME_LOOP(5 * SECONDS, 3 * SECONDS) {
            this->generateRandomMessage();
            PRINTF("[topic1] publish: 0x%04x\n", this->data.checksum);
            topic1.publish(this->data);

            this->generateRandomMessage();
            PRINTF("[topic2] publish: 0x%04x\n", this->data.checksum);
            topic2.publish(this->data);
        }
    }
} sender;
