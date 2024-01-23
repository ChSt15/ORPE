#include "rodos.h"
#include "topics.h"

static Application receiverSimple("ReceiverSimple", 1100);

class SimpleSub : public Subscriber {
public:
    SimpleSub() : Subscriber(counter1, "simpleSub") {} // subscribing for counter1

    // the put function is called everytime new data has been published;
    // receive the data in this method and send it to a thread via CommBuffer or FiFo
    // (it is also possible to resume a thread when new data is available)
    uint32_t put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo) {
        PRINTF("SimpleSub - Length: %lu Data: %ld TopicId: %lu \n",
               static_cast<unsigned long>(len),
               *static_cast<long *>(data),
               static_cast<unsigned long>(topicId));
        return 1;
    }
} simpleSub; // try this: , a, b, c;
