#include "rodos.h"
#include "topics.h" // includes the topics

static Application senderName("Publisher 01 simple", 1100);

class MyPublisher01 : public StaticThread<> {
public:
    MyPublisher01() : StaticThread<>("SenderSimple") {}

    void run() {
        long cnt = 0;
        TIME_LOOP(3 * SECONDS, 3 * SECONDS) {
            PRINTF("Publisher01 sending Counter1 %ld\n", ++cnt);
            counter1.publish(cnt); // publishes every 3 seconds the incremented counter
        }
    }
} publisher01;
