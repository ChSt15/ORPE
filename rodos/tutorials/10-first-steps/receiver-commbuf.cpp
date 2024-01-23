#include "rodos.h"
#include "topics.h"

static Application receiverName("ReceiverComBuffer", 1200);

static CommBuffer<long> buf;

// subscriber that fills the CommBuffer buf with values from topic counter1
static Subscriber receiverBuf(counter1, buf, "receiverBuf");

class ReceiverBuf : public StaticThread<> {
    void run() {
        long cnt;
        TIME_LOOP(0, 1100 * MILLISECONDS) {
            buf.get(cnt); // the thread gets the latest value
            PRINTF("ReceiverComBuffer - counter1: %ld\n", cnt);
        }
    }
} receiverBufThread;
