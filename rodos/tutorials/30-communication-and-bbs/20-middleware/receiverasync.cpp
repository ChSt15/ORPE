#include "rodos.h"
#include "demo_topics.h"
#include "mask.h"

static Application receiverName("ReceiverAsync", 1300);

static Fifo<Position, 5> buf;
static Subscriber nameNotImportant01(position, buf, "position5");

class ReceiverAsync : public StaticThread<> {
    void run() {
        Position myPos;
        TIME_LOOP(20 * SECONDS, 12 * SECONDS) {
            while (buf.get(myPos)) {
                PRINTF(RX "%d" RY "%d" RZ "%d\n", (int) myPos.x, (int) myPos.y, (int) myPos.z);
            }
        }
    }
} receiverAsync;

