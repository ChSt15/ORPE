#include "rodos.h"
#include "demo_topics.h"
#include "mask.h"

static Application receiverName("ReceiverComBuffer", 1200);

static CommBuffer<Position> buf;
static Subscriber nameNotImportant02(position, buf, "receiverBuf");

class ReceiverBuf : public StaticThread<> {
    void run() {
        Position myPos;
        TIME_LOOP(10 * SECONDS, 2 * SECONDS) {
            buf.get(myPos);
            PRINTF(CX "%d" CY "%d" CZ "%d\n", (int) myPos.x, (int) myPos.y, (int) myPos.z);
        }
    }
} recbuf;

