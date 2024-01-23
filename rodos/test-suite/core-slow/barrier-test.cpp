#include "rodos.h"
#include "random.h"
#include "barrier.h"

uint32_t printfMask = 0;

Barrier barrier;

class Waiter : public StaticThread<> {
    void run() {
        printfMask = 1;
        while (true) {
            PRINTF("before barrier\n");
            bool ok = barrier.waitForSignal(true, NOW() + 2 * SECONDS);
            PRINTF("  waitFor = %s\n", ok ? "ok" : "Timeout");
        }
    }

  public:
    Waiter() : StaticThread<>() {}
} waitingThread;


class SignalingThread : public StaticThread<> {
    void run() {
        TIME_LOOP(1 * SECONDS, 500 * MILLISECONDS) {
            int action = uint32Rand() % 4;
            if (action == 0) {
                PRINTF("                            Unblocking\n");
                bool ok = barrier.unblock();
                PRINTF("  unblock  = %d\n", ok);
            } else {
                PRINTF("                            no action\n");
            }

            if (NOW() > 10 * SECONDS) hwResetAndReboot();
        }
    }

  public:
    SignalingThread() : StaticThread<>() {}
} signalingThread;
