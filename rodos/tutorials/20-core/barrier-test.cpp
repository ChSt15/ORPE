#include "rodos.h"
#include "random.h"
#include "barrier.h"


Barrier barrier;

class Waiter : public StaticThread<> {
    void run() {
        while(1) {
            PRINTF("before barrier\n");
            bool ok = barrier.waitForSignal(true, NOW() + 2  * SECONDS);
            PRINTF("  waitFor = %s\n", ok ? "ok" : "Timeout");
        }
    }

  public:
    Waiter() : StaticThread<>() {}
} watingTrehad;


class SignalingThread : public StaticThread<> {
    void run() {
        TIME_LOOP(1 * SECONDS, 500 * MILLISECONDS) {
           int action = uint32Rand() % 4;
	   if(action == 0) {
               PRINTF("                            Unblocking\n");
               bool ok = barrier.unblock();
	       PRINTF("  unblock  = %d\n", ok);
	   } else {
               PRINTF("                            no action\n");
	   }
            
        }
    }

  public:
    SignalingThread() : StaticThread<>() {}
} signalingThread;


