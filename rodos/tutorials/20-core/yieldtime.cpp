#include "rodos.h"
#include <atomic>

static Application module01("YieldTimeMeasure");

Semaphore printProtect;

std::atomic<uint32_t> yieldGlobal = 0;

class TestThread : public StaticThread<> {

    void run() {
        uint32_t yieldCnt = 0;
        while (1) {
            yield();
            yieldCnt++;
            asm("": "+g"(yieldCnt) : :); // avoid complier optimization of busy loop
            yieldGlobal++;
            // displays every 100000 times
            if ((yieldCnt % 100000) == 0) {
                long long timeNow = NOW();
                printProtect.enter();
                PRINTF(" %s: ", getName());
                PRINTF("cnt = %ld, ", (long) yieldCnt);
                PRINTF("totalyields = %ld, ", (long) getScheduleCounter());
                PRINTF("Time = %3.9f, ", (double) timeNow / SECONDS);
                PRINTF("nsec/dispatchCnt = %ld  ",
                       (long) (((double) timeNow / (double) getScheduleCounter()) * NANOSECONDS));
                PRINTF("nsec/yieldCnt = %ld\n", (long) (((double) timeNow / (double) yieldGlobal) * NANOSECONDS));
                printProtect.leave();
            }
        }
    }
    //constructor
public:
    TestThread(const char *name = "xx") : StaticThread<>(name) { }

    void init() { PRINTF(" Thread activated"); }
};

/*****************************/
TestThread a("A(1)"), b("B(2)"), c("C(3)");
