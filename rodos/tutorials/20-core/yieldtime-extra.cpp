#include "rodos.h"

static Application module01("YieldTime-extra");

class SimpleThread : public StaticThread<> {
    void      run() {
        uint64_t yieldCnt = 0;
        while(1) {
            yield();
            yieldCnt++;
            asm("": "+g"(yieldCnt) : :); // avoid complier optimization of busy loop
            if((yieldCnt % 100000) == 0) {
                long long timeNow = NOW();
                PRINTF(" %s: cnt = %lld, totalyields = %lld, Time = %3.9f  nsec/dispatchCnt = %ld\n",
                       getName(),
                       yieldCnt,
                       getScheduleCounter(),
                       (double)timeNow / SECONDS,
                       (long)(((double)timeNow / (double)getScheduleCounter()) * NANOSECONDS));
            }
        }
    }

  public:
    SimpleThread() : StaticThread<>("Simple") { }
    void init() { PRINTF(" Simple Thread activated"); }
};

/*****************************/

SimpleThread x[20];
