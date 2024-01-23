#include "rodos.h"


class LowPriorityThread : public StaticThread<> {
public:
    LowPriorityThread() : StaticThread<>("LowPriority", 100) {}

    void run() {
        int64_t cnt = 0;
        int64_t intervalToPrint = getSpeedKiloLoopsPerSecond() * 10;
        xprintf("\nLow prio begins printing '.'\n");
        while (1) {
            asm("": "+g"(cnt) : :); // avoid complier optimization of busy loop
            if (++cnt % intervalToPrint == 0) {
                xprintf(".");
                FFLUSH();
                AT(NOW() + 10 * MILLISECONDS);
            }
        }
    }
} lowPrio;

//__________________________________________________________

class HiPriorityThread : public StaticThread<> {
public:
    HiPriorityThread() : StaticThread<>("HiPriority", 200) {}

    void run() {
        int64_t cnt = 0;
        int64_t intervalToPrint = getSpeedKiloLoopsPerSecond() * 10;
        xprintf("\nHi prio waits 1 second and then print +\n");
        AT(1 * SECONDS);
        xprintf("\nHi Priority in busy waiting\n");
        while (NOW() < 5 * SECONDS) {
            asm("": "+g"(cnt) : :); // avoid complier optimization of busy loop
            if (++cnt % intervalToPrint == 0) {
                xprintf("+");
                FFLUSH();
            }
        }


        xprintf("\nHi prio in busy waiting in PRIORITY_FLOORING\n");
        {
            PRIORITY_FLOORING_IN_SCOPE();
            while (NOW() < 10 * SECONDS) {
                cnt++;
                asm("": "+g"(cnt) : :); // avoid complier optimization of busy loop
                if (cnt % intervalToPrint == 0) {
                    xprintf("+");
                    FFLUSH();
                }
            }
        }
        xprintf("\nHi prio Ends\n");
    }
} hiPri;
