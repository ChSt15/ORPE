#include "rodos.h"

static Application nameNotImportantEV("resumeFromEvent");

class TestWaiter : public StaticThread<> {
public:
    void run() {
        while (1) {
            PRINTF("Suspend and wait until some one resumes me\n");
            AT(END_OF_TIME); // suspends the thread forever
            PRINTF("testWaiter running again at %3.9f\n", SECONDS_NOW());
        }
    }
};

static TestWaiter testWaiter;

class TimeEventTest : public TimeEvent { // defines an event
public:
    void handle() { // handles an event
        xprintf("   Time Event at %3.9f\n", SECONDS_NOW());
        testWaiter.resume(); // resumes the suspended thread
        xprintf("   testWaiter resumed from me\n");
    }

    // defines when the event is being raised; could also use activateAt(time)
    void init() { activatePeriodic(5 * SECONDS, 3 * SECONDS); }
};

static TimeEventTest te01;
