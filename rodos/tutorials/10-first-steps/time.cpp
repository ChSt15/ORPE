#include "rodos.h"

static Application nameNotImportantTM("TestTimeAT");

static class TestTime : public StaticThread<> {

public:
    TestTime() : StaticThread<>("waitAT") {}

    void run() {
        PRINTF("waiting until 3rd second after start\n");
        AT(3 * SECONDS); // waits for the point in time: 3 seconds after start
        PRINTF("after 3rd second\n");

        PRINTF("waiting until 1 second has passed\n");
        AT(NOW() + 1 * SECONDS); // waits for 1 second
        PRINTF("1 second has passed\n");

        PRINTF("print every 2 seconds, start at 5 seconds\n");
        // code in the loop will be executed every 2 seconds; the first execution will be at 5 seconds after start
        TIME_LOOP(5 * SECONDS, 2 * SECONDS) {
            PRINTF("current time: %3.9f\n", SECONDS_NOW());
        }
    }

    void init() { PRINTF("Waiting time"); }

} testTime;
