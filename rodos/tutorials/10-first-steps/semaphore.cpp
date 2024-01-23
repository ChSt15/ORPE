#include "rodos.h"

Semaphore onlyOne; // semaphore definition outside the threads
char globalCnt = 'A';

class Watcher : public StaticThread<> {
public:
    void run() {

        char myId;

        //distribute the ids
        onlyOne.enter();
        myId = globalCnt++;
        onlyOne.leave();

        while (1) {
            onlyOne.enter(); // enters semaphore "onlyOne"
            PRINTF(" only one, I am -- %c -- ", myId);
            // because of the semaphore, the upcoming yield cannot interrupt between the print statements
            yield(); // Just to show that the semaphore protects
            PRINTF("time %3.9f\n", SECONDS_NOW());
            onlyOne.leave(); // leaves semaphore "onlyOne"
        }
    }

} team[20];
