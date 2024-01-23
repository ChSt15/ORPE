#include "rodos.h"

static Application nameNotImportantCB("ComBufTest");

CommBuffer<int> buf; // CommBuffer of type Integer

class Sender : public StaticThread<> {
    void run() {
        int cnt = 0;
        while (1) {
            cnt++;
            PRINTF("Writing %d\n", cnt);
            buf.put(cnt); // thread puts local counter data into the CommBuffer
            AT(NOW() + 3 * SECONDS);
        }
    }
};

class Receiver : public StaticThread<> {
    void run() {
        int cnt;
        while (1) {
            buf.get(cnt); // thread gets counter data from the CommBuffer and saves it into local variable
            PRINTF("Reading %d\n", cnt);
            AT(NOW() + 2 * SECONDS);
        }
    }
};

Sender sender;
Receiver receiver;
