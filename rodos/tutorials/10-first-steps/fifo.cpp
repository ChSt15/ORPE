#include "rodos.h"

static Application applic("FifoTest");

Fifo<int, 10> fifo; // FiFo for 10 Integer values

class Sender : public StaticThread<> {
    void run() {
        int cnt = 0;
        PRINTF("sender\n");
        while (1) {
            cnt++;
            bool ok = fifo.put(cnt); // puts the current counter value into the FiFo and checks if the FiFo is full
            if (ok) {
                PRINTF("Sending %d\n", cnt);
            } else {
                PRINTF("Fifo full\n");
            }
            if ((cnt % 15) == 0) {
                PRINTF("Waiting 3 seconds\n");
                AT(NOW() + 3 * SECONDS);
            }
        }
    }
};

class Receiver : public StaticThread<> {
    void run() {
        int cnt;
        PRINTF("receiver\n");

        while (1) {
            bool ok = fifo.get(cnt); // receives the current counter value from the FiFo and checks if the FiFo is empty
            if (ok) {
                PRINTF("reading %d\n", cnt);
            } else {
                AT(NOW() + 1 * SECONDS);
            }
        }
    }
};

Sender sender;
Receiver receiver;
