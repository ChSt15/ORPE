#include "rodos.h"

class HelloWorld : public StaticThread<> {
    void run() {
        PRINTF("Hello ");   // Be aware of preemption at this point
        PRINTF("World!\n"); // what you will see shall not be a surprise
    }
};

HelloWorld helloWorld[10];
