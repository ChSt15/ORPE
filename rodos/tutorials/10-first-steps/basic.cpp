#include "rodos.h" // includes the RODOS API

static Application nameNotImportantHW("HelloWorld"); // application that wraps all threads, events,... in this file

class HelloWorld : public StaticThread<> { // defines a thread named HelloWorld

public:
    HelloWorld() : StaticThread<>("HelloWorld") {} // thread constructor with definition of the thread name

    void init() { // is called before the scheduler starts
        PRINTF(SCREEN_RED "This is init() for Printing Hello World" SCREEN_RESET);
    }

    void run() { // implements the code for the main task of the thread; In this case: print "Hello World!"
        PRINTF("Hello World!\ndid you see the init in red?\n");
    }
};

HelloWorld helloWorld; // instantiates one thread of type HelloWorld
