#include "rodos.h" // includes the RODOS API

class HelloWorld : public StaticThread<> { // defines a Thread named HelloWorld
    void run() { // implements the code of the main task of the thread
        PRINTF("Hello World!\n"); // print "hello World!"
    }
} helloworld; // instantiates one thread of type HelloWorld
