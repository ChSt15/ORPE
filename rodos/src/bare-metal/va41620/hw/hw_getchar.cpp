#include "rodos.h"
#include "hw_uart_dbg.h"

namespace RODOS {

char getcharNoWait() {
    int c = uartStdout.getcharNoWait();
    if(c<0) {
        return 0;
    } else {
        return static_cast<char>(c);
    }
}

bool isCharReady() { 
    return uartStdout.isDataReady();
}

char* getsNoWait() {
    constexpr auto IN_BUFFER_SIZE { 122 };
    static constinit char inBuffer[IN_BUFFER_SIZE];
    static constinit int inBufferIndex { 0 };

    while(1) {
        char c = getcharNoWait();
        if(c == 0) return 0;

        if(c == '\r' || c == '\n') {
            inBuffer[inBufferIndex] = 0;
            inBufferIndex = 0;
            return inBuffer;
        }

        inBuffer[inBufferIndex] = c;
        if(inBufferIndex < 120) inBufferIndex++;
    }
}

void activateTopicCharInput() { }

}
