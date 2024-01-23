
#include "thread.h"

extern volatile long *contextT;

namespace RODOS {


uintptr_t Thread::getCurrentStackAddr(){
    return reinterpret_cast<uintptr_t>(context);
}
}

