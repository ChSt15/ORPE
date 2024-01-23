#include "peripheral_ctrl/aborting_error.h"

namespace RODOS {
[[noreturn]] void abortingError(const char* message){
    throw message;
}
}
