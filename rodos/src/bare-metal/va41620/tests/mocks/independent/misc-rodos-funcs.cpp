#include "misc-rodos-funcs.h"
#include <cstdlib>

namespace RODOS {

void* xmalloc(size_t len){
    return std::malloc(len);
}

void* xmallocAndTrapOnFailure(size_t len){
    return std::malloc(len);
}

}
