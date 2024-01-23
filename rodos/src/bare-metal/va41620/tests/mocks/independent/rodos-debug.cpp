#include <iostream>
#include "rodos-debug.h"

#include "gtest/gtest.h"
#include <string>

namespace RODOS {

void RODOS_ERROR(const char* text) {
    FAIL() << "RODOS_ERROR called with message: " << text;
}



}
