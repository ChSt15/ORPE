#pragma once
#include <cstdint>

namespace RODOS {

class CPUManager {
public:
    static void setupDebugging();
    static uint32_t getFirstUUIDWord();
};

}
