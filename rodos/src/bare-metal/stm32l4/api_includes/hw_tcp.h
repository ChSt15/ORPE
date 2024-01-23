#pragma once

// DUMMY!

namespace RODOS {

class HW_TCPServer {
public:
    ~HW_TCPServer();

    bool listen([[maybe_unused]] const long portNr) { return false; };
    bool acceptNewConnection() { return false; };
    int sendData([[maybe_unused]] void* buf, [[maybe_unused]] int len) { return 0; };
    int getData([[maybe_unused]] void* buf, [[maybe_unused]] int maxLen) { return 0; };
    int getErrorCode() const { return -1; };
};

class HW_TCPClient {
public:
    ~HW_TCPClient();

    bool reopen([[maybe_unused]] const long portNr, [[maybe_unused]] const char* hostname = "localhost") { return false; };
    int sendData([[maybe_unused]] void* buf, [[maybe_unused]] int len) { return 0; };
    int getData([[maybe_unused]] void* buf, [[maybe_unused]] int maxLen) { return 0; };
    int getErrorCode() const { return -1; };
};

} // namespace
