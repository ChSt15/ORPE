#include <string.h>
#include "rodos.h"
#include "hw_udp.h"


namespace RODOS {


UDPReceiver::UDPReceiver([[maybe_unused]] const int32_t port)
{
}
UDPReceiver::~UDPReceiver()
{
}
void UDPReceiver::reopen([[maybe_unused]] const int32_t port)
{
}
/*
bool UDPReceiver::isConnected()
{
    if(pcbUdp == NULL)
        return false;
    else
        return true;
}
*/
bool UDPReceiver::readyToGet()
{
    return false;
}

void UDPReceiver::setAsync([[maybe_unused]] Topic<GenericMsgRef>* associatedTopic){}

int32_t UDPReceiver::get([[maybe_unused]] void* userData, [[maybe_unused]] const size_t maxLen)
{
    return 0;
}
int32_t UDPReceiver::get([[maybe_unused]] void* userData, [[maybe_unused]] const size_t maxLen, [[maybe_unused]] uint32_t *ipaddr)
{
    return 0;
}

UDPTransmitter::UDPTransmitter([[maybe_unused]] const int32_t port, [[maybe_unused]] const char *host)
{
}
UDPTransmitter::UDPTransmitter([[maybe_unused]] const int32_t _portNr, [[maybe_unused]] uint32_t _ipAddr)
{
}
UDPTransmitter::UDPTransmitter([[maybe_unused]] const int32_t _portNr, [[maybe_unused]] int ip0, [[maybe_unused]] int ip1, [[maybe_unused]] int ip2, [[maybe_unused]] int ip3)
{
}
UDPTransmitter::~UDPTransmitter() { }

void UDPTransmitter::openConnection([[maybe_unused]] const int32_t port, [[maybe_unused]] const char *host) { }

bool UDPTransmitter::send([[maybe_unused]] const void* msg, [[maybe_unused]] const size_t len)
{
        return false;
}
bool UDPTransmitter::sendTo([[maybe_unused]] const void* userData, [[maybe_unused]] const size_t maxLen, [[maybe_unused]] uint32_t _ipAddr)
{
        return false;
}

}
