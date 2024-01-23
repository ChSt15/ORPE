
/**
* @file hw_udp.h
* @date 2008/07/15 16:35
* @author Sergio Montenegro
*
*/


#include <sys/types.h>
//#include <netinet/in.h>

//#include "hw_datatypes.h"
#include "topic.h"

#pragma once

namespace RODOS {

/**
*  UDP interface for Linux platform, using sockets
*/


/**
 *    UDP input port for linux
 */
class UDPReceiver {

private:

public:
    UDPReceiver( __attribute__((unused))const int port) { }
    ~UDPReceiver()              { }

    void reopen( __attribute__((unused))const int port) { }
 
    void setAsync( __attribute__((unused))Topic<GenericMsgRef>* associatedTopic) { }

    long get( __attribute__((unused))void* userData,   __attribute__((unused))const unsigned int maxLen=1300)    { return 0; }
    long get( __attribute__((unused))void* userData,  __attribute__((unused))int maxLen,  __attribute__((unused))unsigned long *ipaddr) { return 0; }
    bool readyToGet()                                           { return false;}
    bool isInitialised()                                        { return true; } 
};


class UDPTransmitter {
private:

public:

    /** init udp communication, can be re-called at any time
     ** WARNING: Negative port number means broadcast **/

    UDPTransmitter( __attribute__((unused))const int port,   __attribute__((unused))const char *host = "localhost")         { }
    UDPTransmitter(__attribute__((unused))const long _portNr,  __attribute__((unused))unsigned long _ipAddr)               { }
    UDPTransmitter( __attribute__((unused))const long _portNr,  __attribute__((unused))int ip0,  __attribute__((unused))int ip1,  __attribute__((unused))int ip2,  __attribute__((unused))int ip3)  { }
    virtual ~UDPTransmitter()                                               { }

    void openConnection( __attribute__((unused))const int port,  __attribute__((unused))const char *host)                  { }

    /** Send it as datagram containing "userdata", default length = all bytes
     * @param data pointer to datafield
     * @param length of data field
     */
    virtual bool send( __attribute__((unused))const void*msg,  __attribute__((unused))const unsigned int len)               { return false; }

    /** Send datagram to a specified host different to the address used during initialisation
     * @param data pointer to datafield
     * @param length of data field
     * @param ipAddr of receiving host
     */
    bool sendTo( __attribute__((unused))const void* userData,  __attribute__((unused))const int maxLen,  __attribute__((unused))unsigned long ipAddr) { return false; }
    bool isInitialised()                                                      { return true; } 
};

} // namespace

