
/**
* @file hw_udp.h
* @date 2008/07/15 16:35
* @author Sergio Montenegro, Lutz Dittrich
*
*
* @brief UDP communication via sockets (Linux)
*/

#pragma once

#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "putter.h"
#include "topic.h"

namespace RODOS {
/**
*  UDP interface for Linux platform, using sockets
*/

/**
 *    UDP input port for Linux
 */
class UDPReceiver {

  private:
    int         sock;
    sockaddr_in inputAddr;
    bool        enableMultiReader;
    //    Putter* incomingDataHandler;  // What for? never used?

  public:
    /** Constructor
     * Warning: negative port number means more than one can receive on the same
     * @param portNr port number on localhost used for reception , negative for broadcast
     */
    UDPReceiver(const int32_t port);
    /** Destructor */
    ~UDPReceiver();

    /** Opens a socket for UDP reception.
     * On success attribute 'initialised' is set true.
     * Warning: negative port number means more than one can receive on the same
     * @param portNr port number on localhost, negative for broadcast
     */
    void reopen(const int32_t port);

    void setAsync(Topic<GenericMsgRef>* associatedTopic);

    void joinMulticastGroup(uint32_t ipaddr);

    /**
     * Receives data from Linux UDP socket. Writes data up to maxLen to userData.
     * @param[IN/OUT] userData pointer to input buffer
     * @param[IN] size of input buffer
     * @return length of message written to userData
     */
    int32_t get(void* userData, const size_t maxLen = 1300);
    int32_t get(void* userData, const size_t maxLen, uint32_t* ipaddr); // return number of bytes read, or 0, or < 0 if error
    bool readyToGet();
    bool isInitialised() { return initialised; } // due to windows compatibility
  private:
    bool initialised;
};


/**
*	output UDP port for Linux
*/
class UDPTransmitter {
  private:
    int         sock;
    hostent*    hp;
    sockaddr_in outputAddr;

    bool enableBroadCast;

  public:
    /** init udp communication, can be re-called at any time
     ** WARNING: Negative port number means broadcast **/

    UDPTransmitter(const int32_t port, const char* host = "localhost");
    UDPTransmitter(const int32_t _portNr, uint32_t _ipAddr); // _ipAddr as eg. 192.168.0.20
    UDPTransmitter(const int32_t _portNr, int ip0, int ip1, int ip2, int ip3);
    virtual ~UDPTransmitter();

    void openConnection(const int32_t port, const char* host);


    /** Send it as datagram containing "userdata", default length = all bytes
     * @param data pointer to datafield
     * @param length of data field
     */
    virtual bool send(const void* msg, const size_t len);

    /** Send datagram to a specified host different to the address used during initialisation
     * @param data pointer to datafield
     * @param length of data field
     * @param ipAddr of receiving host
     */
    bool sendTo(const void* userData, const size_t maxLen, uint32_t ipAddr);
    bool isInitialised() { return initialised; } // due to windows compatibility
  private:
    bool initialised;
};


/********* Global Functions **************/

bool setMyIP(int ip0, int ip1, int ip2, int ip3);     // eg. setMyIP(192,168,20,10);
bool setMyIPMask(int ip0, int ip1, int ip2, int ip3); // eg. setMyIPMask(255,255,0,0);

void init_sigio_handler();

} // namespace RODOS
