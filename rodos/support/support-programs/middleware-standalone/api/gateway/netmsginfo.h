
/**
* @file subscriber.h
* @date 2008/09/01 
* @author Sergio Montenegro
*      Update: ting, 2022/03/10, Sergio Montenegro 2022/08/01
*
*/

#pragma once

#include <stdint.h>


#include <algorithm>
#include <cstdint>
#include <cstring>
#include "checksumes.h"
#include "stream-bytesex.h"


#include "default-platform-parameter.h"
/******** Identification of Messages ************/
namespace RODOS {

enum class NetMsgType  : uint16_t { // set but not used until now
    PUB_SUB_MSG    = 0,  // default message type, 
    TOPIC_LIST,          // See receiverNode+receiverNodesBitMap.txt
    P2P_RELIABLE,
    P2P_ACK,
    REQUEST,
    RESPONSE

    // TIME_SYNC, // not used until now
    // BROADCAST,  // not used until now
    // RECONFIG,  // not used until now
    // HEARTBEAT,  // not used until now
    // ERROR,  // not used until now
};


constexpr uint32_t LINK_ID_RODOS_LOCAL_BROADCAST = 0; // What is that? (SM) DEPRECATED !

constexpr uint32_t MAX_NUM_OF_NODES = 32; // Warning: in an 32-bit int each bit   reporesents a node from 0 to 31


/**
 * Data structure containing information about topic messages sent via network (gateway).
 * Used only intern in the node. not transmited to the network,
 * See NetworkMessage, Why not the same? This is due to historical reasons
 * WARNING: Keep consistent with NetworkMessage !!!!
 */

class NetMsgInfo {
  public:
    int32_t    senderNode;     ///< Set by sendig application, set in publish()
    uint32_t   senderThreadId; ///< The ID of the sending thread, Set in publish()
    int64_t    sentTime;       ///< Time in localTime units, Set in pulbish()
    int32_t    receiverNode;   ///< See receiverNode+receiverNodesBitMap.txt
    uint32_t   receiverNodesBitMap;  ///< See receiverNode+receiverNodesBitMap.txt
    uint32_t   linkId;         ///< The ID of the Linkinterface from which the message was received. Set by Linkinterface
    NetMsgType messageType;    ///< The type of the message, set by sender

    //NetMsgInfo (NetMsgType type = NetMsgType::PUB_SUB_MSG) { init(type); }
    
    NetMsgInfo (NetMsgType type = NetMsgType::PUB_SUB_MSG) { } // CHANGED FAISAL

/*
    void init(NetMsgType type = NetMsgType::PUB_SUB_MSG) {
         linkId         = LINK_ID_RODOS_LOCAL_BROADCAST; // Whatfor ?? (SM)
         sentTime       = NOW();
         senderNode     = getNodeNumber();
         intptr_t ptr   = (intptr_t)(Thread::getCurrentThread());
         senderThreadId = static_cast<uint32_t>(ptr);
         messageType    = type;
         receiverNode   = -1; // Not used until now, but 0xffffffff shall be broadcast
    } */
};

/**
 * Simple message data protocol to transmit data to a remote node.
 * Header is serialized NetMsgInfo, followed by the user data
 */
class NetworkMessage {
    static constexpr uint16_t HEADER_SIZE = 36;
    uint8_t header [HEADER_SIZE];
public:
    inline void    put_receiverNode(int32_t x)         {int32_tToBigEndian(header + 0, x); } // see receiverNode+receiverNodesBitMap.txt
    inline void    put_receiverNodesBitMap(uint32_t x) {uint32_tToBigEndian(header+ 4, x); } // see receiverNode+receiverNodesBitMap.txt
    inline void    put_maxStepsToForward(int16_t x)    {int16_tToBigEndian(header + 8, x); }
    inline void    put_checkSum(uint16_t x)            {uint16_tToBigEndian(header+10, x); }
    // from here protected with checksum : next index (12) will be used as literal : calculateCheckSum()
    inline void    put_type(uint16_t x)                {uint16_tToBigEndian(header+12, x); }
    inline void    put_len(uint16_t x)                 {uint16_tToBigEndian(header+14, x); } // len of only user data
    inline void    put_senderNode(int32_t x)           {int32_tToBigEndian(header +16, x); }
    inline void    put_sentTime (int64_t x)            {int64_tToBigEndian(header +20, x); }
    inline void    put_senderThreadId(uint32_t x)      {uint32_tToBigEndian(header+28, x); }
    inline void    put_topicId(uint32_t x)             {uint32_tToBigEndian(header+32, x); }

    inline int32_t  get_receiverNode()         const { return bigEndianToInt32_t(header + 0); } // see receiverNode+receiverNodesBitMap.txt
    inline uint32_t get_receiverNodesBitMap()  const { return bigEndianToUint32_t(header+ 4); } // see receiverNode+receiverNodesBitMap.txt
    inline int16_t  get_maxStepsToForward()    const { return bigEndianToInt16_t(header + 8); }
    inline uint16_t get_checksum()             const { return bigEndianToUint16_t(header+10); }
    // from here protected with checksum : next index (12) will be used as literal : calculateCheckSum()
    inline uint16_t get_type()                 const { return bigEndianToUint16_t(header+12); }
    inline uint16_t get_len()                  const { return bigEndianToUint16_t(header+14); } // only user data
    inline int32_t  get_senderNode()           const { return bigEndianToInt32_t(header +16); }
    inline int64_t  get_sentTime ()            const { return bigEndianToInt64_t(header +20); }
    inline uint32_t get_senderThreadId()       const { return bigEndianToUint32_t(header+28); }
    inline uint32_t get_topicId()              const { return bigEndianToUint32_t(header+32); }

    uint8_t userDataC[MAX_NETWORK_MESSAGE_LENGTH]; ///< local buffer for user data ca 1300. See platform-parameter.h

    /**
     * Copy constructor and assign operator that only copes nesacary data.
     */
    NetworkMessage() { }
    NetworkMessage(const NetworkMessage &oldMsg) { memcpy(this, &oldMsg, oldMsg.numberOfBytesToSend()); }
    NetworkMessage& operator= (const NetworkMessage& oldMsg) {
        if(this != & oldMsg) memcpy(this, &oldMsg, oldMsg.numberOfBytesToSend());
        return *this;
    }

    inline void dec_maxStepsToForward()    { put_maxStepsToForward(static_cast<int16_t>(get_maxStepsToForward() - 1)); }

    /** Copies user generated data into the message body.
     *
     * @param data pointer to input data stream to be transfered
     * @param length of message, supposed to be <= MAX_NETWORK_MESSAGE_LENGTH
     * @return length of message written
     */
    uint16_t setUserData(const void* data, uint16_t len);

    /** Copies data from message received to a user provided buffer.
     *
     * @param destination pointer to output data stream received
     * @param length of message buffer, supposed to be <= MAX_NETWORK_MESSAGE_LENGTH
     * @return length of message copied
     */
    uint16_t getUserData(void* destination, uint16_t maxLen);

    /* WARNING: Len has to be set befor you call this.  **/
    uint16_t numberOfBytesToSend() const { return static_cast<uint16_t>(HEADER_SIZE + get_len()); }
    uint16_t calculateCheckSum()         { return checkSum(header+12, HEADER_SIZE-12u + get_len()); } // exection: crc-self and fields modified in routers
    bool     isCheckSumOk()              { return calculateCheckSum() == get_checksum(); }
    void     setCheckSum()               { put_checkSum(calculateCheckSum()); }

} __attribute__((packed));

}  // namespace

