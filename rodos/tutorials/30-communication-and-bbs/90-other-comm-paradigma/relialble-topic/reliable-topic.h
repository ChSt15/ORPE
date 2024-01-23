/**
 * @file reliable-topic.h
 * @date 2022/05/21 
 * @author Sergio Montenegro
 *
 *
 * @brief synchonous reliable topic for middleware
 *
 */

#pragma once

#include "topic.h"
#include "subscriber.h"
#include "barrier.h"
#include "string_pico.h"
#include <stdint.h>

namespace RODOS {

/**
 *  @class ReliableTopic
 *  @brief Synchonos reliable link using acks
 *
 */

inline bool areMsgIdsEquivalent(const NetMsgInfo& a, const NetMsgInfo& b) {
    return a.sentTime == b.sentTime && a.senderNode == b.senderNode && a.senderThreadId == b.senderThreadId;
}

/*************************************/

template <class MsgType>
class ReliableTopic : public TopicInterface, public Subscriber {
    volatile bool ackPending = false;
    Putter*       receiver   = 0;
    Barrier       pubSubSynchonizer;
    NetMsgInfo    pendingMsgInfo; // on the sender side to identify the corresponding ack
    NetMsgInfo    lastMsgInfo;    // on the receiver side to avoid to deliver the same msg more than once

  public:
    ReliableTopic(int64_t id, const char* name, bool _onlyLocal = false) : 
        TopicInterface(id, sizeof(MsgType), name, _onlyLocal),
        Subscriber(*this, name) {}

    ~ReliableTopic() {
        if(isShuttingDown) return;
        RODOS_ERROR("ReliableTopic deleted");
    }

    //__________________________________________________________ publisher side waits until the right ack arrives

    // Yes, I know! I could call directly receiver->putGeneric, but they could/shall be in different nodes!
    inline bool publishAndWait(MsgType& msg, int64_t timeOutUntil = END_OF_TIME, bool shallSendToNetwork = true) {
        pendingMsgInfo.init(NetMsgType::P2P_RELIABLE);
        ackPending = true;
        while(NOW() < timeOutUntil) { // retry every 100 ms until timeout -> smallest time out is 100 ms (Sorry)
            TopicInterface::publishMsgPart(&msg, sizeof(MsgType), shallSendToNetwork, &pendingMsgInfo);
            if(!ackPending) break;                                                   // the ack may come while the publish is still running
            pubSubSynchonizer.waitForSignal(ackPending, NOW() + 100 * MILLISECONDS); // will be unblock when receiving ack from receiver
        }
        bool ok    = !ackPending; // else it was a timeout
        ackPending = false;
        return ok;
    }

    //__________________________________________________________ receiver side : Ack Handler & Forward msg to reciever
 
    inline void registerReceiver(Putter* rec) { receiver = rec; }

    uint32_t put([[gnu::unused]] const uint32_t topicId, [[gnu::unused]] const size_t len, void* data, const NetMsgInfo& netMsgInfo) override {

        // Receiver ack handler, unblock the publisher
        if(netMsgInfo.messageType == NetMsgType::P2P_ACK) {
            NetMsgInfo* ack = (NetMsgInfo*)data;
            if(!ackPending) return 0;

            if(areMsgIdsEquivalent(*ack, pendingMsgInfo)) {
                ackPending = false;
                pubSubSynchonizer.unblock();
                return 1;
            }
            return 0;
        }

        // forward data to receiver and send ack (Remember they may be in different nodes, else it would be simpler)
        if(netMsgInfo.messageType == NetMsgType::P2P_RELIABLE) {
            bool ok = false;
            if(!areMsgIdsEquivalent(lastMsgInfo, netMsgInfo)) { // a new message! Avoid to deliver same msg more than onece
                lastMsgInfo = netMsgInfo;
                if(receiver) ok = receiver->putGeneric(topicId, len, data, netMsgInfo);
            }
            // In any case send the ack (the msginfo of the original msg) to the corresponding netMsgInfo
            NetMsgInfo netMsgAck(NetMsgType::P2P_ACK);
            if(ok) TopicInterface::publishMsgPart((void*)&netMsgInfo, sizeof(NetMsgInfo&), true, &netMsgAck); // Send Ack
            return 1;
        }
        return 0; // message not expected (not a P2P_RELIABLE and not a P2P_ACK)
    }  // put
};

} // namespace RODOS


