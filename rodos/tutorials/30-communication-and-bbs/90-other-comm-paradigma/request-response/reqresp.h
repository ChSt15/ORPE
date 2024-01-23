

/**
 * @file reqresp.h
 * @date 2022/05/14 
 * @author Sergio Montenegro
 *
 *
 * @brief Request (client) / response (Server) topic
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
 *  @class ReqResp (Request Response)
 *  @brief Synchonos remote procedure call using a common topic
 *
 */

/*************************************/

template <class ReqType, class RespType>
class ReqResp : public TopicInterface, public Subscriber {
    Putter*       server          = 0;
    volatile bool responsePending = false;
    Barrier       clientServerSynchonizer;
    RespType*     respBuffer;

  public:
    /** constructor, parameter name is used for debug purposes and as hash value
      * to generate a topic id if it was defined as -1. This is the proposed method.
      */
    ReqResp(int64_t id, const char* name, bool _onlyLocal = false) : 
        TopicInterface(id, max(sizeof(ReqType), sizeof(RespType)), name, _onlyLocal),
        Subscriber(*this, name) {}

    ~ReqResp() {
        if(isShuttingDown) return;
        RODOS_ERROR("topic deleted");
    }

    inline void setServer(Putter* serv) { server = serv; }

    //__________________________________________________________ Sender side Client and server
    /** 
     * Cleint side, client request
     * See Topic::publish. Almos the same but:
     * distribute the request message 
     * Suspends until the response comes or timeout
     *  Note: Yes, I know! I could call directly the own putter instead of publishing the message. 
     *        this would work, but only if client and server are in the same node
     */
    inline bool requestAndWait(ReqType& reqMsg, RespType& respMsg, int64_t timeOutUntil = END_OF_TIME, bool shallSendToNetwork = true) {
        NetMsgInfo netMsgInfo(NetMsgType::REQUEST);
        respBuffer      = &respMsg;
        responsePending = true;
        TopicInterface::publishMsgPart(&reqMsg, sizeof(ReqType), shallSendToNetwork, &netMsgInfo);
        if(responsePending) clientServerSynchonizer.waitForSignal(responsePending, timeOutUntil);
        bool ok         = !responsePending;
        responsePending = false;
        return ok;
    }


    /** 
     * server side, server response
     * See Topic::publish. Almos the same but:
     * distribute the response  message 
     */
    inline uint32_t respond(RespType& respMsg, bool shallSendToNetwork = true) {
        NetMsgInfo netMsgInfo(NetMsgType::RESPONSE);
        return TopicInterface::publishMsgPart(&respMsg, sizeof(RespType), shallSendToNetwork, &netMsgInfo);
    }

    //__________________________________________________________ receiver side client and server


    uint32_t put([[gnu::unused]] const uint32_t topicId, [[gnu::unused]] const size_t len, void* data, const NetMsgInfo& netMsgInfo) override {

        // Receiver for the client: Get response
        if(netMsgInfo.messageType == NetMsgType::RESPONSE) {
            if(!responsePending) return 0;

            if(respBuffer) memcpy(respBuffer, data, min(len, sizeof(RespType)));
            respBuffer      = 0;
            responsePending = false;
            clientServerSynchonizer.unblock();
            return 1;
        } // handle responses


        // Receiver for the server: Get request
        if(netMsgInfo.messageType == NetMsgType::REQUEST) {
            // The server shall answer with publishResponse
            if(server) server->putGeneric(topicId, len, data, netMsgInfo);
            return 1;
        } // Handle request

        return 0; // message not expected (not a REQUEST and not a RESPONSE)
    }             // put
};

} // namespace RODOS
