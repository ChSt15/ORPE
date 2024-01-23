/**
 * @file linkinterfaceuart.h
 * @author Emilio Miranda
 * @date created: 16.06.2022
 * @date modified: 01.07.2022
 * @brief Implementation of linkinterface and network message with s3p frame
 * 
 */

#pragma once

#include "gateway/linkinterface.h"
#include "hal/hal_uart.h"
#include "s3p-synchronous-interface.h"


namespace RODOS {

class LinkinterfaceUART : public Linkinterface, IOEventReceiver, S3pReceiverSynchronous, S3pSenderSynchronous {
private:
    const int MAX_UART_MESSAGE_LENGTH = MAX_NETWORK_MESSAGE_LENGTH;
    
    HAL_UART *uart;

    uint32_t maxNumOfRetriesPerWrite;  // transmition abort in case of too many errors
    bool errFLag;               // Tx max attempts reached while sending?
    volatile bool txInProg;     // True if we are sending a message, otherwise False 

    // ___________________________   functions called from s3p encoder and decoder to send and receive
    void putByte(uint8_t c);
    uint8_t getByte();

public:
    LinkinterfaceUART(HAL_UART *uart, uint32_t baudRate, int64_t id = -1, uint32_t maxRetries = 1000);
    void init(); 

    bool sendNetworkMsg(NetworkMessage& outgoingMessage);
    bool getNetworkMsg(NetworkMessage &inMsg, int32_t &numberOfReceivedBytess);
    bool isNetworkMsgSent();
    virtual void suspendUntilDataReady(int64_t reactivationTime = END_OF_TIME);
};
}  // namespace
