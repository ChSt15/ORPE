
/**
 * @file linkinterfaceuart.cc
 * @author Emilio Miranda
 * @date created: 16.05.2022
 * @date modified: 01.07.2022
 * @brief Link Interface to uart.
 *
 */

#include "gateway/linkinterfaceuart.h"

namespace RODOS {

// __________________________________________ Constructor and Init
LinkinterfaceUART::LinkinterfaceUART(HAL_UART *uart, uint32_t baudRate, int64_t id, uint32_t maxRetries) : Linkinterface(id) {
    this->uart = uart;
    this->uart->init(baudRate);
    txInProg = false;
    this->maxNumOfRetriesPerWrite = maxRetries; // when to give up in case of errors
}

void LinkinterfaceUART::init() { uart->setIoEventReceiver(this); }

//_______________________________________ Sender Side

/**
 * @brief send via uart a message with the s3p frame.
 * 
 * @param outgoingMessage address of message to tx
 * @return true if all the bytes of outgoingMessage were successfully tx
 * @return false if there were more than x failed uart tries (check sendUartBuffer for x)
 */
bool LinkinterfaceUART::sendNetworkMsg(NetworkMessage &outgoingMessage)	{
    txInProg = true;
    uint8_t* outputBuffer     = reinterpret_cast<uint8_t*>(&outgoingMessage);
    int      outputBufferSize = static_cast<int>(outgoingMessage.numberOfBytesToSend());

    errFLag = false;
    sendMsg(outputBufferSize, outputBuffer); // in the S3S lib, encodes and calls putByte
    
    txInProg = false; // tx finished!
    return !errFLag;    
}

// downcall from S3P encoder
void LinkinterfaceUART::putByte(uint8_t c) { // caller will be suspended until data is sent
    
    uint32_t errCnt = 0;
    while(uart->write(&c, 1) <= 0) {            
        errCnt++;
        if(errCnt >= maxNumOfRetriesPerWrite) {
            PRINTF("ERROR: linkinterfaceuart.cpp in putByte func - uart transmit error\n");
            errFLag = true;
            return; // <<------------------
        } // max err cnt
        Thread::suspendCallerUntil(NOW() + 5*MILLISECONDS);
    } // retry loop
} // putBytes


//____________________________________________ Receiver Side

/**
 * @brief To receive a message in S3P frame from Uart
 * 
 * @param inMsg is a pointer where we store message
 * @param numberOfReceivedBytes is a pointer where we store the message length
 * @return true iff a message was receive successfully
 */
bool LinkinterfaceUART::getNetworkMsg(NetworkMessage &inMsg, int32_t &numberOfReceivedBytes) {
    uint8_t* inputBuffer = (unsigned char*)&inMsg;

    numberOfReceivedBytes = getMsg(MAX_UART_MESSAGE_LENGTH, inputBuffer); // in S3P, calls getByte

    return numberOfReceivedBytes != -1;
}

// dowcall form S3P decoder
uint8_t LinkinterfaceUART::getByte() { // caller will be suspend until data ready

    while(1) {
        if(uart->isDataReady()) {
            return static_cast<uint8_t>(uart->getcharNoWait()); //<<------------------
	}
        uart->suspendUntilDataReady(NOW() + 5*MILLISECONDS); 
    }
}


//_________________________________ Misc Function not used internaly


/**
 * @brief returns true if a message is being sent (in progress)
 */

bool LinkinterfaceUART::isNetworkMsgSent() { return !txInProg; }

void LinkinterfaceUART::suspendUntilDataReady(int64_t reactivationTime) { // NOT USED??
        uart->suspendUntilDataReady(reactivationTime);
}

} // namespace
