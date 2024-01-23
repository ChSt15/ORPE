#include "hal/genericIO.h"
#include "misc-rodos-funcs.h"

namespace RODOS {


GenericIOInterface::GenericIOInterface() {
    ioEventReceiver=0;
    callerReadFinished = 0;
    callerWriteFinished = 0;
    callerDataReady = 0;
} // create all drivers static but do not init there


void GenericIOInterface::suspendUntilReadFinished([[maybe_unused]] int64_t reactivationTime) {
    return;
}


void GenericIOInterface::suspendUntilWriteFinished([[maybe_unused]] int64_t reactivationTime) {
    return;
}


void GenericIOInterface::suspendUntilDataReady([[maybe_unused]] int64_t reactivationTime) {
    return;
}


void GenericIOInterface::upCallWriteFinished() { // last transmit order is concluded
    if(ioEventReceiver) ioEventReceiver->onWriteFinished();
}


void GenericIOInterface::upCallReadFinished() { // last receive order is concluded
    if(ioEventReceiver) ioEventReceiver->onReadFinished();
}


void GenericIOInterface::upCallDataReady() {  // data has arrived
    if(ioEventReceiver) ioEventReceiver->onDataReady();
}


void GenericIOInterface::setIoEventReceiver(IOEventReceiver* receiver){
    ioEventReceiver=receiver;
}


}

