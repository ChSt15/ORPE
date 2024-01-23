
#pragma once


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif


/**
     RingBuffer administrates a pool of objects in a ring manner (Template class).

     With put you can rewrite the oldest entry in the buffer.
     With get you get cyclic one for one, all written entries
     in the buffer. If there was no put before (see writeCnt) 
     get returns false and do not modify its parameter

     Thread-safe for one writer and one reader.

     @param      t Class to be administrated (Template)
     @param      poolSize how many elements are to be allocated

     @author     Sergio Montenegro
     @version    Sept. 1999, March 2000, Feb 2009


*/

template <class T, uint32_t poolSize>
class RingBuffer {
public:
    uint32_t   writeIndex = 0;
    uint32_t   readIndex = 0;
    uint32_t   currentWrite = UINT32_MAX;
    T          vals[poolSize];

    /// How often it was written
    uint64_t writeCnt = 0;
    /// How often it was readed
    uint64_t readCnt = 0;
    /// How many objects are in FIFO ( == writeCnt - readCnt)
    uint32_t occupiedCnt = 0;

    T* getNextEntryToPut() { return &vals[writeIndex]; }

    void clear() {
        writeIndex = 0;
        readIndex = 0;
        currentWrite = UINT32_MAX;
        writeCnt = 0;
        readCnt = 0;
        occupiedCnt = 0;
    }

    void put(const T& newdata) {
        currentWrite = writeIndex;
        vals[writeIndex] = newdata;
        writeIndex++;
        if (writeIndex  >= poolSize) writeIndex = 0;
        if (occupiedCnt <  poolSize) occupiedCnt++;
        writeCnt++;
        currentWrite = UINT32_MAX;
    }

    /// get the next
    bool get(T& fromRing) {
        if(writeCnt == 0) return false;

        /** Jump the current being written record **/
        if (readIndex == currentWrite) readIndex++;
        if (readIndex >= poolSize) readIndex = 0;

        /** readIndex should not pass write index if there is no data after it */
        if (writeCnt < poolSize && readIndex >= writeIndex) readIndex = 0;

        fromRing = vals[readIndex];
        readIndex++;
        if (readIndex >= poolSize) readIndex = 0;

        if (occupiedCnt > 0) occupiedCnt--;
        readCnt++;

	return true;
    }

    ///
    int getLen() { return poolSize; }

};



#ifndef NO_RODOS_NAMESPACE
}
#endif

