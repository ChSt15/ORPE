#include "rodos.h"
#include "ringbuffer.h"

static Application applic("RinfBufTest");

RingBuffer<int32_t, 10> buf;

class Tester : public StaticThread<> {
    void writer(int numOfs) { 
            static int32_t cnt = 0;
	    for(int i = 0; i < numOfs; i++) {
		    PRINTF("W %03d\n", (int)cnt);
		    buf.put(cnt++); 
	    }
    }
    void reader(int numOfs) { 
	    int32_t readBuf;
	    for(int i = 0; i < numOfs; i++) {
	        bool ok = buf.get(readBuf);
		if (ok) PRINTF("      R %03d\n", (int)readBuf);
		else    PRINTF("      R --\n");
	    }
    }
    void run() {
	reader(2);
	writer(2);
	reader(10);
	writer(10);
	reader(5);
	writer(20);
	reader(20);
	PRINTF("____________________ Clear\n");
	buf.clear();
	reader(2);
	writer(2);
	reader(10);
	writer(10);
	reader(5);
	writer(20);
	reader(20);
    }
} tester;

