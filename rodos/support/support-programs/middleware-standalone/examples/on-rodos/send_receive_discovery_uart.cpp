#include "rodos.h"
#include "gateway.h"

#include "../topics.h"

Topic<Position> posTopic(topicIdLinux2Rodos,  "posTopic"); // to receive (subscrived)
Topic<MyTime>   timeTopic(topicIdRodos2Linux, "timeTopic"); // to send (pulish)

/******************************/

static HAL_UART uart(UART_IDX2); // USB-UART      Tx:= PD5 , Rx:= PD6
static LinkinterfaceUART linkif(&uart, 115200, 3, 10);
static Gateway gw(&linkif, true);

/******************************/

class MyPublisher : public StaticThread<> {
  MyTime myTime;
public:
  MyPublisher() : StaticThread("sender from RODOS") {}
  void init() { uart.init(115200); }
  void run() {
    int32_t cnt = 10000;
    TIME_LOOP(0, 1600 * MILLISECONDS) {
      PRINTF("sending %d\n", (int)cnt++);
      myTime.msgIndex = cnt;
      myTime.timeNow  = NOW();
      timeTopic.publish(myTime);
    }
  }
} myPublisher;

struct PosReceiver : public Subscriber {
  PosReceiver() : Subscriber(posTopic, "pos Pinter") {}
  uint32_t put(const uint32_t topicId, const size_t len, void *msg, const NetMsgInfo &) override {
    Position* pos = (Position*)msg;
    PRINTF("Got topicId=%d len =%d : ", (int)topicId, (int)len);
    PRINTF(" Name: %s, cnt %d, coordinates (%f,%f,%f)\n", pos->name, (int)(pos->cnt), pos->x, pos->y, pos->z);
    return true;
  }
} posReceiver;
