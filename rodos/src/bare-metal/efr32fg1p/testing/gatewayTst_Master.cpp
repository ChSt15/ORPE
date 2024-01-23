#include "rodos.h"
#include "hal/hal_radio_mstrslv.h"
#include "gateway.h"
#include "gateway/linkinterfaceradio.h"
#include "platform-parameter.h"

static HAL_RADIO_MASTER   master;
static LinkinterfaceRADIO linkinterfaceRADIO(&master, NULL, true, -5001);
static Gateway            gateway1(&linkinterfaceRADIO, true);

Topic<int32_t> myTopic(1337, "MyTopic");


class MasterTst : public Thread {   // executer of the state machine
  public:
    MasterTst() : Thread("MasterTst",100,1000) {}

    void run() {
        master.init();
        master.config(POLL_CHANNEL, 2);
        master.config(DATA_CHANNEL, 3);

        master.executeState();
        TIME_LOOP(5000 * MICROSECONDS, 100 * MICROSECONDS) {
            if(master.switchState() == 1) master.executeState();
        }
    }
} masterTst;

//__________________________________________________________

class SimpleSub : public Subscriber {
  public:
    SimpleSub() : Subscriber(myTopic, "simplesub") {}
    long put(const long topicId, const long len, const void* data, const NetMsgInfo& netMsgInfo) {
        PRINTF("SimpleSub - Length: %ld Data: %ld TopicId: %ld \n", len, *(int32_t*)data, topicId);
        return 1;
    }
} simpleSub;

//__________________________________________________________

HAL_GPIO         led(GPIO_024);
static class Update : public Thread {  // just print status
  public:
    Update() : Thread("Update",100,1000) {}

    void run() {
        int led_On = 0;

        led.config(GPIO_CFG_OUTPUT_ENABLE, 1);

        TIME_LOOP(5 * SECONDS, 10 * SECONDS) {
            PRINTF("Time: %.1f   ", SECONDS_NOW());
            PRINTF("Rx Counter: %d   Tx Counter: %d\n", master.rxTimestampCounter, master.txTimestampCounter);
            led.setPins(led_On);
            led_On = ~led_On;
        }
    }
} update;
