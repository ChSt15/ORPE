#include "rodos.h"
#include "hal/hal_radio_mstrslv.h"
#include "gateway.h"
#include "gateway/linkinterfaceradio.h"
#include "topic.h"
#include "platform-parameter.h"

Topic<int32_t> myTopic(1337, "MyTopic");

static HAL_RADIO_SLAVE    slave;
static LinkinterfaceRADIO linkinterfaceRADIO(NULL, &slave, false, -5001);
static Gateway            gateway1(&linkinterfaceRADIO, true);

int id = 4;

class SlaveTst : public Thread {  // to execute the state machine
  public:
    SlaveTst() : Thread("slaveThread",100,1000) {}

    void run() {
        slave.config(PACKET_LENGTH, 300);
        slave.init();
        slave.config(POLL_CHANNEL, 2);
        slave.config(DATA_CHANNEL, 3);
        slave.config(SLAVE_ID, id); // default Id = 0

        slave.executeState();
        TIME_LOOP(5000 * MICROSECONDS, 100 * MICROSECONDS) {
            if(slave.switchState() == 1) slave.executeState();
        }
    }
} slaveTst;

//_________________________________________________________
class MyPublisher01 : public Thread {

  public:
    MyPublisher01() : Thread("SenderSimple",100,1000) {}

    void run() {
        int32_t value = 200;
        TIME_LOOP(3 * SECONDS, 500 * MILLISECONDS) {
            myTopic.publish(value);
            value++;
        }
    }
} publishier01;

//_________________________________________________________
HAL_GPIO led(GPIO_024);
static class Update : public Thread { // Just to print status
  public:
    Update() : Thread("Update",100,1000) {}

    void run() {
        int led_On = 0;

        led.config(GPIO_CFG_OUTPUT_ENABLE, 1);

        TIME_LOOP(5 * SECONDS, 10 * SECONDS) {
            PRINTF("Time: %.1f   ", SECONDS_NOW());
            PRINTF("Rx Counter: %d   Tx Counter: %d\n", slave.rxTimestampCounter, slave.txTimestampCounter);
            led.setPins(led_On);
            led_On = ~led_On;
        }
    }
} update;
