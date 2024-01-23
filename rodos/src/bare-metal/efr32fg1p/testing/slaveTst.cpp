#include "rodos.h"
#include "hal/hal_radio_mstrslv.h"
#include "platform-parameter.h"


static HAL_RADIO_SLAVE    slave;
char            example_msg[1600] = { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 };

int id = 4;

class SlaveTst : public Thread {  // to execute the state machine
  public:
    SlaveTst() : Thread("slaveThread") {}

    void run() {
        slave.config(PACKET_LENGTH, 1536);
        slave.init();
        slave.config(POLL_CHANNEL, 2);
        slave.config(DATA_CHANNEL, 3);
        slave.config(SLAVE_ID, id); // default Id = 0

        slave.executeState();
        TIME_LOOP(1 * MILLISECONDS, 100 * MICROSECONDS) {
            if(slave.switchState() == 1) slave.executeState();
        }
    }

    void init() { PRINTF("Id %d", id); }
} slaveTst;

static class MessageTst : public Thread {
  public:
    MessageTst() : Thread("sendMsg") {}

    void run() {
        TIME_LOOP(5 * MILLISECONDS, 50 * MILLISECONDS) {
            slave.write(example_msg, 1500);
        }
    }

    void init() { PRINTF("Write"); }

} messageTst;

//_________________________________________________________
HAL_GPIO led(GPIO_024);
static class Update : public Thread { // Just to print status
  public:
    Update() : Thread("Update") {}

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
