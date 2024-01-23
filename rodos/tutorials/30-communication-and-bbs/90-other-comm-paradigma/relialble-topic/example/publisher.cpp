#include "rodos.h"
#include "topics.h"
#include "reliable-topic.h"

//____________________________________________________________


class Publisher : public StaticThread<> {
    Vector3D sensor = { 10, 20, 30 };

    void run() {
        PRINTF("\n\n\n\n");
        while(1) {
            sensor.x = sensor.x * 1.00001;
            sensor.y = sensor.y + 2.25;
            sensor.z = sensor.z + 3.333333;

            PRINTF("sending (%f %f %f)\n", sensor.x, sensor.y, sensor.z);
            bool ok = position.publishAndWait(sensor, NOW() + 3*SECONDS);
            PRINTF("ok = %s\n", ok ? "ok" : "timeout");
        }
    }

  public:
} publisher;
