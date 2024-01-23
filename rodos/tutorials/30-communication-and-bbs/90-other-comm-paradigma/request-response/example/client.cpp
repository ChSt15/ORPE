#include "rodos.h"
#include "reqresp.h"
#include "topics.h"

//____________________________________________________________


class Client : public StaticThread<> {
    Vector3D sensor = {10,20,30};
    Vector6D state;

    void run() {
        PRINTF("\n\n\n\n");
        while(1) {
            sensor.x = sensor.x * 1.01;
            sensor.y = sensor.y + 2.25;
            sensor.z = sensor.z + 3.333333 + sensor.x; 
            
            PRINTF("  ____________________________________\n");
            PRINTF("sending Request for (%f %f %f)\n", sensor.x, sensor.y, sensor.z);
            bool ok = estimator.requestAndWait(sensor, state, NOW() + 5*SECONDS); 
            if(ok) {
                PRINTF("state (%f %f %f  ... %f %f %f)\n", 
                    state.x, state.y, state.z,
                    state.deltaX, state.deltaY, state.deltaZ);
            } else {
                PRINTF(" tiemeout\n");
           }
        }
    }

  public:
    Client() : StaticThread<>() {}
} client;




