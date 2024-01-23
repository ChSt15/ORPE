#include "rodos.h"
#include "reqresp.h"
#include "topics.h"


class Server : public Putter, public Initiator {

    Vector3D prevValue = {0,0,0};

    void init() {
        estimator.setServer(this);
    }

    bool putGeneric([[gnu::unused]] const uint32_t topicId, [[gnu::unused]] const size_t len, const void* msg, [[gnu::unused]] const NetMsgInfo& netMsgInfo) {
        const Vector3D* input = (const Vector3D*)msg;
        Vector6D output;

        PRINTF("                                                    got Request for (%f %f %f)\n", input->x, input->y, input->z);
        AT(NOW() + 1*SECONDS);
        output.x      = input->x + 0.1;
        output.y      = input->y + 0.1;
        output.z      = input->z + 0.1;
        output.deltaX = input->x - prevValue.x;
        output.deltaY = input->y - prevValue.y;
        output.deltaZ = input->z - prevValue.z;
        prevValue = *input;
        estimator.respond(output);
        return true;
    }

} server;



