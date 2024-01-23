#include "rodos.h"
#include "topics.h"


class Subscriver : public Putter, public Initiator {

    void init() {
        position.registerReceiver(this);
    }

    bool putGeneric([[gnu::unused]] const uint32_t topicId, [[gnu::unused]] const size_t len, const void* msg, [[gnu::unused]] const NetMsgInfo& netMsgInfo) {
        const Vector3D* input = (const Vector3D*)msg;

        PRINTF("                                       got (%f %f %f)\n", input->x, input->y, input->z);
        return true;
    }

} server;
