#include "rodos.h"


#include <stdint.h>

#include "gateway/router.h"
#include "gateway/gateway.h"

namespace RODOS {

class UartRouter : public Router {
public:
    UartRouter(bool forward_old_topicreports = false,
        Gateway *gateway1 = nullptr, Gateway *gateway2 = nullptr,
        Gateway *gateway3 = nullptr, Gateway *gateway4 = nullptr)
        : Router(forward_old_topicreports, gateway1, gateway2, gateway3, gateway4) {}

    void routeMsg(NetworkMessage& msg, uint32_t linkid) override;
};

}



struct OrpePoseEst {

    // Received pose data.
    int16_t px_, py_, pz_, ax_, ay_, az_;
    // Received pose frame number.
    uint32_t frameNum_;

};


extern RODOS::Topic<OrpePoseEst> orpePoseEstTopic;

extern RODOS::Topic<bool> orpePowerCommandTopic;
extern RODOS::CommBuffer<bool> orpePowerCommandBuf;

extern RODOS::Topic<float> orpeTestingTopic;