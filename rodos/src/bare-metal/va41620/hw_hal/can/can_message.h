#pragma once
#include "can_frame_metadata.h"


namespace RODOS {

struct CanMessage {
    uint16_t data[4];
    VA41620CanFrameMetadata metadata;
    uint8_t length; ///< length of can message in bytes
};

}
