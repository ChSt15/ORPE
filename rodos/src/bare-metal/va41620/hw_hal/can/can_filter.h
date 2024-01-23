#pragma once
#include "can_frame_metadata.h"

namespace RODOS {

class HW_HAL_CAN;

class CanFilter {
public:
    CanFilter(uint32_t id, uint32_t idMask, bool ide, bool rtr, HW_HAL_CAN* owner);
    CanFilter() = default;

    bool matches(const VA41620CanFrameMetadata& data) const;

    HW_HAL_CAN* getOwner() const;
private:
    VA41620CanFrameMetadata m_mask;
    VA41620CanFrameMetadata m_value;
    HW_HAL_CAN* m_owner { nullptr };
};

}
