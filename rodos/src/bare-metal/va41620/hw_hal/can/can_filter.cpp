#include "can_filter.h"

namespace RODOS {

CanFilter::CanFilter(uint32_t id, uint32_t idMask, bool ide, bool rtr, HW_HAL_CAN* owner)
: m_mask { VA41620CanFrameMetadata::generateForMask(idMask, ide) }
, m_value { id, ide, rtr }
, m_owner { owner }
{}

bool CanFilter::matches(const VA41620CanFrameMetadata& data) const {
    uint32_t mismatchingBits = data.getRawValue()^m_value.getRawValue();
    return (mismatchingBits & ~m_mask.getRawValue())==0;
}

HW_HAL_CAN* CanFilter::getOwner() const {
    return m_owner;
}

}
