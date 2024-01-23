#pragma once
#include "bit_field.h"

namespace RODOS {


/**
 * @brief stores the CAN ID and the IDE and RTR flags in the same format as
 * the Registers in the CAN Controller of the VA41620
 */
class VA41620CanFrameMetadata {
public:
    using XRTR = UInt32SubValue<0>;
    using ExtendedId0To17 = UInt32SubValue<1, 18>;
    using IDE = UInt32SubValue<19>;
    using RTR = UInt32SubValue<20>;
    using SRR = UInt32SubValue<20>;
    using ExtendedId18To28 = UInt32SubValue<21, 11>;
    using BasicId = ExtendedId18To28;

    constexpr VA41620CanFrameMetadata(uint32_t id, bool ide, bool rtr)
    : m_rawData {
            XRTR((ide&&rtr)?1:0) |
            ExtendedId0To17(ide?(id&0x3ffff):0) |
            IDE(ide?1:0) |
            RTR((ide||rtr)?1:0) |
            ExtendedId18To28(ide?(id>>18):id)
        }
    {}

    constexpr VA41620CanFrameMetadata(uint32_t Id0RegisterValue, uint32_t Id1RegisterValue)
    : m_rawData { Id0RegisterValue | (Id1RegisterValue<<16) }
    {}

    constexpr VA41620CanFrameMetadata() = default;

    /**
     * works like the regular constructor, but sets unused bits to 1(don't-care)
     * also always sets the bits relevant to ide and rtr to 0 because the RODOS api can't set them to don't-care
     */
    static constexpr VA41620CanFrameMetadata generateForMask(uint32_t idMask, bool ide){
        return {
            XRTR(ide?0:1) |
            ExtendedId0To17(ide?(idMask&0x3ffff):0x3ffff) |
            IDE(0) |
            RTR(0) |
            ExtendedId18To28(ide?(idMask>>18):idMask)
        };
    }

    constexpr uint32_t getId() const {
        if(getIde()){
            return m_rawData.read<ExtendedId18To28>()<<18 | m_rawData.read<ExtendedId0To17>();
        }else{
            return m_rawData.read<BasicId>();
        }
    }

    constexpr bool getIde() const {
        return m_rawData.read<IDE>();
    }

    constexpr bool getRTR() const {
        return getIde() ? m_rawData.read<XRTR>() : m_rawData.read<RTR>();
    }

    constexpr uint16_t getId1Value() const {
        return static_cast<uint16_t>(m_rawData.read()>>16);
    }

    constexpr uint16_t getId0Value() const {
        return static_cast<uint16_t>(m_rawData.read());
    }

    constexpr uint32_t getRawValue() const {
        return m_rawData.read();
    }
private:
    UInt32BitField m_rawData;

    constexpr VA41620CanFrameMetadata(uint32_t rawValue)
    : m_rawData { rawValue }
    {}
};

}
