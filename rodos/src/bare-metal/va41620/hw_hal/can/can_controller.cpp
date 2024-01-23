#include "peripheral_ctrl/aborting_error.h"
#include "hw_hal_can.h"
#include "can_controller.h"
#include "peripheral_ctrl/peripheral_defs.h"
#include "cortex_m/nvic.h"

#include "hal/hal_can.h"

#include <new>

namespace RODOS {

CanController* CanController::singletons[] {};

CanController::CanController(CAN_IDX idx)
    : m_idx { idx }
    , m_can { cans[idx] }
{}

CanController& CanController::getSingleton(CAN_IDX idx){
    if(idx>=2){
        __builtin_trap();
    }
    if(singletons[idx]==nullptr){
        singletons[idx] = xmallocAndTrapOnFailure<CanController>();
        new (singletons[idx]) CanController { idx };
    }
    return *singletons[idx];
}

int8_t CanController::addOwner(HAL_CAN* owner) {
    int8_t txBufferIndex { m_usedTxBufferCounter.fetch_add(1, std::memory_order_relaxed) };
    if (txBufferIndex >= NUM_TX_BUFFERS){
        RODOS_ERROR("Too many HAL_CANs for the same IDX");
        return -1;
    }
    m_txBufferOwners[txBufferIndex] = owner;
    return txBufferIndex;
}

void CanController::configureBaudrate(uint32_t baudrate){
    using namespace CAN_CTIM;
    using namespace CANTimingConfig;

    if(baudrateNoPrescaler%baudrate != 0){
        PRINTF("CAN baudrate should be a divisor of %lu\n", static_cast<long unsigned>(baudrateNoPrescaler));
        abortingError("can't set requested CAN baudrate");
    }
    const uint32_t prescaler = baudrateNoPrescaler/baudrate;
    m_can.CTIM.write(TSEG2(t_BS2-1), TSEG1(t_BS1-1), SJW(t_RJW-1), PSC(prescaler-2));
}

void CanController::enableInterrupts(){
    using namespace CAN_CIEN;
    constexpr uint16_t ALL_BUFFER_INTERRUPTS { 0x7fff };
    m_can.CIEN.write(IEN(ALL_BUFFER_INTERRUPTS));
    m_can.CICEN.write(IEN(ALL_BUFFER_INTERRUPTS));

    NVIC::enableIRQ(getIRQn());
}

uint8_t CanController::getIRQn() const {
    //Reference: programmer's manual section 2.3
    return static_cast<uint8_t>(72 + 2*m_idx);
}

void CanController::configureRxBuffers(){
    using namespace CAN_CNSTAT;
    for(int i = 0; i<NUM_RX_BUFFERS;i++){
        m_can.buffers[i].CNSTAT.write(ST(uint32_t(CAN_ST_VALUE::RX_READY)));
    }
}

void CanController::configureTxBuffers(){
    using namespace CAN_CNSTAT;
    for(int i = FIRST_TX_BUFFER_INDEX; i<NUM_BUFFERS_TOTAL;i++){
        m_can.buffers[i].CNSTAT.write(ST(uint32_t(CAN_ST_VALUE::TX_NOT_ACTIVE)));
    }
}

void CanController::setGlobalMaskToMatchAnything(){
    constexpr uint16_t SIXTEEN_BIT_MASK_WITH_ALL_ONES { 0xffff };
    m_can.GMSKB.write(SIXTEEN_BIT_MASK_WITH_ALL_ONES);
    m_can.GMSKX.write(SIXTEEN_BIT_MASK_WITH_ALL_ONES);
}

void CanController::init(uint32_t baudrate) {
    using namespace CAN_CGCR;
    if (m_initialized.test_and_set(std::memory_order_relaxed)) {
        RODOS_ERROR("Each CAN Controller should only be initialized once");
    }

    sysconfigPeripheralBitband->CAN_ENABLE[m_idx].write(1);
    configureRxBuffers();
    configureTxBuffers();

    setGlobalMaskToMatchAnything();
    configureBaudrate(baudrate);

    m_can.CGCR.write(CANEN(1), BUFFLOCK(1), DDIR(1));
    enableInterrupts();
}


bool CanController::putIntoTxBuffer(int8_t txBufferIdx, const CanMessage& msg) {
    using namespace CAN_CNSTAT;
    if(!isTxBufferEmpty(txBufferIdx)){
        return false;
    }
    CANHardwareBuffer& buffer { m_can.buffers[FIRST_TX_BUFFER_INDEX+txBufferIdx] };
    buffer.DATA0.write(msg.data[3]);
    buffer.DATA1.write(msg.data[2]);
    buffer.DATA2.write(msg.data[1]);
    buffer.DATA3.write(msg.data[0]);

    buffer.ID0.write(msg.metadata.getId0Value());
    buffer.ID1.write(msg.metadata.getId1Value());
    //TODO: it would be good to set the PRI register field to prioritise outgoing messages with a low ID
    buffer.CNSTAT.write(
        ST(uint32_t(CAN_ST_VALUE::TX_ONCE)),
        DLC(msg.length)
    );
    return true;
}

bool CanController::isTxBufferEmpty(int8_t bufferIndex) const{
    using namespace CAN_CNSTAT;
    return m_can.buffers[FIRST_TX_BUFFER_INDEX+bufferIndex].CNSTAT.read<ST>() == uint32_t(CAN_ST_VALUE::TX_NOT_ACTIVE);
}



void CanController::handleInterruptFromRxBuffer(int8_t bufferId) {
    using namespace CAN_CNSTAT;
    CANHardwareBuffer& buffer = m_can.buffers[bufferId];
    VA41620CanFrameMetadata incomingFrameMetadata {
        buffer.ID0.read(),
        buffer.ID1.read()
    };
    int8_t activeFilters { m_usedFilterCounter.load(std::memory_order_relaxed) };
    for(int8_t i = 0; i<activeFilters; i++){
        if(m_softwareFilters[i].matches(incomingFrameMetadata)){
            const CanMessage msg {
                .data = {
                    static_cast<uint16_t>(buffer.DATA3.read()),
                    static_cast<uint16_t>(buffer.DATA2.read()),
                    static_cast<uint16_t>(buffer.DATA1.read()),
                    static_cast<uint16_t>(buffer.DATA0.read()),
                },
                .metadata = incomingFrameMetadata,
                .length = static_cast<uint8_t>(buffer.CNSTAT.read<DLC>())
            };
            m_softwareFilters[i].getOwner()->addRxFrame(msg);
            break;
        }
    }
    m_can.CICLR.write(uint32_t(1<<bufferId));
    buffer.CNSTAT.write(ST(uint32_t(CAN_ST_VALUE::RX_READY)));
}

CAN_ERROR_TYPE CanController::getErrorType(){
    using namespace CAN_CSTPND;
    uint32_t nsValue { m_can.CSTPND.read<NS>() };
    switch(nsValue){
    case 0b010: [[fallthrough]];
    case 0b011:
        return CAN_ERROR_TYPE::ERROR_ACTIVE;
    case 0b100: [[fallthrough]];
    case 0b101:
        return CAN_ERROR_TYPE::ERROR_PASSIVE;
    case 0b110: [[fallthrough]];
    case 0b111:
        return CAN_ERROR_TYPE::BOF;
    default:
        return CAN_ERROR_TYPE::UNKNOWN;
    }
}

uint8_t CanController::getRxErrorCounter() const {
    using namespace CAN_CANEC;
    return static_cast<uint8_t>(m_can.CANEC.read<REC>());
}

uint8_t CanController::getTxErrorCounter() const {
    using namespace CAN_CANEC;
    return static_cast<uint8_t>(m_can.CANEC.read<TEC>());
}

void CanController::handleInterruptFromTxBuffer(int8_t bufferId) {
    HAL_CAN& bufferOwner { *m_txBufferOwners[(bufferId-FIRST_TX_BUFFER_INDEX)] };
    bufferOwner.upCallWriteFinished();
    m_can.CICLR.write(uint32_t(1<<bufferId));
}

void CanController::irqHandler() {
    using namespace CAN_CSTPND;
    uint32_t istValue { 0 };
    while((istValue = m_can.CSTPND.read<IST>()) != 0){
        const int8_t bufferIndex { static_cast<int8_t>(istValue-1) };
        if(bufferIndex<FIRST_TX_BUFFER_INDEX){
            handleInterruptFromRxBuffer(bufferIndex);
        }else{
            handleInterruptFromTxBuffer(bufferIndex);
        }
    }
}

void CanController::callIrqHandler(CAN_IDX idx){
    singletons[idx]->irqHandler();
}

bool CanController::addIncomingFilter(uint32_t id, uint32_t idMask, bool ide, bool rtr, HW_HAL_CAN* filterOwner){
    const int8_t filterIndex { m_usedFilterCounter.fetch_add(1, std::memory_order_relaxed) };
    if (filterIndex >= (NUM_SOFTWARE_FILTERS)){
        RODOS_ERROR("Too many CAN filters");
        return false;
    }

    m_softwareFilters[filterIndex] = {id, idMask, ide, rtr, filterOwner};
    return true;
}


extern "C" {
void CAN0_IRQHandler() {
    CanController::callIrqHandler(CAN_IDX0);
}
void CAN1_IRQHandler() {
    CanController::callIrqHandler(CAN_IDX1);
}
} // extern "C"

}
