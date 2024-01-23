#include "peripheral_ctrl/aborting_error.h"
#include "hw_hal_can.h"
#include "can_ctrl.h"
#include "peripheral_ctrl/periph_map.h"
#include "peripheral_ctrl/clock_management/clock_manager.h"
#include "peripherals.h"
#include "cortex_m/nvic.h"

#include "hal/hal_can.h"
#include "rodos.h"
#include "hw_hal/hw_hal_gpio.h"



namespace RODOS {

constinit CAN_Ctrl* volatile canCtrls[mcu_specific::CANS::getNumPeriphs()] {};
constinit HW_HAL_CAN* CAN_Ctrl::filterOwners[CAN_Ctrl::numFiltersTotal] {};


CAN_Ctrl::CAN_Ctrl(CAN_IDX idx)
    : initialized(false)
    , idx(idx)
    , rxPin(GPIO_INVALID)
    , txPin(GPIO_INVALID)
    , can(mcu_specific::CANS::getPeriphStruct(idx))
    , owners{}
    , currentTxMailboxIndex(0)
    , currentFilterIndex(getFirstFilterIndex(idx))
{}

constexpr uint8_t CAN_Ctrl::getFirstFilterIndex(CAN_IDX idx){
    RODOS_ASSERT(idx * numFiltersSingle < UINT8_MAX);
    return static_cast<uint8_t>(idx * numFiltersSingle);
}

bool CAN_Ctrl::arePinsSet() const{
    return (rxPin != GPIO_INVALID) && (txPin != GPIO_INVALID);
}

void CAN_Ctrl::setPins(GPIO_PIN rxPin, GPIO_PIN txPin) {
    if (arePinsSet()){
        __builtin_trap();
    }
    this->rxPin = rxPin;
    this->txPin = txPin;
}

bool CAN_Ctrl::isUnusedMailboxAvailable() const {
    return currentTxMailboxIndex<numTxMailboxes;
}

int8_t CAN_Ctrl::addOwner(HAL_CAN* owner) {
    if (!isUnusedMailboxAvailable()){
        RODOS_ERROR("Too many HAL_CANs for the same IDX");
        return -1;
    }
    owners[currentTxMailboxIndex] = owner;
    return currentTxMailboxIndex++;
}


void CAN_Ctrl::initClocks(){
    CAN1Def::rccPeriph->enableClock();
    if constexpr(mcu_specific::CANS::getNumPeriphs() == 2) {
        if (idx == CAN_IDX1) {
            CAN2Def::rccPeriph->enableClock();
        }
    }
}

void CAN_Ctrl::configurePins(){
    configurePin(rxPin);
    configurePin(txPin);
}

void CAN_Ctrl::configureBaudrate(uint32_t baudrate){
    using namespace CAN_BTR;
    using namespace CANTimingConfig;

    if(baudrateNoPrescaler%baudrate != 0){
        PRINTF("CAN baudrate should be a divisor of %lu\n", static_cast<long unsigned>(baudrateNoPrescaler));
        abortingError("can't set requested CAN baudrate");
    }
    const uint32_t prescaler = CANTimingConfig::baudrateNoPrescaler/baudrate;

    using namespace CAN_BTR;
    using namespace CANTimingConfig;
    can.BTR.write(BRP(prescaler-1), TS1(t_BS1-1), TS2(t_BS2-1), SJW(t_RJW-1));
}

void CAN_Ctrl::enableDebugOptions(){
    using namespace CAN_BTR;
    if constexpr (debugEnableLoopback) {
        can.BTR.set(LBKM(1));
    }
}

void CAN_Ctrl::enableInterrupts(){
    using namespace CAN_IER;
    can.IER.write(FMPIE0(1), TMEIE(1));

    //enable TX+RX interrupt
    NVIC::enableIRQ(mcu_specific::CANS::getIRQn(idx).getCANx_TX());
    NVIC::enableIRQ(mcu_specific::CANS::getIRQn(idx).getCANx_RX0());
}

void CAN_Ctrl::init(uint32_t baudrate) {
    if (initialized) {
        RODOS_ERROR("Each CAN Controller should only be initialized once");
    }
    using namespace CAN_MCR;

    initialized = true;

    initClocks();
    configurePins();

    setInitMode(true);
    can.MCR.set(ABOM(1));

    configureBaudrate(baudrate);
    enableDebugOptions();

    setInitMode(false);
    can.MCR.set(SLEEP(false));
    enableInterrupts();
}


void CAN_Ctrl::setInitMode(bool enable){
    //TODO: would probably be good to have timeouts/max retries
    using namespace CAN_MCR;
    using namespace CAN_MSR;
    can.MCR.set(INRQ(enable));
    while (!(can.MSR.read<INAK>() == enable));
}


void CAN_Ctrl::configurePin(GPIO_PIN pinIdx){
    GPIO::PinGroup pinGrp = GPIO::PinGroup(pinIdx);
    if (!pinGrp.isValid()) {
        __builtin_trap();
    }
    GPIO::ALTERNATEFUNCTION af = mcu_specific::CANS::getAlternateFunction(idx, (uint8_t)pinIdx);

    pinGrp.enableClocks();
    pinGrp.setMode(GPIO::MODE::ALTERNATE);
    pinGrp.setAlternateFunction(af);
    pinGrp.setOutputType(GPIO::OUTPUTTYPE::PUSHPULL);
    pinGrp.setSpeed(GPIO::SPEED::HIGH);
}

bool CAN_Ctrl::putIntoTxMailbox(int8_t mailboxNum, const CANMsg& msg) {
    using namespace CAN_TIxR;
    using namespace CAN_TDTxR;
    RODOS_ASSERT_IFNOT_RETURN(mailboxNum >= 0, false);

    CAN_TXMailbox& mailbox = can.TX[mailboxNum];
    if (mailbox.isFull()){
        return false;
    }

    if(msg.extID){
        mailbox.TIxR.write(RTR(msg.rtr), IDE(msg.extID), EXID(msg.canID));
    }else {
        mailbox.TIxR.write(RTR(msg.rtr), IDE(msg.extID), STID(msg.canID));
    }
    mailbox.TDTxR.write(DLC(msg.len));
    mailbox.TDLxR.write(msg.data[0]);
    mailbox.TDHxR.write(msg.data[1]);
    mailbox.TIxR.set(TXRQ(1));
    return true;
}

void CAN_Ctrl::TxIRQHandler() {
    using namespace CAN_TSR;
    const RegisterCopy tsr = can.TSR;
    if (tsr.read<RQCP0>()){
        owners[0]->upCallWriteFinished();
        can.TSR.write(RQCP0(1));
    }
    if (tsr.read<RQCP1>()){
        owners[1]->upCallWriteFinished();
        can.TSR.write(RQCP1(1));
    }
    if (tsr.read<RQCP2>()){
        owners[2]->upCallWriteFinished();
        can.TSR.write(RQCP2(1));
    }
}

void CAN_Ctrl::RxIRQHandler() {
    using namespace CAN_RFxR;
    using namespace CAN_RIxR;
    using namespace CAN_RDTxR;

    const RegisterCopy RIxRCopy = can.RX[0].RIxR;
    const RegisterCopy RDTxRCopy = can.RX[0].RDTxR;
    const bool isExtID = RIxRCopy.read<IDE>();

    const CANMsg msg = {
        {can.RX[0].RDLxR.read(), can.RX[0].RDHxR.read()},
        isExtID ? RIxRCopy.read<EXID>() : RIxRCopy.read<STID>(),
        (uint8_t) RDTxRCopy.read<DLC>(),
        (bool) isExtID,
        (bool) RIxRCopy.read<RTR>()
    };

    const uint32_t filterIndex = RDTxRCopy.read<FMI>()+getFirstFilterIndex(idx);
    filterOwners[filterIndex]->addRXMsg(msg);

    can.RF0R.write(RFOM(1));
}

CAN_ERROR_TYPE getCANError(uint8_t errorCnt){
    if (errorCnt==0){
        return CAN_ERROR_TYPE::NO_ERROR;
    }else if (errorCnt<128){
        return CAN_ERROR_TYPE::ERROR_ACTIVE;
    }else{
        return CAN_ERROR_TYPE::ERROR_PASSIVE;
    }
}

CanErrorMsg CAN_Ctrl::getRxError() const{
    using namespace CAN_ESR;
    const uint8_t errorCnt = static_cast<uint8_t>(can.ESR.read<REC>());
    return {errorCnt, getCANError(errorCnt)};
}

CanErrorMsg CAN_Ctrl::getTxError() const{
    using namespace CAN_ESR;
    const RegisterCopy esr = can.ESR;
    const bool busOff = esr.read<BOFF>();
    if(busOff){
        //if the Bus-off state is reached, the error counter has overflown and a useful error count is not available
        return {0, CAN_ERROR_TYPE::BOF};
    }
    const uint8_t errorCnt = static_cast<uint8_t>(esr.read<TEC>());
    return {errorCnt, getCANError(errorCnt)};
}

CanErrorMsg CAN_Ctrl::status(CAN_STATUS_TYPE type) const {
    switch (type) {
    case CAN_STATUS_RX_ERROR:
        return getRxError();
    case CAN_STATUS_TX_ERROR:
        return getTxError();
    default:
        //there is no useful CAN_ERROR_TYPE for this situation
        return {0, CAN_ERROR_TYPE::UNKNOWN};
    }
}

bool CAN_Ctrl::isMailboxFull(int8_t mailboxNum) const {
    RODOS_ASSERT_IFNOT_RETURN(mailboxNum >= 0, true);
    return can.TX[mailboxNum].isFull();
}


int16_t CAN_Ctrl::reserveFilter(HW_HAL_CAN* const filterOwner){
    if(currentFilterIndex>=(getFirstFilterIndex(idx)+numFiltersSingle)){
        return -1;
    }
    filterOwners[currentFilterIndex] = filterOwner;
    return currentFilterIndex++;
}

void CAN_Ctrl::configureFilter(uint8_t filterIndex, uint32_t ID, uint32_t IDMask, bool extID, bool rtr){
    using namespace CAN_FMR;
    using namespace CAN_FiRx;

    CANStruct& can1 = *CAN1Def::periphStruct;
    CAN_FilterBank& filter = can1.filterBanks[filterIndex];
    can1.FMR.set(FINIT(1));
    can1.FS1R.setBits(0x0FFF'FFFFul);

    if(extID){
        filter.FxR0.write(EXID(ID), IDE(1), RTR(rtr));
        filter.FxR1.write(EXID(~IDMask), IDE(1), RTR(1));
    }else{
        filter.FxR0.write(STID(ID), IDE(0), RTR(rtr));
        filter.FxR1.write(STID(~IDMask), IDE(1), RTR(1));
    }

    can1.FA1R.setBits(1u<<filterIndex);
    can1.FMR.set(FINIT(0));
}

bool CAN_Ctrl::addIncomingFilter(uint32_t ID, uint32_t IDMask, bool extID, bool rtr, HW_HAL_CAN* filterOwner){
    const int16_t filterIndex = reserveFilter(filterOwner);
    if(filterIndex==-1){
        return false;
    }
    configureFilter(static_cast<uint8_t>(filterIndex), ID, IDMask, extID, rtr);
    return true;
}

template<CAN_IDX idx>
inline void CANx_TX_IRQHandler() {
    canCtrls[idx]->TxIRQHandler();
    NVIC::clearPendingIRQ(mcu_specific::CANS::getIRQn(idx).getCANx_TX());
}
template<CAN_IDX idx>
inline void CANx_RX0_IRQHandler() {
    canCtrls[idx]->RxIRQHandler();
    NVIC::clearPendingIRQ(mcu_specific::CANS::getIRQn(idx).getCANx_RX0());
}

extern "C" {
void CAN1_TX_IRQHandler() {
    if constexpr (mcu_specific::CANS::exists(CAN_IDX0)) {
        CANx_TX_IRQHandler<CAN_IDX0>();
    }
}
void CAN1_RX0_IRQHandler() {
    if constexpr (mcu_specific::CANS::exists(CAN_IDX0)) {
        CANx_RX0_IRQHandler<CAN_IDX0>();
    }
}
void CAN2_TX_IRQHandler() {
    if constexpr (mcu_specific::CANS::exists(CAN_IDX1)) {
        CANx_TX_IRQHandler<CAN_IDX1>();
    }
}
void CAN2_RX0_IRQHandler() {
    if constexpr (mcu_specific::CANS::exists(CAN_IDX1)) {
        CANx_RX0_IRQHandler<CAN_IDX1>();
    }
}
} // extern "C"

}
