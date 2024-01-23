#include "hw_hal_pwm.h"

#include "peripheral_ctrl/clock_management/clock_manager.h"

namespace RODOS {

HW_HAL_PWM::HW_HAL_PWM(PWM_IDX idx)
    : m_selectedPwmIdx(idx)
    , m_timerId(static_cast<TimerId>(static_cast<uint8_t>(idx) / 4))
    , m_timerChannelId(static_cast<TimerChannelId>(static_cast<uint8_t>(idx % 4)))
    , m_timer(mcu_specific::TIMS::getPeriphStruct(static_cast<uint8_t>(m_timerId)))
    { }

int32_t HW_HAL_PWM::init(uint32_t frequency, uint32_t increments) {

    if (m_timerId != TimerId::TIM_2 && increments > UINT16_MAX){
        RODOS_ERROR("Selected timer is a 16 bit timer, it cannot hold the given number of increments");
        return ERROR;
    }

    mcu_specific::TIMS::getRCCPeriph(static_cast<uint8_t>(m_timerId)).enableClock();

    const GPIO_PIN pinId = getAssociatedGPIOId(m_selectedPwmIdx);

    if (configurePinForPWMOutput(pinId) == ERROR) {
        return ERROR;
    }

    m_currentFrequency = frequency;
    m_currentIncrements = increments;

    if (setTimerFrequencyAndIncrements() == ERROR) {
        return ERROR;
    }

    using namespace TIMGeneralPurpose_CR1;
    using namespace TIMGeneralPurpose_CCMR1;
    using namespace TIMGeneralPurpose_CCMR2;
    using namespace TIMGeneralPurpose_EGR;
    using namespace TIMGeneralPurpose_CCER;
    using namespace TIMGeneralPurpose_TIM1_BDTR;

    constexpr uint8_t DISABLE_UPDATE_EVENTS = 1;
    m_timer.CR1.set(UIDS(DISABLE_UPDATE_EVENTS));

    if (m_timerId == TimerId::TIM_1){
        m_timer.TIM1_BDTR.write(MOE(1));
    }

    constexpr uint8_t ENABLE_PRELOAD = 1;
    switch (m_timerChannelId) {
        case TimerChannelId::CH1:
            m_timer.CCMR1.set(OC1PE(ENABLE_PRELOAD));
            m_timer.CCMR1.set(OC1M_Bit3(0b0), OC1M_Bits0To2(0b110)); // Set PWM mode 1

            m_timer.CCR1.write(0ul);
            break;
        case TimerChannelId::CH2:
            m_timer.CCMR1.set(OC2PE(ENABLE_PRELOAD));
            m_timer.CCMR1.set(OC2M_Bit3(0b0), OC2M_Bits0To2(0b110)); // Set PWM mode 1

            m_timer.CCR2.write(0ul);
            break;
        case TimerChannelId::CH3:
            m_timer.CCMR2.set(OC3PE(ENABLE_PRELOAD));
            m_timer.CCMR2.set(OC3M_Bit3(0b0), OC3M_Bits0To2(0b110)); // Set PWM mode 1

            m_timer.CCR3.write(0ul);
            break;
        case TimerChannelId::CH4:
            m_timer.CCMR2.set(OC4PE(ENABLE_PRELOAD));
            m_timer.CCMR2.set(OC4M_Bit3(0b0), OC4M_Bits0To2(0b110)); // Set PWM mode 1

            m_timer.CCR4.write(0ul);
            break;
    }

    constexpr uint8_t ENABLE_UPDATE_EVENTS = 0;
    m_timer.CR1.set(UIDS(ENABLE_UPDATE_EVENTS));

    constexpr uint8_t RELOAD_PRELOADED_REGISTERS = 1;
    m_timer.EGR.set(UG(RELOAD_PRELOADED_REGISTERS));

    constexpr uint8_t ENABLE_TIMER_OUTPUT = 1;
    switch(m_timerChannelId) {
        case TimerChannelId::CH1:
            m_timer.CCER.set(CC1E(ENABLE_TIMER_OUTPUT));
            break;
        case TimerChannelId::CH2:
            m_timer.CCER.set(CC2E(ENABLE_TIMER_OUTPUT));
            break;
        case TimerChannelId::CH3:
            m_timer.CCER.set(CC3E(ENABLE_TIMER_OUTPUT));
            break;
        case TimerChannelId::CH4:
            m_timer.CCER.set(CC4E(ENABLE_TIMER_OUTPUT));
            break;
    }

    constexpr uint8_t TIMER_START_COUNTING = 1;
    m_timer.CR1.set(CEN(TIMER_START_COUNTING));

    return SUCCESS;
}

int32_t HW_HAL_PWM::config(PWM_PARAMETER_TYPE type, int32_t paramVal){

    switch (type){
        case PWM_PARAMETER_TYPE::PWM_PARAMETER_INCREMENTS:
            m_currentIncrements = static_cast<uint32_t>(paramVal);
            break;
        case PWM_PARAMETER_TYPE::PWM_PARAMETER_FREQUENCY:
            m_currentFrequency = static_cast<uint32_t>(paramVal);
            break;
    }

    return setTimerFrequencyAndIncrements();
}

int32_t HW_HAL_PWM::setTimerFrequencyAndIncrements() {
    const uint32_t timerFrequency = getTimerFrequency();
    if (m_currentFrequency > timerFrequency) {
        RODOS_ERROR("The requested frequency is too high for this timer");
        return ERROR;
    }

    const uint32_t maxIncrements = timerFrequency / m_currentFrequency;
    if (m_currentIncrements > maxIncrements) {
        RODOS_ERROR("The number of increments requested is too high for the requested frequency");
        return ERROR;
    }

    const uint32_t timerFrequencyNeededForPWM = m_currentFrequency * m_currentIncrements;
    if (timerFrequencyNeededForPWM > timerFrequency) {
        RODOS_ERROR("The frequency needed to accomodate the requested number of increments is too high for this timer");
        return ERROR;
    }

    // Given:
    //   - PSC the value of the prescaler of the timer
    //   - f_d the desired frequency
    //   - f_t the original timer frequency
    //
    // We have that: f_d = f_t / (PSC + 1)

    const uint16_t prescalerValue = static_cast<uint16_t>((timerFrequency / timerFrequencyNeededForPWM) - 1);
    m_timer.PSC.write16(prescalerValue);

    using namespace TIMGeneralPurpose_CR1;

    constexpr uint8_t DISABLE_UPDATE_EVENTS = 1;
    m_timer.CR1.set(UIDS(DISABLE_UPDATE_EVENTS));

    m_timer.ARR.write(m_currentIncrements);

    constexpr uint8_t ENABLE_UPDATE_EVENTS = 0;
    m_timer.CR1.set(UIDS(ENABLE_UPDATE_EVENTS));

    return SUCCESS;
}

int32_t HW_HAL_PWM::write(const uint32_t pulseWidthInIncs) {

    switch (m_timerChannelId) {
        case TimerChannelId::CH1:
            m_timer.CCR1.write(pulseWidthInIncs);
            break;
        case TimerChannelId::CH2:
            m_timer.CCR2.write(pulseWidthInIncs);
            break;
        case TimerChannelId::CH3:
            m_timer.CCR4.write(pulseWidthInIncs);
            break;
        case TimerChannelId::CH4:
            m_timer.CCR4.write(pulseWidthInIncs);
            break;
    }

    return SUCCESS;
}

void HW_HAL_PWM::reset(){
    mcu_specific::TIMS::getRCCPeriph(static_cast<uint8_t>(m_timerId)).enableClock();

    const GPIO_PIN pinId = getAssociatedGPIOId(m_selectedPwmIdx);
    GPIO::PinGroup pin = GPIO::PinGroup(pinId);

    if(!pin.isValid()) {
        RODOS_ERROR("Cannot get GPIO pin with requested id");
        return;
    }

    pin.reset();
}

int HW_HAL_PWM::configurePinForPWMOutput(GPIO_PIN pinId){
    GPIO::PinGroup pin = GPIO::PinGroup(pinId);

    if(!pin.isValid()) {
        RODOS_ERROR("Cannot get GPIO pin with requested id");
        return ERROR;
    }

    pin.enableClocks();
    pin.setPull(GPIO::PULL::NONE);
    pin.setOutputType(GPIO::OUTPUTTYPE::PUSHPULL);
    pin.setAlternateFunction(mcu_specific::TIMS::getAlternateFunction(static_cast<uint8_t>(m_timerId), pinId));
    pin.setMode(GPIO::MODE::ALTERNATE);
    pin.setSpeed(GPIO::SPEED::HIGH);

    return SUCCESS;
}

uint32_t HW_HAL_PWM::getTimerFrequency(){
    uint32_t timerFrequency = 0;

    // Computation for timerFrequency explained in section 6.2.15 of the manual
    // Meaning of PCLK and PPRE[1,2] explained in Figure 16 (Clock tree)

    switch (m_timerId) {
        case TimerId::TIM_1:
            timerFrequency = PCLK::freq2 * (BusClockConfig::PPRE2 > 1 ? 2 : 1);
            break;
        case TimerId::TIM_2:
        case TimerId::TIM_3:
        case TimerId::TIM_4:
            timerFrequency = PCLK::freq1 * (BusClockConfig::PPRE1 > 1 ? 2 : 1);
            break;
    }

    return timerFrequency;
}

GPIO_PIN HW_HAL_PWM::getAssociatedGPIOId(PWM_IDX idx) {
    constexpr GPIO_PIN pins[] {
        GPIO_073, GPIO_075, GPIO_077, GPIO_078,
        GPIO_000, GPIO_001, GPIO_002, GPIO_003,
        GPIO_006, GPIO_007, GPIO_016, GPIO_017,
        GPIO_060, GPIO_061, GPIO_062, GPIO_063
    };

    return pins[static_cast<int>(idx)];
}

}
