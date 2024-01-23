#include "hal/hal_pwm.h"
#include "rodos.h"

#include "hw_hal_gpio.h"

namespace RODOS {

class HW_HAL_PWM{
  private:
    enum class TimerId { TIM_1 = 0, TIM_2 = 1, TIM_3 = 2, TIM_4 = 3 };
    enum class TimerChannelId { CH1 = 0, CH2 = 1, CH3 = 2, CH4 = 3 };

    constexpr static int32_t SUCCESS = 0;
    constexpr static int32_t ERROR = -1;

    const PWM_IDX m_selectedPwmIdx;
    const TimerId m_timerId;
    const TimerChannelId m_timerChannelId;
    TIMGeneralPurposeStruct& m_timer;

    uint32_t m_currentFrequency = 0;
    uint32_t m_currentIncrements = 0;

    uint32_t getTimerFrequency();

    int configurePinForPWMOutput(GPIO_PIN pinId);
    GPIO_PIN getAssociatedGPIOId(PWM_IDX idx);
    int32_t setTimerFrequencyAndIncrements();

  public:
    HW_HAL_PWM(PWM_IDX idx);

    int32_t init(uint32_t frequency, uint32_t increments);
    int32_t config(PWM_PARAMETER_TYPE type, int32_t paramVal);
    void reset();
    int32_t write(const uint32_t pulseWidthInIncs);
};


}
