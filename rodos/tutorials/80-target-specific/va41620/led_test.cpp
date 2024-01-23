#include "rodos.h"

class LedTestThread : RODOS::StaticThread<> {
public:
    LedTestThread(GPIO_PIN idx)
        : RODOS::StaticThread<>()
        , m_gpio {idx}
    {}
private:
    HAL_GPIO m_gpio;

    void init() final{
        m_gpio.init(true);
    }

    void run() final{
        uint16_t outValue = 0;
        TIME_LOOP(0, 100*MILLISECONDS){
            PRINTF("toggling led at time %lld\n", NOW());
            outValue = static_cast<uint16_t>(~outValue);
            m_gpio.setPins(outValue);
        }
    }
};

LedTestThread test1{GPIO_101};
