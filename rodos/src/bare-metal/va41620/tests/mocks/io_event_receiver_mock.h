#include "gmock/gmock.h"
#include "hal/genericIO.h"

namespace RODOS::testing {

class IOEventReceiverMock : public RODOS::IOEventReceiver {
public:
    MOCK_METHOD(void, onWriteFinished, (), (override));
    MOCK_METHOD(void, onReadFinished, (), (override));
    MOCK_METHOD(void, onDataReady, (), (override));
    virtual ~IOEventReceiverMock() = default;
};

}
