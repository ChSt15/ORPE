#include <rodos.h>

using namespace RODOS;

Topic<int32_t> intTopic(-1, "intTopic");
Topic<double>  doubleTopic(-1, "doubleTopic");

//_____________________________________________________________________
class MySubscriber : public SubscriberReceiver<int32_t>, SubscriberReceiver<double> {
public:
  MySubscriber() : SubscriberReceiver<int32_t>(intTopic), SubscriberReceiver<double>(doubleTopic) {}
  
  void put(int32_t& data) override {
    PRINTF("intTopic received: %d\n", data);
  }
  
  void put(double& data) override {
    PRINTF("doubleTopic received: %f\n", data);
  }
} mySubscriber;

//_____________________________________________________________________
class MyPublisher : public StaticThread<> {
public:  
  void run() override {
    int32_t intValue    = 0;
    double  doubleValue = 0;
    
    TIME_LOOP(1*SECONDS, 1*SECONDS) {
      PRINTF("Sending...\n");
      
      intTopic.publish(intValue);
      doubleTopic.publish(doubleValue);
      
      intValue++;
      doubleValue += 0.1;
    }
  }
} myPublisher;

