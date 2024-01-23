#include "demo_topics.h"

Topic<TestData> topic1(1001, "topic1");
Topic<TestData> topic2(1002, "topic2");

// just to force the linker to include the RODOS::topicRegister
static DistributedTopicRegisterDecoy nameNotImportant;
