/*
 * @date: Agust 2022
 * @author: Sergio Montenegro
 *
 * print the external subscribers for each topic
 * as result from Distributed-Topic-Register
 */

#include "rodos.h"

DistributedTopicRegisterDecoy nameNotImportant008;

void printBin(uint32_t val) {
    for(int i = 0; i < 32; i++) {
        PRINTF("%c", (val & 0x80000000) ? '1' : '0');
        val = val << 1;
    }
}


//____________________________________________________________________________________________________
class PrintTopicRecieverNodes : public StaticThread<> {
    void run() {
        TIME_LOOP(2017 * MILLISECONDS, 2 * SECONDS) {
            PRINTF("–––––––––––––––––––––––––––––––––––––––––––––––––––––––– Iam %2d\n", static_cast<int>(getNodeNumber() % 32));
            ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
                if(iter->topicId < 2000 && iter->topicId >= 1000) {
                    PRINTF("topic %4d %9s: rec %2d bitmap %08x: ", static_cast<int>(iter->topicId), iter->name, static_cast<int>(iter->receiverNodesBitMap2Index()), static_cast<unsigned int>(iter->receiverNodesBitMap));
                    printBin(iter->receiverNodesBitMap);
                    PRINTF("\n");
                }
            } // for all topics

        } // TIME_LOOP
    }     // run


} printTopicRecieverNodes;
