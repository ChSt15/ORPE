/*
 * @file: Distributed-Topic-Register
 * @date: Jun 10, 2022, August 2022
 * @author: S. Büttner, T. Hegel, E. Zischka, Sergio Montenegro
 */

/**
 * *************** See api/receiverNode+receiverNodesBitMap.txt  ***************
 *
 * Each node has one instance of a Distributed-Topic-Register
 * It has 2 functions / sides:
 * 1. It publishes the Topic-Ids of the local topics with  subscriber
 * 2. It gets Topic-Ids reports from other nodes and keep track
 *    which topic has subscriber in which nodes
 *
 * but please see api/receiverNode+receiverNodesBitMap.txt  !!!!
 */

#include "rodos.h"
#include "gateway/networkmessage.h" // for class TopicListReport
#include "netmsginfo.h"

//____________________________________________________________________________________________________
struct TopicListPerNode {
    uint32_t        nodeNr;    // see MAX_NUM_OF_NODES but for example on_posix we have big numbers, which are not sequentially
    uint8_t         nodeIndex; // 0..31 normally == nodeNr, but see comment of nodeNr
    TopicListReport topicListReport;
};

Topic<TopicListPerNode> topicListDistribution(TOPIC_ID_FOR_TOPICLIST_DISTRIBUTION, "topicListDistribution"); // ID == 1 -> always Broadcast

static Application nameNotImportant007("Distributed-Topic-Register", 22);

DistributedTopicRegisterDecoy::DistributedTopicRegisterDecoy() { } // just decoy to link this whole file (application)
//____________________________________________________________________________________________________
class DistributedTopicRegister : public SubscriberReceiver<TopicListPerNode>, public StaticThread<>  {
  public:
    int64_t lastReportTimeOfNode[MAX_NUM_OF_NODES];
    TopicListPerNode topicListFromMyNode; // topic report for all nodes

    DistributedTopicRegister() : 
        SubscriberReceiver<TopicListPerNode>(topicListDistribution, "Distributed-Topic-Register"), 
        StaticThread<>("Distributed-Topic-Register", 10) { // low Priority: not time critical
            for(uint32_t i = 0; i < MAX_NUM_OF_NODES; i++) lastReportTimeOfNode[i] = END_OF_TIME;
    }

    void run();                            // side 1: topic reports distribution
    void put(TopicListPerNode& hisTopics); // side 2: collect reports and update local topics

} distributedTopicRegister;

//_________________________________________ Side 1: topic reports distribution
void DistributedTopicRegister::run() {

    //_____________ preparations: which are my local topics with subscribers?
    NetMsgInfo netMsgInfo; //(NetMsgType::TOPIC_LIST);

    ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
        if(iter->mySubscribers != 0) topicListFromMyNode.topicListReport.add(iter->topicId);
    }

    //_______________________________________________________
    TIME_LOOP(1117 * MILLISECONDS, 3 * SECONDS) {
        topicListFromMyNode.nodeNr    = (uint32_t)getNodeNumber();
        topicListFromMyNode.nodeIndex = topicListFromMyNode.nodeNr % MAX_NUM_OF_NODES;

        netMsgInfo.init(NetMsgType::TOPIC_LIST);
        topicListDistribution.publish(topicListFromMyNode, true, &netMsgInfo); // always the same, but important for new nodes

        //____________________ find dead nodes and remove from bitmap of receivers

        int64_t definitionOfVeryOld = NOW() - 10*SECONDS;
        for(uint32_t i = 0; i < MAX_NUM_OF_NODES; i++) {
            if(lastReportTimeOfNode[i] < definitionOfVeryOld) { // if dead, set corresponding bit to 0 in all topics
                uint32_t zeroAtBitPos = ~(0x01u  << i);
                ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
                    iter->receiverNodesBitMap &= zeroAtBitPos;
                } // for all topics
            } // if node is dead
        } // for all nodes
    } // TIME_LOOP
} // run

//_________________________________________ Side 2: collect topic reports and update local topics
void DistributedTopicRegister::put(TopicListPerNode& hisTopics) {
    if(hisTopics.nodeNr == (uint32_t)getNodeNumber()) return; // Ignore my own reports

    uint32_t oneAtBitPos = 0x01u  << hisTopics.nodeIndex; // See line 54: 0..31
    lastReportTimeOfNode[hisTopics.nodeIndex] = NOW();

    //PRINTF(" got report from %d -> %d: ", hisTopics.nodeNr, hisTopics.nodeIndex);
    //for(uint32_t i = 0; i < hisTopics.topicListReport.numberOfTopics; i++) PRINTF(" %d ", hisTopics.topicListReport.topicList[i]);
    //PRINTF("\n");


    // I have 2 options:
    //   1. For each topic_id in the report: scan all my local topics
    //   2. For each of my local topics: scan all topic_ids in the report
    // I have the feeling Nr. 2 is more efficient.

    ITERATE_LIST(TopicInterface, TopicInterface::topicList) {
        if(hisTopics.topicListReport.find(iter->topicId)) iter->receiverNodesBitMap |= oneAtBitPos;
    }
} 

