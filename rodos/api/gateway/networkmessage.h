


/**
* @file networkMessages.h
* @date 2013/06/30
* @author Sergio Montenegro
*
* @brief Messages to be exchanged between nodes and network
*
*/

#pragma once

#include <stdint.h>
#include "rodos-assert.h"
#include "netmsginfo.h"

/******** Generic Network Message ***************/

namespace RODOS {

/***** Topic list reports for network management ***/

class TopicListReport {
public:
    uint32_t numberOfTopics;             ///< total number of topics assigned for the network
    uint32_t topicList[MAX_SUBSCRIBERS]; ///< simple list of topic IDs subscribed for network

    TopicListReport() { init(); }      ///< Constructor, initializes topicList.
    void init() { numberOfTopics = 0; }

    /** Look up list, whether a topic is set for network messages.
     * @param[in] topicId ID to check whether topic is listed
     */
    bool find(const uint32_t topicId) const {
        for (uint32_t i = 0; i < numberOfTopics; i++) {
            if (topicId == topicList[i]) {
                return true;
            }
        }
        return false;
    }

    /** Adds a new topic to list, if not already present.
     * @param[in] topicId ID to add to the list
     */
    void add(const uint32_t topicId) {
        RODOS_ASSERT_IFNOT_RETURN_VOID(numberOfTopics < MAX_SUBSCRIBERS - 1); // topics avialable
        if (find(topicId)) return;
        topicList[numberOfTopics] = topicId;
        numberOfTopics++;
    }

    /// returns the size needed for transmission
    size_t numberOfBytesToSend() { return (sizeof(long) + numberOfTopics * sizeof(topicList[0])); }
} __attribute__((packed));


}  // namespace



