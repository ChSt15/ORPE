
#include "rodos.h"
#include "gateway.h"

// Provides a link to an UDP Broadcast-Network
static UDPInOut udp(-5001);
static LinkinterfaceUDP linkinterfaceUDP(&udp);
static Gateway gateway1(&linkinterfaceUDP, true);

// To "activate" the Topic-List-Distribution simply define this decoy object
static DistributedTopicRegisterDecoy nameNotImportant;
