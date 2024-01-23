#include <stddef.h>
#include "rodos.h"

#include "gateway/linkinterface.h"
#include "gateway/gateway.h"
#include "timemodel.h"
#include "netmsginfo.h"

#include "Datalink.h"

// UART setup
static HAL_UART uart(UART_IDX4);
static int init_dummy = uart.init(115200);

// Gateway setup
static LinkinterfaceUART uart_linkinterface(&uart, 115200);
static Gateway uart_gateway(&uart_linkinterface, true);

static UDPInOut udp(50000, "255.255.255.255");
static LinkinterfaceUDP linkinterface(&udp);
static Gateway udp_gateway(&linkinterface, true);

//Topics
RODOS::Topic<OrpePoseEst> orpePoseEstTopic(100, "Camera Topic");
RODOS::Topic<bool> orpePowerCommandTopic(-1, "OrpePowerCommandTopic");
RODOS::Topic<float> orpeTestingTopic(501, "OrpeTesting");

//Buffers
RODOS::CommBuffer<bool> orpePowerCommandBuf;

//Subscribers
RODOS::Subscriber orpePowerCommandSubr(orpePowerCommandTopic, orpePowerCommandBuf, "Orpe Power Command");

//Router
RODOS::Router gatewayRouter(false, &uart_gateway, &udp_gateway);

// Init before scheduling (dont know why, taken from examples -Max)
class GatewayInitiator : public Initiator
{
    void init()
    {
        // Add Topic to forward
        uart_gateway.resetTopicsToForward();
        uart_gateway.addTopicsToForward(&orpePoseEstTopic);
        uart_gateway.addTopicsToForward(&orpePowerCommandTopic);

        //udp_gateway.resetTopicsToForward();
        //udp_gateway.addTopicsToForward(&orpePoseEstTopic);
        //udp_gateway.addTopicsToForward(&orpePowerCommandTopic);
	
	//udp_gateway.addTopicsToForward(&orpeTestingTopic);
	uart_gateway.addTopicsToForward(&orpeTestingTopic);
    }
};

GatewayInitiator gatewayinitiator;


namespace RODOS {

struct NodeLookup {
    int32_t  node_nr  = -1;
    uint32_t node_bit = 0;
    Gateway *gateway  = nullptr;
};

static NodeLookup g_node_lookup[MAX_NUM_OF_NODES];

void UartRouter::routeMsg(NetworkMessage &msg, uint32_t linkid) {
    int32_t sender_node_nr = msg.get_senderNode();
    uint32_t sender_bit = 1u << (static_cast<uint32_t>(sender_node_nr) % MAX_NUM_OF_NODES);

    for (size_t i = 0; i < MAX_NUM_OF_NODES; i++) {
        // already seen messages from this sender
        if (g_node_lookup[i].node_nr == sender_node_nr)
            break;

        // empty entry => reached the end without finding the node...
        if (g_node_lookup[i].gateway == nullptr) {
            // ...therefore I need to search through the known gateways...
            for (size_t j = 0; j < this->numberOfGateways; j++) {
                if (this->gateways[j]->getLinkIdentifier() == linkid) {
                    //...and add a new entry in the lookup table
                    g_node_lookup[i].node_nr  = sender_node_nr;
                    g_node_lookup[i].node_bit = sender_bit;
                    g_node_lookup[i].gateway  = this->gateways[j];
                    break;
                }
            }
            break;
        }
    }

    if (!shouldRouteThisMsg(msg, linkid)) return;

    uint32_t receiver_bitmap  = msg.get_receiverNodesBitMap();
    uint32_t receiver_bit     = 1u;
    size_t   receiver_pending = 0;

    for (size_t i = 0; i < MAX_NUM_OF_NODES; i++, receiver_bit <<= 1) {
        if ((receiver_bitmap & receiver_bit) == 1)
            receiver_pending++;

        if (g_node_lookup[i].gateway != nullptr) {
            if ((g_node_lookup[i].node_bit & receiver_bitmap) == 0)
                continue;

            // found a valid receiver => forward message
            receiver_pending--;
            if (shouldRouteThisMsgToGateway(msg, linkid, g_node_lookup[i].gateway)) {
                g_node_lookup[i].gateway->sendNetworkMessage(msg);
            }
        }
    }

    // there are still some receivers pending...
    if (receiver_pending > 0) {
        bool found;
        // ...go through all the gateways...
        for (size_t j = 0; j < this->numberOfGateways; j++) {
            found = false;
            for (size_t i = 0; i < MAX_NUM_OF_NODES; i++) {
                if (g_node_lookup[i].gateway == nullptr)
                    break;
                else if (g_node_lookup[i].gateway == this->gateways[j]) {
                    found = true;
                    break;
                }
            }
            // ...and forward "msg" to all unknowns
            if (!found && shouldRouteThisMsgToGateway(msg, linkid, this->gateways[j])) {
                this->gateways[j]->sendNetworkMessage(msg);
            }
        }
    }
}

}
