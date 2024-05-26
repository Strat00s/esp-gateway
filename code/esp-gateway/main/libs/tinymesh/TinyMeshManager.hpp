#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
#include "../containers/StaticDeque.hpp"
#include "../interfaces/interfaceWrapperBase.hpp"


#define TMM_OK                0

#define TMM_NO_DATA           1

#define TMM_ERR_BUILD_PACKET  2
#define TMM_ERR_QUEUE_FULL    3
#define TMM_ERR_GET_DATA      4
#define TMM_ERR_CHECK_HEADER  5
#define TMM_ERR_RECEIVE       6
#define TMM_ERR_TIMEOUT       7
#define TMM_ERR_SEND_DATA     8
#define TMM_ERR_NULL          9

#define TMM_PACKET_DUPLICATE  10
#define TMM_QUEUE_EMPTY       11
#define TMM_SENT              12
#define TMM_AWAIT             13

#define TMM_MEDIUM_BUSY       14

#define TMM_REQUEST       0b00010000
#define TMM_RESPONSE      0b00100000
#define TMM_FORWARD       0b01000000
#define TMM_ERR_FORWARD   0b10000000

#define TMM_PACKET_RND_RESPONSE 0b00000010
#define TMM_PACKET_FORWARD      0b00000100
#define TMM_PACKET_REQUEST      0b00001000
#define TMM_PACKET_RESPONSE     0b00010000

#define TOA 50 //time on air

//TODO change timeout acording to network size (max PING)
//TODO scan network
//TODO fix initial TTS of a packet (currently starts at TOA)
//TODO change TOA to be requested from interface


typedef struct {
    TMPacket packet;
    unsigned long tts;
} packet_tts_t;


template <size_t Q_SIZE, size_t PID_SIZE = 10>
class TinyMeshManager {
private:
    InterfaceWrapperBase *interface = nullptr;

    uint8_t address      = TM_DEFAULT_ADDRESS;
    uint8_t node_type    = TM_NODE_TYPE_NORMAL;
    uint8_t sequence_num = 0;

    TMPacket packet; //incoming packet
    StaticDeque<packet_tts_t, Q_SIZE> send_queue;
    TMPacket *last_request;
    unsigned long last_loop_time = 0;

    TMPacketID pid_list[PID_SIZE];
    size_t pid_index = 0;

    uint8_t last_ret;


    unsigned long (*millis)();

    /** @brief Request handler to be implemented by the user.
     * 
     * @param request New request packet.
     * @param fwd If the packet is to be forwarded (don't send a response).
     * 
     * @return 0 on success.
     */
    uint8_t (*requestHandler)(TMPacket *request, bool fwd);

    /** @brief  Response handler to be implemented by the user.
     * 
     * @param request Request packet to which the response packet belongs.
     * @param response The response packet.
     * 
     * @return 0 on success.
     */
    uint8_t (*responseHandler)(TMPacket *request, TMPacket *response);

    /** @brief Check if a packet is a response to any queued packet
     * 
     * @param response Received packet
     * @return True if response packet is a response to the request packet.
     */
    bool isResponse(TMPacket *response) {
        for (size_t i = 0; i < send_queue.size(); i++) {
            if (response->getSource() == send_queue[i].getDestination() && response->getDestination() == send_queue[i].getSource()) {
                last_request = &send_queue[i];
                return true;
            }
        }
        return false;
    }


    /** @brief Create packet ID from packet.
     * 
     * @param packet Packet for which to create ID.
     * @return The packet ID.
     */
    TMPacketID createPacketID(TMPacket *packet) {
        TMPacketID pid;
        pid.setSource(packet->getSource());
        pid.setDestination(packet->getDestination());
        pid.setSequence(packet->getSequence());
        //pid.setFlags(packet->getFlags());
        pid.setRepeatCount(packet->getRepeatCount());
        return pid;
    }

    /** @brief Set specific packet ID to mirror a packet.
     * 
     * @param pid Packet ID which to set.
     * @param packet Packet which to mirror.
     */
    void setPacketID(TMPacketID *pid, TMPacket *packet) {
        pid->setSource(packet->getSource());
        pid->setDestination(packet->getDestination());
        pid->setSequence(packet->getSequence());
        //pid->setFlags(packet->getFlags());
        pid->setRepeatCount(packet->getRepeatCount());
    }

    /** @brief Save ID of a packet to pid_list.
     * 
     * @param packet 
     * @return True on success.
     * False if packet is a duplicate. 
     */
    bool savePacketID(TMPacket *packet) {
        for (uint8_t i = 0; i < PID_SIZE; i++) {
            
            //pid exists
            if (pid_list[i].getSource() == packet->getSource() &&
                pid_list[i].getSequence() == packet->getSequence() &&
                pid_list[i].getDestination() == packet->getDestination()) {
                
                //update pid
                if (pid_list[i].getRepeatCount() < packet->getRepeatCount()) {
                    setPacketID(&pid_list[i], packet);
                    return true;
                }
                // duplicate
                return false;
            }
        }

        //add new pid
        setPacketID(&pid_list[pid_index], packet);
        pid_index = (pid_index + 1) % PID_SIZE;
        return true;
    }


    /** @brief Classify packet if it is a response, requiest, to be forwarded and so on.
     * 
     * @return TMM_PACKET_ macros. 
     */
    uint8_t classifyPacket() {
        //if (isResponse(&send_queue[curr_index], &packet))
        //    return TMM_PACKET_RESPONSE;

        // response
        if (packet.isResponse()) {
            if (isResponse(&packet))
                return TMM_PACKET_RESPONSE;

            if (packet.getDestination() == address)
                return TMM_PACKET_RND_RESPONSE;

            return TMM_PACKET_FORWARD;
        }

        // request
        if (packet.getDestination() == address)
            return TMM_PACKET_REQUEST;

        if (packet.getDestination() == TM_BROADCAST_ADDRESS)
            return TMM_PACKET_REQUEST | TMM_PACKET_FORWARD;

        return TMM_PACKET_FORWARD;
    }


    uint8_t handleRequest(TMPacket *request, bool fwd) {
        if (request->getMessageType() == TM_MSG_PING) {
            if (fwd)
                return TMM_OK;

            uint8_t data = request->getData()[0];
            return queuePacket(request->getSource(), TM_MSG_OK, &data, 1);
        }

        return requestHandler(request, fwd);
    }

    /** @brief Handle incoming packets*/
    uint8_t receivePacket() {
        if (!interface->hasData())
            return TMM_NO_DATA;

        uint8_t len = TM_PACKET_SIZE;
        last_ret = interface->getData(packet.raw, &len);
        if (last_ret)
            return TMM_ERR_GET_DATA;

        last_ret = packet.checkHeader();
        if (last_ret)
            return TMM_ERR_CHECK_HEADER;

        return TMM_OK;
    }


public:
    TinyMeshManager(InterfaceWrapperBase *interface = nullptr) {
        this->interface = interface;
    }
    TinyMeshManager(uint8_t address, InterfaceWrapperBase *interface) : TinyMeshManager(interface) {
        this->address = address;
    }
    TinyMeshManager(uint8_t address, uint8_t node_type, InterfaceWrapperBase *interface) : TinyMeshManager(address, interface) {
        this->node_type = node_type;
    }

    inline void registerMillis(unsigned long (*millis)()) {
        this->millis = millis;
    }

    inline void registerRequestHandler(uint8_t (*func)(TMPacket *, bool)) {
        this->requestHandler = func;
    }

    inline void registerResponseHandler(uint8_t (*func)(TMPacket *, TMPacket *)) {
        this->responseHandler = func;
    }


    /** @brief Set node address*/
    inline void setAddress(uint8_t address) {
        this->address = address;
    }

    /** @brief Set node type*/
    void setNodeType(uint8_t node_type) {
        if (node_type > TM_NODE_TYPE_LP)
            node_type = TM_NODE_TYPE_NORMAL;
        this->node_type = node_type;
    }

    /** @brief Get node address*/
    inline uint8_t getAddress() {
        return address;
    }

    /** @brief Get node type*/
    inline uint8_t getNodeType() {
        return node_type;
    }

    /** @brief Get last internal returned value (mostly for debugging)*/
    inline uint8_t getStatus() {
        return last_ret;
    }


    /** @brief Queue request to be sent.
     * 
     * @param destination Destination node address.
     * @param message_type Message type.
     * @param data Data to send.
     * @param length Length of the data.
     * @return TMM_OK on success.
     */
    uint8_t queuePacket(uint8_t destination, uint8_t message_type, uint8_t *data, uint8_t length) {
        if (send_queue.reserve())
            return TMM_ERR_QUEUE_FULL;

        auto request = send_queue.last();
        last_ret = request->packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret) {
            request->packet.clear();
            send_queue.popBack();
            return TMM_ERR_BUILD_PACKET;
        }
        request->tts = 0;
        return TMM_OK;
    }

    /** @brief Main loop responsible for reading and handling incoming packets and sending queued packets.
     * 
     * @return What occured during the loop. Aditional information can be retrieved by getStatus().
     * @param TMM_ERR_RECEIVE
     * @param TMM_QUEUE_EMPTY
     * @param TMM_ERR_TIMEOUT
     * @param TMM_ERR_SEND_DATA
     * @param TMM_SENT
     * @param TMM_AWAIT
     * @param TMM_REQUEST
     * @param TMM_RESPONSE
     * @param TMM_FORWARD
     * @param TMM_ERR_NULL if interface is null
     */
    uint8_t loop() {
        uint8_t ret = TMM_OK;
        last_ret = receivePacket();

        // return if there was an error while processing possible incoming packet
        if (last_ret == TMM_ERR_GET_DATA || last_ret == TMM_ERR_CHECK_HEADER)
            return ret | TMM_ERR_RECEIVE;

        // got some data
        if (!last_ret) {
            last_ret = classifyPacket();

            // not a duplicate packet
            if (savePacketID(&packet)) {
                //increase ping
                if (packet.getMessageType() == TM_MSG_PING)
                    packet.getData()[0]++;

                //handle request
                if (last_ret & TMM_PACKET_REQUEST) {
                    last_ret = handleRequest(&packet, last_ret & TMM_PACKET_FORWARD);
                    ret |= TMM_REQUEST;
                }

                //handle response
                if (last_ret & TMM_PACKET_RESPONSE) {
                    last_ret = responseHandler(last_request, &packet);
                    last_request->clear();
                    ret |= TMM_RESPONSE;
                }

                //forward packet
                if (last_ret & TMM_PACKET_FORWARD) {
                    if (send_queue.full())
                        ret |= TMM_ERR_FORWARD;
                    else {
                        send_queue.pushBack({packet, TOA + (address * TOA) % TOA});
                        ret |= TMM_FORWARD;
                    }
                }
            }
        }

        //TODO queue packets as before
        //just push them to back
        //but now, every newly added packet has to be scheduled at least time-on-air ms away from next transmission

        if (send_queue.empty())
            return (ret & 0xF0) | TMM_QUEUE_EMPTY;

        //medium is busy
        if (interface->isMediumBusy())
            return ret | TMM_MEDIUM_BUSY;

        auto time_passed = millis() - last_loop_time;
        last_loop_time = millis();
        bool sent = false;
        bool skip = false;
        for (size_t i = 0; i < send_queue.size(); i++) {
            auto &next = send_queue[i];
        
            if (next->packet.empty())
                continue;

            if (next->packet.getRepeatCount() == 3) {
                next->packet.clear();
                ret |= TMM_ERR_TIMEOUT;
            }

            //update packet timer
            if (next->tts > time_passed) {
                next->tts -= time_passed;
                continue;
            }

            if (sent || skip) {
                if (next->tts < TOA)
                    next->tts = TOA;
                continue;
            }

            //send the packet
            last_ret = interface->sendData(next->packet.raw, next->packet.size());
            if (last_ret)
                skip = true;
        
            sent = true;
        }


        while (!send_queue.empty() && send_queue.first()->packet.empty())
            send_queue.popFront();

        while (!send_queue.empty() && send_queue.last()->packet.empty())
            send_queue.popBack();

        if (skip)
            return (ret & 0xF0) | TMM_ERR_SEND_DATA;

        if (!sent)
            return (ret & 0xF0) | TMM_AWAIT;

        return ret | TMM_SENT;
    }
};
