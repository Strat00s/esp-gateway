#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
#include "../containers/StaticDeque.hpp"
#include "../interfaces/interfaceWrapperBase.hpp"


#define TMM_OK                0

#define TMM_ERR_BUILD_PACKET  2
#define TMM_ERR_QUEUE_FULL    3

//loop return flags
#define TMM_RECV_NO_DATA        0b0000000000000001
#define TMM_ERR_RECV_HEADER     0b0000000000000010
#define TMM_ERR_RECV_GET_DATA   0b0000000000000100
#define TMM_SEND_QUEUE_EMPTY    0b0000000000001000
#define TMM_RECV_REQUEST        0b0000000000010000
#define TMM_RECV_RESPONSE       0b0000000000100000
#define TMM_RECV_RND_RESPONSE   0b0000000001000000
#define TMM_RECV_FORWARD        0b0000000010000000
#define TMM_MEDIUM_BUSY         0b0000000100000000
#define TMM_SENT                0b0000001000000000
#define TMM_SEND_QUEUED         0b0000010000000000
#define TMM_SEND_TIMEOUT        0b0000100000000000
#define TMM_ERR_SEND_DATA       0b0001000000000000
#define TMM_ERR_HANDLE_REQUEST  0b0010000000000000
#define TMM_ERR_HANDLE_RESPONSE 0b0100000000000000
#define TMM_ERR_FWD_QUEUE_FULL  0b1000000000000000

#define TIMEOUT 1000


//TODO change timeout acording to network size (max PING)
    //TODO scan network


//TODO respons to repeated packet should just update repeat counter
//TODO clear forwarded packets to which we just got answer
//now they are full new packets



template<size_t DATA_LEN>
struct packet_tts_t {
    TMPacket<DATA_LEN> packet;
    unsigned long tts;
};


template <size_t Q_SIZE, size_t PID_SIZE = 10, uint8_t DATA_LEN = 16>
class TinyMeshManager {
private:
    InterfaceWrapperBase *interface = nullptr;

    uint8_t address      = TM_DEFAULT_ADDRESS;
    uint8_t node_type    = TM_NODE_TYPE_NORMAL;
    uint8_t sequence_num = 0;

    TMPacket<DATA_LEN> packet; //incoming packet
    TMPacket<DATA_LEN> *request;
    StaticDeque<packet_tts_t<DATA_LEN>, Q_SIZE> send_queue;
    unsigned long last_loop_time = 0;
    unsigned long wait_for_med = 0;

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
     * 1 on failure.
     */
    uint8_t (*requestHandler)(TMPacket<DATA_LEN> *request, bool fwd);

    /** @brief  Response handler to be implemented by the user.
     * 
     * @param request Request packet to which the response packet belongs.
     * @param response The response packet.
     * 
     * @return 0 on success.
     * 1 on failure.
     */
    uint8_t (*responseHandler)(TMPacket<DATA_LEN> *request, TMPacket<DATA_LEN> *response);


    /** @brief Check if a packet is a response to any queued packet and save its reference.
     * 
     * @param response Received packet
     * @return True if response packet is a response to some currently stored packet.
     */
    bool isResponse(TMPacket<DATA_LEN> *response) {
        for (size_t i = 0; i < send_queue.size(); i++) {
            if (!send_queue[i].packet.isResponse() &&
                //response->getDestination() == address &&
                response->getDestination() == send_queue[i].packet.getSource() &&
                response->getSource() == send_queue[i].packet.getDestination()) {

                request = &send_queue[i].packet;
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
    TMPacketID createPacketID(TMPacket<DATA_LEN> *packet) {
        TMPacketID pid;
        pid.setSource(packet->getSource());
        pid.setDestination(packet->getDestination());
        pid.setSequence(packet->getSequence());
        pid.setRepeatCount(packet->getRepeatCount());
        return pid;
    }

    /** @brief Set specific packet ID to mirror a packet.
     * 
     * @param pid Packet ID which to set.
     * @param packet Packet which to mirror.
     */
    void setPacketID(TMPacketID *pid, TMPacket<DATA_LEN> *packet) {
        pid->setSource(packet->getSource());
        pid->setDestination(packet->getDestination());
        pid->setSequence(packet->getSequence());
        pid->setRepeatCount(packet->getRepeatCount());
    }

    /** @brief Save ID of a packet to pid_list.
     * 
     * @param packet 
     * @return 0 on new packet.
     * 1 on updated packet.
     * 2 if duplicate. 
     */
    uint8_t savePacketID(TMPacket<DATA_LEN> *packet) {
        printf("Saving ID: ");
        for (uint8_t i = 0; i < PID_SIZE; i++) {
            //pid exists
            if (pid_list[i].getSource() == packet->getSource() &&
                pid_list[i].getSequence() == packet->getSequence() &&
                pid_list[i].getDestination() == packet->getDestination()) {
                
                //update pid
                if (pid_list[i].getRepeatCount() < packet->getRepeatCount()) {
                    setPacketID(&pid_list[i], packet);
                    printf("UPDATE: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[i].getSource(), pid_list[i].getDestination(), pid_list[i].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
                    return 1;
                }
                // duplicate
                printf("DUPLICATE: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[i].getSource(), pid_list[i].getDestination(), pid_list[i].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
                return 2;
            }
        }

        //add new pid
        setPacketID(&pid_list[pid_index], packet);
        printf("NEW: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[pid_index].getSource(), pid_list[pid_index].getDestination(), pid_list[pid_index].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
        pid_index = (pid_index + 1) % PID_SIZE;
        return 0;
    }


    /** @brief Classify packet if it is a response, requiest, to be forwarded and so on.
     * 
     * @return Combinations of: `TMM_RECV_RESPONSE`, `TMM_RECV_RND_RESPONSE`, `TMM_RECV_FORWARD`, `TMM_RECV_REQUEST`.
     */
    uint8_t classifyPacket() {
        printf("Packet is a ");
        // response
        if (packet.isResponse()) {
            //packet is response to something we have in queue
            if (isResponse(&packet)) {
                //a response to some elses packet -> remove request and forward this response
                if (request->isForwarded()) {
                    request->clear();
                    printf("RESPONSE to other packet -> FORWARD\n");
                    return TMM_RECV_FORWARD;
                }

                //response to us
                if (packet.getDestination() == address) {
                    printf("RESPONSE\n");
                    return TMM_RECV_RESPONSE;
                }

                printf("--------------------------NOT RESPONSE NOR FORWARD--------------------------\n");
            }

            //response to us, but we have no request
            if (packet.getDestination() == address) {
                printf("RANDOM RESPONSE\n");
                return TMM_RECV_RND_RESPONSE;
            }

            printf("FORWARD\n");
            return TMM_RECV_FORWARD;
        }

        // request
        if (packet.getDestination() == address) {
            printf("REQUEST\n");
            return TMM_RECV_REQUEST;
        }

        if (packet.getDestination() == TM_BROADCAST_ADDRESS) {
            printf("FORWARD REQUEST\n");
            return TMM_RECV_REQUEST | TMM_RECV_FORWARD;
        }

        printf("FORWARD\n");
        return TMM_RECV_FORWARD;
    }


    uint8_t handleRequest(TMPacket<DATA_LEN> *request, bool fwd) {
        printf("Handling request packet\n");
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
            return TMM_RECV_NO_DATA;

        uint8_t len = TM_HEADER_LENGTH + DATA_LEN;
        last_ret = interface->getData(packet.raw, &len);
        if (last_ret)
            return TMM_ERR_RECV_GET_DATA;

        last_ret = packet.checkHeader();
        if (last_ret)
            return TMM_ERR_RECV_HEADER;

        printf("Got new packet: ");
        printPacketSimple(&packet);

        return TMM_OK;
    }


    void printPacketSimple(TMPacket<DATA_LEN> *packet) {
        printf("%3d %3d %3d ", packet->getSource(), packet->getDestination(), packet->getSequence());
        if (packet->getMessageType() == TM_MSG_OK)
            printf(" OK ");
        else if (packet->getMessageType() == TM_MSG_ERR)
            printf(" ER ");
        else 
            printf(" XX ");
        printf("%d\n", packet->getRepeatCount());
    }

public:
    unsigned long toa = 0;

    TinyMeshManager(InterfaceWrapperBase *interface) {
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

    inline void registerRequestHandler(uint8_t (*func)(TMPacket<DATA_LEN> *, bool)) {
        this->requestHandler = func;
    }

    inline void registerResponseHandler(uint8_t (*func)(TMPacket<DATA_LEN> *, TMPacket<DATA_LEN> *)) {
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

    size_t queueSize() {
        return send_queue.size();
    }

    /** @brief Queue request to be sent.
     * 
     * @param destination Destination node address.
     * @param message_type Message type.
     * @param data Data to send.
     * @param length Length of the data.
     * @return `TMM_OK` on success.
     * `TMM_ERR_QUEUE_FULL` if queue is full.
     * `TMM_ERR_BUILD_PACKET` if failed to build a packet
     */
    uint8_t queuePacket(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0) {
        if (send_queue.reserve())
            return TMM_ERR_QUEUE_FULL;

        packet_tts_t<DATA_LEN> *request = send_queue.last();
        last_ret = request->packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret) {
            request->packet.clear();
            send_queue.popBack();
            return TMM_ERR_BUILD_PACKET;
        }
        request->tts = 0;//toa + (millis() - last_loop_time);
        savePacketID(&request->packet);
        printf("Queued packet in %ld\n", request->tts);
        return TMM_OK;
    }

    //TODO proper return values for everything
    /** @brief Main loop responsible for reading and handling incoming packets and sending queued packets.
     * 
     * @return Returns any of the following macros: `TMM_RECV_NO_DATA`, `TMM_ERR_RECV_HEADER`, `TMM_ERR_RECV_GET_DATA`,
     * `TMM_SEND_QUEUE_EMPTY`, `TMM_RECV_REQUEST`, `TMM_RECV_RESPONSE`, `TMM_RECV_RND_RESPONSE`,
     * `TMM_RECV_FORWARD`, `TMM_MEDIUM_BUSY`, `TMM_SENT`, `TMM_SEND_QUEUED`, `TMM_SEND_TIMEOUT`, `TMM_ERR_SEND_DATA`.
     */
    uint16_t loop() {
        //save current loop time
        unsigned long loop_time = millis() - last_loop_time;
        last_loop_time = millis();

        //update time on air for the first time
        if (toa == 0) {
            toa = (this->interface->getTimeOnAir(TM_HEADER_LENGTH + DATA_LEN) + 1) * 1.2; //ceiling
            printf("TOA updated: %ld\n", toa);
        }

        uint16_t ret = receivePacket();

        //got some data
        if (!ret) {
            uint8_t packet_type = classifyPacket();
            ret |= packet_type;

            uint8_t pid_type = savePacketID(&packet);
            //not a duplicate packet
            if (pid_type != 2) {
                //increase ping
                if (packet.getMessageType() == TM_MSG_PING)
                    packet.getData()[0]++;

                //handle request
                //error only if: request AND new packet AND failed to handle request)
                if (packet_type & TMM_RECV_REQUEST) {
                    if (!pid_type) {
                        if (handleRequest(&packet, packet_type & TMM_RECV_FORWARD))
                            ret |= TMM_ERR_HANDLE_REQUEST;
                    }
                    //else
                        //TODO resend answer only
                }

                //handle response
                if (packet_type & TMM_RECV_RESPONSE && !pid_type) {
                    printf("Handling response packet\n");
                    if (responseHandler(request, &packet))
                        ret |= TMM_ERR_HANDLE_RESPONSE;
                    request->clear();
                }

                //forward packet
                if (packet_type & TMM_RECV_FORWARD) {
                    printf("Handling forward packet\n");
                    if (send_queue.full())
                        ret |= TMM_ERR_FWD_QUEUE_FULL;
                    else {
                        printf("Queued packet in %ld\n", loop_time + toa + (address % toa));
                        packet.setIsForwarded(true);
                        send_queue.pushBack({packet, loop_time + toa});
                    }
                }
            }
        }

        //nothing to do with empty queue
        if (send_queue.empty())
            return ret | TMM_SEND_QUEUE_EMPTY;

        //medium is busy
        if (interface->isMediumBusy()) {
            wait_for_med = millis();
            return ret | TMM_MEDIUM_BUSY;
        }
        if (millis() - wait_for_med <= toa)
            return ret | TMM_MEDIUM_BUSY;

        bool sent = false;
        bool skip = false;
        //go through the queueu
        for (size_t i = 0; i < send_queue.size(); i++) {
            packet_tts_t<DATA_LEN> *next = &send_queue[i];
        
            //skip empty packets
            if (next->packet.empty())
                continue;

            //clear timeouted packets
            if (!next->packet.isForwarded() && next->packet.getRepeatCount() == 3) {
                next->packet.clear();
                printf("Packet TIMEOUT\n");
                ret |= TMM_SEND_TIMEOUT;
            }

            //update packet timer
            if (next->tts > loop_time) {
                next->tts -= loop_time;
                ret |= TMM_SEND_QUEUED;
                continue;
            }

            //only one packet is sent per TOA
            if (sent) {
                if (next->tts < toa)
                    next->tts = toa;
                continue;
            }

            //send the packet
            if (interface->sendData(next->packet.raw, next->packet.size())) {
                ret |= TMM_ERR_SEND_DATA;
                printf("Failed to send packet\n");
                continue;
            }
            sent = true;
            ret |= TMM_SENT;
            printf("Packet %03d-%03d:%03d was sent\n", next->packet.getSource(), next->packet.getDestination(), next->packet.getSequence());

            //clear responses or forwared packets as they are to be sent only once
            if (next->packet.isResponse() || next->packet.isForwarded()) {
                next->packet.clear();
                printf("Removed forward and responses\n");
                continue;
            }

            //increase repeat count and update time to send
            next->packet.setRepeatCount(next->packet.getRepeatCount() + 1);
            next->tts = TIMEOUT;
        }

        //remove stale packets from queue
        while (!send_queue.empty() && send_queue.first()->packet.empty())
            send_queue.popFront();
        while (!send_queue.empty() && send_queue.last()->packet.empty())
            send_queue.popBack();

        return ret;
    }
};
