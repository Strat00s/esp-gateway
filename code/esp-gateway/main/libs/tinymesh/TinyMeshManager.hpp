#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
//#include "../containers/StaticDeque.hpp"
#include "../interfaces/interfaceWrapperBase.hpp"
#include "../containers/LinkedList.hpp"


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

#define TMM_PID_NEW       0
#define TMM_PID_UPDATE    1
#define TMM_PID_DUPLICATE 2

#define TIMEOUT 1000


//TODO change timeout acording to network size (max PING)
    //TODO scan network


//TODO respons to repeated packet should just update repeat counter
//TODO clear forwarded packets to which we just got answer
//now they are full new packets

//TODO move from queue to list


template<size_t DATA_LEN>
struct packet_tts_t {
    TMPacket<DATA_LEN> packet;
    uint16_t tts = 0;   //time to send
    uint8_t flags = 0;  //0 -> normal, 1 -> keep once after sent, 2 -> remove on tts
};


template <size_t PID_SIZE = 10, uint8_t DATA_LEN = 16>
class TinyMeshManager {
private:
    InterfaceWrapperBase *interface = nullptr;

    uint8_t address      = TM_DEFAULT_ADDRESS;
    uint8_t node_type    = TM_NODE_TYPE_NORMAL;
    uint8_t sequence_num = 0;

    TMPacket<DATA_LEN> packet; //incoming packet
    TMPacket<DATA_LEN> *request;
    linkedList<packet_tts_t<DATA_LEN>> send_list;
    //linkedList<packet_tts_t<DATA_LEN>> response_list;
    unsigned long loop_timer = 0;
    uint16_t busy_timeout = 0;
    uint16_t toa = 0;

    TMPacketID pid_list[PID_SIZE];
    size_t pid_index = 0;


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
                    pid_list[i].setRepeatCount(packet->getRepeatCount());
                    printf("UPDATE: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[i].getSource(), pid_list[i].getDestination(), pid_list[i].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
                    return TMM_PID_UPDATE;
                }
                // duplicate
                printf("DUPLICATE: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[i].getSource(), pid_list[i].getDestination(), pid_list[i].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
                return TMM_PID_DUPLICATE;
            }
        }

        //add new pid
        pid_list[pid_index].setSource(packet->getSource());
        pid_list[pid_index].setDestination(packet->getDestination());
        pid_list[pid_index].setSequence(packet->getSequence());
        pid_list[pid_index].setRepeatCount(packet->getRepeatCount());
        printf("NEW: %03d-%03d:%03d | %03d-%03d:%03d\n", pid_list[pid_index].getSource(), pid_list[pid_index].getDestination(), pid_list[pid_index].getSequence(), packet->getSource(), packet->getDestination(), packet->getSequence());
        pid_index = (pid_index + 1) % PID_SIZE;
        return TMM_PID_NEW;
    }


    /** @brief Classify packet if it is a response, requiest, to be forwarded and so on.
     * 
     * @return Combinations of: `TMM_RECV_RESPONSE`, `TMM_RECV_RND_RESPONSE`, `TMM_RECV_FORWARD`, `TMM_RECV_REQUEST`.
     */
    uint8_t classifyPacket() {
        printf("Packet is a ");
        //requests
        if (!packet.isResponse()) {
            if (packet.getDestination() == address) {
                printf("REQUEST\n");
                return TMM_RECV_REQUEST;
            }

            if (packet.isBroadcast()) {
                printf("FORWARD REQUEST\n");
                return TMM_RECV_REQUEST | TMM_RECV_FORWARD;
            }

            printf("FORWARD\n");
            return TMM_RECV_FORWARD;
        }

        //go through sent packets and find request
        for (auto *item = send_list.front(); item != nullptr; item = item->next) {
            if (!item->data.packet.isResponse() &&
                packet.getSource() == item->data.packet.getDestination() &&
                packet.getDestination() == item->data.packet.getSource()) {

                request = &item->data.packet;

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
        }
        //response to us, but we have no request
        if (packet.getDestination() == address) {
            printf("RANDOM RESPONSE\n");
            return TMM_RECV_RND_RESPONSE;
        }

        printf("FORWARD\n");
        return TMM_RECV_FORWARD;
    }


    /** @brief Handle incoming packets*/
    uint8_t receivePacket() {
        if (!interface->hasData())
            return TMM_RECV_NO_DATA;

        uint8_t len = TM_HEADER_LENGTH + DATA_LEN;
        if (interface->getData(packet.raw, &len))
            return TMM_ERR_RECV_GET_DATA;

        if (packet.checkHeader())
            return TMM_ERR_RECV_HEADER;

        printf("Got new packet: ");
        printPacketSimple(&packet);

        return TMM_OK;
    }


    /** @brief Handle PING or forward request to user defined handler.
     * 
     * @return 0 on success.
     * 1 on failure.
     */
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
    TinyMeshManager(InterfaceWrapperBase *interface) {
        this->interface = interface;
    }
    TinyMeshManager(uint8_t address, InterfaceWrapperBase *interface) : TinyMeshManager(interface) {
        this->address = address;
    }
    TinyMeshManager(uint8_t address, uint8_t node_type, InterfaceWrapperBase *interface) : TinyMeshManager(address, interface) {
        this->node_type = node_type;
    }

    void registerMillis(unsigned long (*millis)()) {
        this->millis = millis;
    }

    void registerRequestHandler(uint8_t (*func)(TMPacket<DATA_LEN> *, bool)) {
        this->requestHandler = func;
    }

    void registerResponseHandler(uint8_t (*func)(TMPacket<DATA_LEN> *, TMPacket<DATA_LEN> *)) {
        this->responseHandler = func;
    }


    /** @brief Set node address*/
    void setAddress(uint8_t address) {
        this->address = address;
    }

    /** @brief Set node type*/
    void setNodeType(uint8_t node_type) {
        if (node_type > TM_NODE_TYPE_LP)
            node_type = TM_NODE_TYPE_NORMAL;
        this->node_type = node_type;
    }

    /** @brief Get node address*/
    uint8_t getAddress() {
        return address;
    }

    /** @brief Get node type*/
    uint8_t getNodeType() {
        return node_type;
    }

    uint8_t begin() {
        //update time on air for the first time
        toa = (this->interface->getTimeOnAir(TM_HEADER_LENGTH + DATA_LEN) + 1) * 1.2; //ceiling
        printf("TOA updated: %d\n", toa);

        return interface->startReception();
    }


    size_t queueSize() {
        return send_list.size();
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
        if (send_list.reserveBack())
            return TMM_ERR_QUEUE_FULL;

        auto *request_elem = send_list.back();
        request_elem->data.packet.clear();
        if (request_elem->data.packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length)) {
            send_list.popBack();
            return TMM_ERR_BUILD_PACKET;
        }

        request_elem->data.tts = 0;//toa + (millis() - loop_timer);
        request_elem->data.flags = request_elem->data.packet.isResponse();
        savePacketID(&request_elem->data.packet);
        printf("Queued packet in %d\n", request_elem->data.tts);
        return TMM_OK;
    }


    /** @brief Main loop responsible for reading and handling incoming packets and sending queued packets.
     * 
     * @return Returns any of the following macros: `TMM_RECV_NO_DATA`, `TMM_ERR_RECV_HEADER`, `TMM_ERR_RECV_GET_DATA`,
     * `TMM_SEND_QUEUE_EMPTY`, `TMM_RECV_REQUEST`, `TMM_RECV_RESPONSE`, `TMM_RECV_RND_RESPONSE`,
     * `TMM_RECV_FORWARD`, `TMM_MEDIUM_BUSY`, `TMM_SENT`, `TMM_SEND_QUEUED`, `TMM_SEND_TIMEOUT`, `TMM_ERR_SEND_DATA`.
     */
    uint16_t loop() {
        //save current loop time
        uint16_t loop_time = millis() - loop_timer;
        loop_timer = millis();

        uint16_t ret = receivePacket();

        //got some data
        if (!ret) {
            uint8_t packet_type = classifyPacket();
            ret |= packet_type;

            uint8_t pid_type = savePacketID(&packet);

            //not a duplicate packet
            if (pid_type != TMM_PID_DUPLICATE) {
                //increase ping
                if (packet.getMessageType() == TM_MSG_PING)
                    packet.getData()[0]++;

                //handle request
                //error only if: request AND new packet AND failed to handle request)
                if (packet_type & TMM_RECV_REQUEST) {
                    //not an update -> handle reaquest
                    if (!pid_type) {
                        if (handleRequest(&packet, packet_type & TMM_RECV_FORWARD))
                            ret |= TMM_ERR_HANDLE_REQUEST;
                    }
                    //update -> resend stored response
                    else {
                        bool found = false;
                        for (auto *item = send_list.front(); item != nullptr; item = item->next) {
                            auto *response = &item->data.packet;
                            if (response->empty() || !response->isResponse())
                                continue;
                            
                            if (response->getSource() == packet.getDestination() && response->getDestination() == packet.getSource()) {
                                item->data.tts = 0;
                                item->data.flags = 0;
                                found = true;
                                break;
                            }
                        }

                        if (!found)
                            printf("RESPONSE NOT FOUND!!!\n");
                    }
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
                    packet.setIsForwarded(true);
                    if (send_list.pushBack({packet, uint16_t(loop_time + toa)})) {
                        printf("Queue is full\n");
                        ret |= TMM_ERR_FWD_QUEUE_FULL;
                    }
                    else
                        printf("Queued packet in %d\n", loop_time + toa + (address % toa));
                }
            }
        }

        //nothing to do with empty queue
        if (send_list.empty())
            return ret | TMM_SEND_QUEUE_EMPTY;

        //medium is busy
        if (interface->isMediumBusy())
            busy_timeout = toa;
        if (busy_timeout > loop_time) {
            busy_timeout -= loop_time;
            return ret | TMM_MEDIUM_BUSY;
        }
        busy_timeout = 0;

        bool sent = false;
        //go through the queueu
        for (auto *item = send_list.front(); item != nullptr; item = item->next) {
            auto *next = &item->data;

            //skip empty packets
            if (next->packet.empty())
                continue;

            //update packet timer
            if (next->tts > loop_time) {
                next->tts -= loop_time;
                ret |= TMM_SEND_QUEUED;
                continue;
            }

            //packet is to removed before being sent
            if (next->flags == 2) {
                next->packet.clear();
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

            //flags the packet in queue for possible later reuse
            if (next->flags == 0b00000001) {
                next->flags =  0b00000010; //will be removed next time before being sent
                next->tts = TIMEOUT * 3 * 2; //timeout * repeat * 2
                continue;
            }

            //clear responses or forwared packets as they are to be sent only once
            if (next->packet.isForwarded()) {
                next->packet.clear();
                printf("Removed forward and responses\n");
                continue;
            }

            //clear timed out packets
            if (!next->packet.isForwarded() && next->packet.getRepeatCount() == 3) {
                next->packet.clear();
                printf("Packet TIMEOUT\n");
                ret |= TMM_SEND_TIMEOUT;
                continue;
            }

            //increase repeat count and update time to send
            next->packet.setRepeatCount(next->packet.getRepeatCount() + 1);
            next->tts = TIMEOUT;
        }

        //remove stale packets from queue
        while (!send_list.empty() && send_list.front()->data.packet.empty())
            send_list.popFront();
        while (!send_list.empty() && send_list.back()->data.packet.empty())
            send_list.popBack();

        return ret;
    }
};
