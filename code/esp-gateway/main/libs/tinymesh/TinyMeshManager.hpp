#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
#include "../interfaces/InterfaceManager.hpp"


#define TMM_QUEUE_SIZE 5
#define TMM_PID_SIZE 10


#define TMM_OK               0

#define TMM_QUEUE_EMPTY      1
#define TMM_SENT             2
#define TMM_AWAIT            3
#define TMM_NO_DATA          4

#define TMM_ERR_TIMEOUT      5
#define TMM_ERR_SEND_DATA    6
#define TMM_ERR_BUILD_PACKET 7
#define TMM_ERR_QUEUE_FULL   8
#define TMM_ERR_GET_DATA     9
#define TMM_ERR_CHECK_HEADER 10
#define TMM_ERR_RECEIVE       11


#define TMM_PACKET_DUPLICATE 0
#define TMM_PACKET_RND_RESPONSE 0
#define TMM_PACKET_FORWARD 0
#define TMM_PACKET_REQUEST 0
#define TMM_PACKET_RESPONSE 0


class TinyMeshManager {
private:
    InterfaceManager *if_manager = nullptr;
    
    uint8_t address      = TM_DEFAULT_ADDRESS;
    uint8_t gateway      = TM_BROADCAST_ADDRESS;
    uint8_t node_type    = TM_NODE_TYPE_NODE;
    uint8_t sequence_num = 0;

    TMPacket packet;
    TMPacket send_queue[TMM_QUEUE_SIZE];
    uint8_t send_index = 0;
    uint8_t curr_index = 0;
    uint8_t queue_len = 0;
    int repeat_timer = 0;

    TMPacketID pid_list[TMM_PID_SIZE];
    uint8_t pid_index = 0;

    uint8_t last_ret;


    unsigned long (*millis)();


    uint8_t (*requestHandler)(TMPacket *request, bool fwd);
    uint8_t (*responseHandler)(TMPacket *request, TMPacket *response);


    /** @brief Check if packet is a response to the packet being currently sent.
     * 
     * @param response Response packet
     * @return 
     */
    bool isResponse(TMPacket *request, TMPacket *response) {
        //response source == request destination
        // AND
        //response destination == response source
        // AND
        //response is OK OR ERR
        return (response->getSource() == request->getDestination() &&
                response->getDestination() == request->getSource() &&
                (response->getMessageType() == TM_MSG_OK || response->getMessageType() == TM_MSG_ERR));
    }

    /** @brief Create a packet ID from packet
     *
     * @param packet 
     * @return TMPacketID
     */
    TMPacketID createPacketID(TMPacket *packet) {
        TMPacketID pid;
        pid.setSource(packet->getSource());
        pid.setDestination(packet->getDestination());
        pid.setSequence(packet->getSequence());
        pid.setMessageType(packet->getMessageType());
        pid.setRepeatCount(packet->getRepeatCount());
        return pid;
    }

    void setPacketID(TMPacketID *pid, TMPacket *packet) {
        pid->setSource(packet->getSource());
        pid->setDestination(packet->getDestination());
        pid->setSequence(packet->getSequence());
        pid->setMessageType(packet->getMessageType());
        pid->setRepeatCount(packet->getRepeatCount());
    }

    /** @brief Save packet ID.
     * 
     * @param packet_id 
     * @param tts 
     */
    bool savePacketID(TMPacket *packet) {
        for (uint8_t i = 0; i < TMM_PID_SIZE; i++) {
            if (pid_list[i].getSource() == packet->getSource() &&
                pid_list[i].getSequence() == packet->getSequence() &&
                pid_list[i].getDestination() == packet->getDestination()) {
                    if (pid_list[i].getRepeatCount() < packet->getRepeatCount()) {
                        setPacketID(&pid_list[i], packet);
                        return true;
                    }
                    //duplicate
                    return false;
                }
        }

        setPacketID(&pid_list[pid_index], packet);
        pid_index++;
        if (pid_index >= TMM_PID_SIZE)
            pid_index = 0;
        
        return true;
    }

    /** @brief Classify packet if it is for us or not or whetere it is a response or request or something else.
     * 
     * @return 
     */
    uint8_t classifyPacket() {
        if (isResponse(&send_queue[curr_index], &packet))
            return TMM_PACKET_RESPONSE;

        //response
        if (packet.getMessageType() == TM_MSG_OK && packet.getMessageType() == TM_MSG_ERR) {
            if (packet.getDestination() == address)
                return TMM_PACKET_RND_RESPONSE;

            return TMM_PACKET_FORWARD;
        }

        //request
        if (packet.getDestination() == address)
            return TMM_PACKET_REQUEST;

        if (packet.getDestination() == TM_BROADCAST_ADDRESS)
            return TMM_PACKET_REQUEST | TMM_PACKET_FORWARD;

        return TMM_PACKET_FORWARD;
    }

    uint8_t handleRequest(TMPacket *request, bool fwd) {
        if (request->getMessageType() == TM_MSG_PING){
            if (fwd)
                return TMM_OK;
            
            uint8_t data = request->getData()[0];
            return sendResponse(request->getSource(), TM_MSG_OK, &data, 1);
        }

        return requestHandler(request, fwd);
    }


public:
    TinyMeshManager(InterfaceManager *interface_manager) {
        this->if_manager = interface_manager;
    }
    TinyMeshManager(InterfaceManager *interface_manager, uint8_t address) : TinyMeshManager(interface_manager) {
        this->address = address;
    }
    TinyMeshManager(InterfaceManager *interface_manager, uint8_t address, uint8_t node_type) : TinyMeshManager(interface_manager, address) {
        this->node_type = node_type;
    }
    TinyMeshManager(InterfaceManager *interface_manager, uint8_t address, uint8_t node_type, uint8_t gateway) : TinyMeshManager(interface_manager, address, node_type) {
        this->gateway = gateway;
    }

    ~TinyMeshManager();


    inline void registerMillis(unsigned long (*millis)()) {
        this->millis = millis;
    }

    inline void registerRequestHandler(uint8_t (*func)(TMPacket *, bool)) {
        this->requestHandler = func;
    }

    inline void registerResponseHandler(uint8_t (*func)(TMPacket *, TMPacket *)) {
        this->responseHandler = func;
    }


    /** @brief Set this node address*/
    void setAddress(uint8_t address) {
        this->address = address;
    }

    /** @brief Set gateway address*/
    void setGateway(uint8_t address) {
        this->gateway = address;
    }

    /** @brief Set tjos mpde type*/
    void setNodeType(uint8_t node_type) {
        if (node_type > TM_NODE_TYPE_OTHER)
            node_type = TM_NODE_TYPE_OTHER;
        this->node_type = node_type;
    }

    /** @brief Get this node address*/
    uint8_t getAddress() {
        return address;
    }
    /** @brief Get gateway address*/
    uint8_t getGateway() {
        return gateway;
    }
    /** @brief Get node type*/
    uint8_t getNodeType() {
        return node_type;
    }


    uint8_t getStatus() {
        return last_ret;
    }


    uint8_t sendResponse(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0) {
        last_ret = packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret)
            return TMM_ERR_BUILD_PACKET;

        last_ret = if_manager->sendData(packet.raw, packet.size());
        if (last_ret)
            return TMM_ERR_SEND_DATA;

        if (!savePacketID(&packet))
            return TMM_PACKET_DUPLICATE;

        return TMM_OK;
    }

    /** @brief Create packet and add it to queue to be sent when in loop
     * 
     * @param destination 
     * @param message_type 
     * @param repeat_cnt 
     * @param data 
     * @param length 
     * @return 
     */
    uint8_t queueRequest(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0, uint16_t timeout_ms = 1000) {
        if (send_index == curr_index && queue_len)
            return TMM_ERR_QUEUE_FULL;

        last_ret = send_queue[send_index].buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret) {
            send_queue[send_index].clear();
            return TMM_ERR_BUILD_PACKET;
        }

        queue_len++;
        send_index++;
        if (send_index >= TMM_QUEUE_SIZE)
            send_index = 0;

        return TMM_OK;
    }

    uint8_t receivePacket() {
        if (!if_manager->hasData())
            return TMM_NO_DATA;

        uint8_t len = TM_PACKET_SIZE;
        last_ret = if_manager->getNextData(packet.raw, &len);
        if (last_ret)
            return TMM_ERR_GET_DATA;
        
        last_ret = packet.checkHeader();
        if (last_ret)
            return TMM_ERR_CHECK_HEADER;
        
        last_ret = classifyPacket();

        if (!savePacketID(&packet))
            return TMM_PACKET_DUPLICATE;

        if (last_ret & TMM_PACKET_RND_RESPONSE)
            return TMM_PACKET_RND_RESPONSE;

        //forward packet first
        if (last_ret & TMM_PACKET_FORWARD) {
            if (packet.getMessageType() == TM_MSG_PING)
                packet.getData()[0]++;
            if_manager->sendData(packet.raw, packet.size());
        }

        //handle request
        if (last_ret & TMM_PACKET_REQUEST) {
            last_ret = handleRequest(&packet, last_ret & TMM_PACKET_FORWARD);
            return TMM_PACKET_REQUEST;
        }

        //handle response
        if (last_ret & TMM_PACKET_RESPONSE) {
            last_ret = responseHandler(&send_queue[curr_index], &packet);
            return TMM_PACKET_RESPONSE;
        }

        return TMM_PACKET_FORWARD;
    }


    uint8_t loop(TMPacketID *packet_id) {
        last_ret = receivePacket();
        if (last_ret)
            return TMM_ERR_RECEIVE;

        if (!queue_len)
            return TMM_QUEUE_EMPTY;

        //empty curr_index
        if (send_queue[curr_index].empty()) {
            curr_index++;
            if (curr_index >= TMM_QUEUE_SIZE)
                curr_index = 0;
            repeat_timer = 0;
        }

        //repeat
        if (repeat_timer <= 0) {
            if (send_queue[curr_index].getRepeatCount() == 4) {
                //total timeout
                queue_len--;
                send_queue[curr_index].clear();
                return TMM_ERR_TIMEOUT;
            }

            last_ret = if_manager->sendData(send_queue[curr_index].raw, send_queue[curr_index].size());
            if (last_ret)
                return TMM_ERR_SEND_DATA;
            
            send_queue[curr_index].setRepeatCount(send_queue[curr_index].getRepeatCount() + 1);
            repeat_timer == 1000;
            return TMM_SENT;
        }
        return TMM_AWAIT;
    }
};
