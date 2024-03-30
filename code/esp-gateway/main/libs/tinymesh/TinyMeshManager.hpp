#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
#include "interfaces/InterfaceManager.hpp"


#define TMM_QUEUE_SIZE 5
#define TMM_PID_SIZE 10


#define TMM_TX_BUILD_PACKET 0
#define TMM_TX_SEND_DATA 0
#define TMM_SAVE_FAILED 0


#define TMM_RX_NO_DATA 0
#define TMM_RX_GET_DATA 0
#define TMM_RX_CHECK_PACKET 0
#define TMM_RX_DUPLICATE 0
#define TMM_RX_WRONG_PACKET 0
#define TMM_RX_REQUEST 0


#define TM_PACKET_DUPLICATE 0
#define TM_PACKET_INV_RESPONSE 0
#define TM_PACKET_RND_RESPONSE 0
#define TM_PACKET_FORWARD 0
#define TM_PACKET_REQUEST 0
#define TM_PACKET_RESPONSE 0
#define TMM_RX_RESPONSE 0
#define TMM_RX_FORWARD 0


class TinyMeshManager {
private:
    InterfaceManagerBase *if_manager = nullptr;
    
    uint8_t address   = TM_DEFAULT_ADDRESS;
    uint8_t gateway   = TM_BROADCAST_ADDRESS;
    uint8_t node_type = TM_NODE_TYPE_NODE;
    uint16_t sequence_num = 0;

    TMPacket packet;
    TMPacket send_queue[TMM_QUEUE_SIZE];
    uint8_t send_index = 0;
    uint8_t curr_index = 0;
    uint8_t queue_len = 0;
    int repeat_timer = 0;

    TMPacketID pid_list[TMM_PID_SIZE];
    uint8_t pid_index = 0;

    uint8_t last_ret;

    void delay(uint32_t ms) {

    }

    uint32_t millis() {
        return 0;
    }

    //uint8_t customRequestHandler();
    //uint8_t customResponseHandler(TMPacketID request_id, packet_t *response_packet);

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

public:
    TinyMeshManager(InterfaceManagerBase *interface_manager) {
        this->if_manager = interface_manager;
    }
    TinyMeshManager(InterfaceManagerBase *interface_manager, uint8_t address) : TinyMeshManager(interface_manager) {
        this->address = address;
    }
    TinyMeshManager(InterfaceManagerBase *interface_manager, uint8_t address, uint8_t node_type) : TinyMeshManager(interface_manager, address) {
        this->node_type = node_type;
    }
    TinyMeshManager(InterfaceManagerBase *interface_manager, uint8_t address, uint8_t node_type, uint8_t gateway) : TinyMeshManager(interface_manager, address, node_type) {
        this->gateway = gateway;
    }

    ~TinyMeshManager();


    void setAddress(uint8_t address) {
        this->address = address;
    }
    void setGateway(uint8_t address) {
        this->gateway = address;
    }
    void setNodeType(uint8_t node_type) {
        this->node_type = node_type;
    }


    uint8_t getAddress() {
        return address;
    }
    uint8_t getGateway() {
        return gateway;
    }
    uint8_t getNodeType() {
        return node_type;
    }


    uint8_t getStatus() {
        return last_ret;
    }



    /** @brief Get next free packet from send queue.
     * Used for direct access to the packet skip unnecesary data copying.
     * 
     * @return uint8_t* 
     */
    uint8_t *getNextSendPacket() {
        return nullptr;
    }


    uint8_t sendPacket(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0) {
        last_ret = packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret)
            return TMM_SEND_BUILD_PACKET;

        last_ret = if_manager->sendData(packet.raw, packet.size());
        if (last_ret)
            return TMM_SEND_SEND_DATA;

        savePacketID(&packet);
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
    uint8_t queuePacket(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0, uint16_t timeout_ms = 1000) {
        //TODO repeat count handling
        if (send_index == curr_index && queue_len)
            return TMM_QUEUE_FULL;

        last_ret = send_queue[send_index].buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
        if (last_ret) {
            send_queue[send_index].clear();
            return TMM_QUEUE_BUILD_PACKET;
        }

        queue_len++;
        send_index++;
        if (send_index >= TMM_QUEUE_SIZE)
            send_index = 0;

        return TMM_OK;
    }

    uint8_t receivePacket() {
        if (!if_manager->hasData())
            return TMM_RX_NO_DATA;

        uint8_t len = TM_PACKET_SIZE;
        last_ret = if_manager->getNextData(packet.raw, &len);
        if (last_ret)
            return TMM_RX_GET_DATA;
        
        last_ret = packet.checkHeader();
        if (last_ret)
            return TMM_RX_CHECK_PACKET;
        
        last_ret = classifyPacket();

        if (!savePacketID(&packet))
            return TMM_RX_DUPLICATE;

        if (last_ret & TM_PACKET_RND_RESPONSE)
            return TMM_RX_WRONG_PACKET;

        //forward packet first
        if (last_ret & TM_PACKET_FORWARD) {
            if (packet.getMessageType() == TM_MSG_PING)
                packet.getData()[0]++;
            if_manager->sendData(packet.raw, packet.size());
        }

        //handle request
        if (last_ret & TM_PACKET_REQUEST) {
            handleRequest(&packet);
            return TMM_RX_REQUEST;
        }

        //handle response
        if (last_ret & TM_PACKET_RESPONSE) {
            last_ret = handleResponse(&send_queue[curr_index], &packet);
            return TMM_RX_RESPONSE;
        }

        return TMM_RX_FORWARD;
    }

    /** @brief Classify packet if it is for us or not or whetere it is a response or request or something else.
     * 
     * @return 
     */
    uint8_t classifyPacket() {
        if (isResponse(&packet))
            return TM_PACKET_RESPONSE;

        //response
        if (packet.getMessageType() == TM_MSG_OK && packet.getMessageType() == TM_MSG_ERR) {
            if (packet.getDestination() == address)
                return TM_PACKET_RND_RESPONSE;

            return TM_PACKET_FORWARD;
        }

        //request
        else {
            if (packet.getDestination() == address)
                return TM_PACKET_REQUEST;

            else if (packet.getDestination() == TM_BROADCAST_ADDRESS)
                if (node_type == TM_NODE_TYPE_GATEWAY && packet.getMessageType() == TM_MSG_REGISTER)
                    return TM_PACKET_REQUEST;
                return TM_PACKET_REQUEST | TM_PACKET_FORWARD;

            else
                return TM_PACKET_FORWARD;
        }

    }



    void handleRequest(TMPacket *request) {
        switch (request->getMessageType()) {
        case TM_MSG_REGISTER:
            uint8_t data = request->getData()[0];
            if (node_type == TM_NODE_TYPE_GATEWAY) {
                data = request->getData()[0] ? request->getData()[0] : getNewAddress();
                sendPacket(request->getSource(), TM_MSG_OK, &data, 1);
            }
            else
                sendPacket(request->getSource(), TM_MSG_ERR, &data, 1);
            break;
        
        case TM_MSG_COMBINED:
            uint8_t data = 1;
            sendPacket(request->getSource(), TM_MSG_ERR, &data, 1);
            break;
        
        case TM_MSG_PING:
            uint8_t data = request->getData()[0];
            sendPacket(request->getSource(), TM_MSG_OK, &data, 1);

        case TM_MSG_STATUS:
        case TM_MSG_REQUEST:
        case TM_MSG_CUSTOM:
            customRequestHandler(TMPacket *request);
            break;
        default:
            break;
        }
    }

    /** @brief 
     * 
     * @param request 
     * @param response 
     * @return 
     */
    bool handleResponse(TMPacket *request, TMPacket *response) {
        bool ret = false;
        switch (request->getMessageType()) {
            case TM_MSG_COMBINED:
                ret = false;

            case TM_MSG_REGISTER:
                if (response->getMessageType() != TM_MSG_OK || response->getDataLength() != 1) {
                    ret = false;
                    break;
                }
                
                gateway = response->getSource();
                address = response->getData()[0];
                ret = true;

            case TM_MSG_PING:
            case TM_MSG_STATUS:
            case TM_MSG_REQUEST:
            case TM_MSG_CUSTOM:
                ret = customResponseHandler(request, response);

            default: ret = false;
        }

        //since it is answered, clear the request and decrease queue length
        request->clear();
        queue_len--;

        return ret;
    }




    /** @brief Check if packet is a response to the packet being currently sent.
     * 
     * @param response Response packet
     * @return 
     */
    bool isResponse(TMPacket *request, TMPacket *response) {
        //response source == request destination OR request was register AND broadcast
        // AND
        //response destination == response source
        // AND
        //response is OK OR ERR
        return ((response->getSource() == request->getDestination() || (request->getDestination() == TM_BROADCAST_ADDRESS && request->getMessageType() == TM_MSG_REGISTER)) &&
                 response->getDestination() == request->getSource() &&
                (response->getMessageType() == TM_MSG_OK || response->getMessageType() == TM_MSG_ERR));
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

    uint8_t loop(TMPacketID *packet_id) {
        receivePacket();

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
                return TMM_TIMEOUT;
            }

            last_ret = if_manager->sendData(send_queue[curr_index].raw, send_queue[curr_index].size());
            if (last_ret)
                return TMM_LOOP_SEND_DATA;
            
            send_queue[curr_index].setRepeatCount(send_queue[curr_index].getRepeatCount() + 1);
            repeat_timer == 1000;
            return TMM_SENT;
        }
        return TMM_AWAIT;
        //timeout test
            //send data
            //save id
            //increase repeat cnt
        //queue_len-- on response or timeout
    }
};
