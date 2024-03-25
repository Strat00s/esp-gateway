#include "tinyMesh.hpp"
#include "interfaces/InterfaceManager.hpp"

using namespace tinymesh;

template <uint8_t N>
class TinyMeshManager {
private:
    /* data */
    packet_id_t sent_list[10];
    uint8_t save_index = 0;
    InterfaceManagerBase *if_manager = nullptr;
    
    uint8_t address   = TM_DEFAULT_ADDRESS;
    uint8_t gateway   = TM_BROADCAST_ADDRESS;
    uint8_t node_type = TM_NODE_TYPE_NODE;
    packet_t *packet = nullptr;

    uint32_t time;

    uint8_t last_ret;

    void delay(uint32_t ms) {

    }

    uint32_t millis() {
        return 0;
    }

    uint8_t customRequestHandler();
    uint8_t customResponseHandler(packet_id_t request_id, packet_t *response_packet)

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

    TinyMeshManager(InterfaceManagerBase *interface_manager, uint8_t address, uint8_t node_type, uint8_t gateway, packet_t *packet) : TinyMeshManager(interface_manager, address, node_type, gateway) {
        this->packet = packet;
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

    /** @brief Used to register external packet structure.
     * Is required due to the potential memory limitation when using large packets.
     * 
     * @param packet 
     */
    void registerPacket(packet_t *packet) {
        this->packet = packet;
    }

    /** @brief Send packet and wait for response
     * 
     * @param destination 
     * @param message_type 
     * @param repeat_cnt 
     * @param data 
     * @param length 
     * @return 
     */
    uint8_t sendPacket(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0, uint16_t timeout_ms = 1000, uint8_t repeat_cnt = 4) {
        if (packet == nullptr)
            return TMM_TX_NULL_PACKET;
        
        for (uint8_t i = 0; i < repeat_cnt; i++) {
            last_ret = buildPacket(packet, address, destination, node_type, message_type, i, data, length);
            if (last_ret)
                return 1; //TODO;

            last_ret = if_manager->sendData(packet->raw, packet->size());
            if (last_ret)
                return 1; //TODO;

            uint32_t timer = millis();

            savePacketID(createPacketID(packet));

            //wait for answer
            while (millis() - timer < timeout_ms) {
                uint8_t ret = receivePacket();
                
                //got some packet
                if (ret == TMM_RX_OK) {
                    //forward packet first
                    if (last_ret & TM_PACKET_FORWARD)
                        if_manager->sendData(packet->raw, packet->size());

                    //handle request
                    if (last_ret & TM_PACKET_REQUEST)
                        handleRequest();

                    //handle response
                    if (last_ret & TM_PACKET_RESPONSE) {
                        auto request_id = findRequestPacketID(createPacketID(packet));
                        handleResponse(request_id);

                        //succesful registration
                        if (GET_MSG_TYPE(packet) == TM_MSG_OK && request_id.flags.msg_type == TM_MSG_REGISTER)
                            return 0;
                    }
                }
                if (ret == TMM_RX_GET_DATA) {
                    //TODO
                }
                if (ret == TMM_RX_CHECK_PACKET) {
                    //TODO
                }
                if (ret == TMM_RX_WRONG_PACKET) {
                    //TODO
                }
            }
        }
    }

    /** @brief 
     * 
     * @return uint16_t - inner returtn value, return reson + if response or request
     */
    uint8_t receivePacket() {
        if (!if_manager->hasData())
            return TMM_RX_NO_DATA;
        
        uint8_t len = packet->size();
        last_ret = if_manager->getNextData(packet->raw, &len);
        if (last_ret)
            return TMM_RX_GET_DATA;
        
        last_ret = checkPacket(packet);
        if (last_ret)
            return TMM_RX_CHECK_PACKET;
        
        last_ret = classifyPacket();
        updatePacketIDTime();

        if (last_ret & TM_PACKET_DUPLICATE)
            return TMM_RX_WRONG_PACKET;

        savePacketID(createPacketID(packet));

        if (last_ret & TM_PACKET_INV_RESPONSE) {
            return TMM_RX_WRONG_PACKET;
        }

        if (last_ret & TM_PACKET_RND_RESPONSE) {
            return TMM_RX_WRONG_PACKET;
        }

        return TMM_RX_OK;
    }


    uint8_t classifyPacket() {
        uint8_t msg_type = GET_MSG_TYPE(packet->fields.flags);

        packet_id_t packet_id = createPacketID(packet);

        //duplicate or response
        for (uint8_t i = 0; i < N; i++) {
            if (sent_list[i].isEmpty())
                continue;

            //duplicate
            //original source = new source
            //original destination = new destination
            //original message type = new message type
            //original repeat >= new repeat
            if (sent_list[i].source == packet_id.source && sent_list[i].destination == packet_id.destination &&
                sent_list[i].flags.msg_type == packet_id.flags.msg_type && sent_list[i].flags.repeat >= packet_id.flags.repeat)
                return TM_PACKET_DUPLICATE;

            //new response
            //original destination = new source
            //original destination = broadcast
            //original source = new destination
            //message type = OK or ERR
            if ((sent_list[i].destination == packet_id.source || sent_list[i].destination == TM_BROADCAST_ADDRESS) && sent_list[i].source == packet_id.destination) {
                if (msg_type != TM_MSG_OK && msg_type != TM_MSG_ERR)
                    return TM_PACKET_INV_RESPONSE
                return TM_PACKET_RESPONSE;
            }
        }

        //request for us
        if (packet->fields.destination == address) {
            if (msg_type == TM_MSG_OK || msg_type == TM_MSG_ERR)
                return TM_PACKET_RND_RESPONSE;
            else
                return TM_PACKET_REQUEST
        }

        //broadcast
        else if (packet->fields.destination == 255) {
            uint8_t ret |= TM_PACKET_FORWARD
            if (msg_type != TM_MSG_OK && msg_type != TM_MSG_ERR)
                ret |= TM_PACKET_REQUEST;
            return ret;
        }

        //forward
        else
            return TM_PACKET_FORWARD
    }

    uint8_t sendPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t repeat_cnt = 0, uint8_t *data = nullptr, uint8_t length = 0) {
        
    }


    uint8_t join(uint16_t timeout_ms = 1000, uint8_t repeat_cnt = 4) {
        if (node_type == TM_NODE_TYPE_GATEWAY)
            return 1;

        for (uint8_t i = 0; i < repeat_cnt; i++) {
            //send get addr
            buildPacket(packet, address, gateway, node_type, TM_MSG_ADDR_REQ, i);
            if_manager->sendData()
            //wait for response
            //if no response - increase repeat cnt and resend
        }
        
        return 0;
    }



    void handleRequest() {

        //once answer was sent
        //request_id.flags.answered = true;
    }

    bool handleResponse(packet_id_t request_id) {
        request_id.flags.answered = true;
        switch (request_id.flags.msg_type) {
            case TM_MSG_COMBINED: //TODO
                return 3;

            case TM_MSG_REGISTER:
                if (GET_NODE_TYPE(packet->fields.flags) != TM_MSG_OK || packet->fields.data_length != 1)
                    return 1;
                
                gateway = packet->fields.source;
                address = packet->fields.data[0]
                return 0;

            case TM_MSG_PING:
            case TM_MSG_STATUS:
            case TM_MSG_REQUEST:
            case TM_MSG_CUSTOM:
                return customResponseHandler(request_id, packet);

            default: return 2;
        }
    }


    //TODO
    bool findPacketID(const packet_id_t &packet_id) {
        for (uint8_t i = 0; i < N; i++) {
            if (sent_list[i].isEmpty())
                continue;
            
            if (packet_id == sent_list[i])
                return true;
        }

        return false;
    }

    /** @brief Find request packet ID once if we have valid response ID.
     * 
     * @param response_id Valid response ID
     * @return packet_id_t 
     */
    packet_id_t findRequestPacketID(const packet_id_t &response_id) {
        for (uint8_t i = 0; i < N; i++) {
            if (sent_list[i].isEmpty() || sent_list[i].flags.answered)
                continue;
            
            if (sent_list[i].source == response_id.destination && sent_list[i].destination == response_id.source)
                return createPacketID(sent_list[i]);
        }
        return {0};
    }

    void updatePacketIDTime() {
        uint8_t t_passed = (millis() - time) / 500;
        time += t_passed * 500;
        if (!t_passed)
            return;
        
        for (uint8_t i = 0; i < N; i++) {
            if (sent_list[i].isEmpty())
                continue;
            
            //clear stale records
            if (sent_list[i].tts <= t_passed)
                sent_list[i].clear();
            else
                sent_list[i].tts -= t_passed;
        }
    }

    void savePacketID(const packet_id_t &packet_id, uint8_t tts = 6) {
        int save_index = -1;
        uint8_t min_tts = 0;
        
        for (uint8_t i = 0; i < N; i++) {
            if (sent_list[i].isEmpty()) {
                save_index = i;
                continue;
            }

            //update packet id
            if (packet_id.source == sent_list[i].source &&
                packet_id.destination == sent_list[i].destination &&
                packet_id.flags.msg_type == sent_list[i].flags.msg_type &&
                packet_id.flags.repeat > sent_list[i].flags.repeat) {
                sent_list[i] = packet_id;
                return;
            }
        }

        //save new packet id
        sent_list[save_index] = packet_id;
    }
};
