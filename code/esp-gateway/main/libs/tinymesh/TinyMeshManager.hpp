#include "TinyMeshPacket.hpp"
#include "TinyMeshPacketID.hpp"
#include "../interfaces/InterfaceManager.hpp"
#include "../containers/StaticDeque.hpp"


#define TMM_QUEUE_SIZE 5
#define TMM_PID_SIZE 10


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

#define TMM_ONGOING_TX        14

#define TMM_REQUEST       0b00010000
#define TMM_RESPONSE      0b00100000
#define TMM_FORWARD       0b01000000
#define TMM_ERR_FORWARD   0b10000000

#define TMM_PACKET_RND_RESPONSE 0b00000010
#define TMM_PACKET_FORWARD      0b00000100
#define TMM_PACKET_REQUEST      0b00001000
#define TMM_PACKET_RESPONSE     0b00010000


typedef struct {
    TMPacket packet;
    unsigned long tts;
} packet_tts_t;


class TinyMeshManager {
private:
    InterfaceManager *if_manager = nullptr;
    
    uint8_t address      = TM_DEFAULT_ADDRESS;
    uint8_t node_type    = TM_NODE_TYPE_NORMAL;
    uint8_t sequence_num = 0;

    TMPacket packet;
    StaticDeque<packet_tts_t, TMM_QUEUE_SIZE> send_queue;
    size_t request_index = 0;
    unsigned long last_loop_time = 0;

    TMPacketID pid_list[TMM_PID_SIZE];
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
    bool isResponse(TMPacket *response);


    /** @brief Create packet ID from packet.
     * 
     * @param packet Packet for which to create ID.
     * @return The packet ID.
     */
    TMPacketID createPacketID(TMPacket *packet);

    /** @brief Set specific packet ID to mirror a packet.
     * 
     * @param pid Packet ID which to set.
     * @param packet Packet which to mirror.
     */
    void setPacketID(TMPacketID *pid, TMPacket *packet);

    /** @brief Save ID of a packet to pid_list.
     * 
     * @param packet 
     * @return True on success.
     * False if packet is a duplicate. 
     */
    bool savePacketID(TMPacket *packet);


    /** @brief Classify packet if it is a response, requiest, to be forwarded and so on.
     * 
     * @return TMM_PACKET_ macros. 
     */
    uint8_t classifyPacket();


    uint8_t handleRequest(TMPacket *request, bool fwd);

    uint8_t sendData(TMPacket *packet, InterfaceWrapper *interface);

    /** @brief Handle incoming packets*/
    uint8_t receivePacket(InterfaceWrapper *interface);


public:
    TinyMeshManager(InterfaceManager *interface_manager = nullptr);
    TinyMeshManager(uint8_t address, InterfaceManager *interface_manager = nullptr);
    TinyMeshManager(uint8_t address, uint8_t node_type, InterfaceManager *interface_manager = nullptr);


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
    void setNodeType(uint8_t node_type);

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
    uint8_t queuePacket(uint8_t destination, uint8_t message_type, uint8_t *data = nullptr, uint8_t length = 0);

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
     * @param TMM_ERR_NULL if both interface or interface manager are null
     */
    uint8_t loop(InterfaceWrapper *interface = nullptr);
};
