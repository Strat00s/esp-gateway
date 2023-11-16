#pragma once
#include <stdio.h>


//TODO consistency
/*
# Packet structure
-------------HEADER-------------
    VERSION             8b
    DEVICE TYPE         8b
    MESSAGE ID          16b
    SOURCE ADDRESS      8b
    DESTINATION ADDRESS 8b
    PORT NUMBER         8b
    MESSAGE TYPE        8b
    DATA LENGTH         8b
--------------DATA--------------
    DATA...             256 - header


# Rules
All predefined messages have simple flow: single request and a single answer (might not arrive)
    Message ID of an answer (OK, ERR) is equal to message ID of request + 1 
Only custom messages are allowed to have flow of any size (continuous request, response, request, response, ...)
    Message ID is increased by 1 for every new message in this flow.

# Predefined messages packet structure and flow:
    ok
        VERSION             1
        NODE TYPE           x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        OK
        DATA LENGTH         l
        DATA...             l bytes (depends on answered message)
    err
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        ERR
        DATA LENGTH         1
        DATA...             ERROR_CODE
    Ping
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        PING
        DATA LENGTH         0
    Register
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      0
        DESTINATION ADDRESS 255
        PORT NUMBER         0
        MESSAGE TYPE        REGISTER
        DATA LENGTH         0
    //Device config
    //    VERSION             1
    //    DEVICE TYPE         G
    //    MESSAGE ID          n+1
    //    SOURCE ADDRESS      GATREWAY_ADDRESS
    //    DESTINATION ADDRESS 0
    //    PORT NUMBER         0
    //    MESSAGE TYPE        CONFIG
    //    DATA LENGTH         1
    //    DATA...             DEVICE_ADDRESS
    TODO Request
        VERSION
        DEVICE TYPE
        MESSAGE ID
        SOURCE ADDRESS
        DESTINATION ADDRESS
        PORT NUMBER
        MESSAGE TYPE
        DATA LENGTH
        DATA...
    TODO Resend
        VERSION
        DEVICE TYPE
        MESSAGE ID
        SOURCE ADDRESS
        DESTINATION ADDRESS
        PORT NUMBER
        MESSAGE TYPE
        DATA LENGTH
        DATA...
    Port advertisement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        PORT_ADVERTISEMENT
        DATA LENGTH         2 * l
        DATA...             l pairs of PORT and DATA_TYPE
            PORT      8b
            DATA_TYPE 8b
                PORT DIR  0bxx000000
                DATA TYPE 0b00xxxxxx
    Route solicitation
        VERSION             1
        DEVICE TYPE         G
        MESSAGE ID          n
        SOURCE ADDRESS      x
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_SOLICITATION
        DATA LENGTH         2 + l
        DATA...             LISTENER_ADDRESS, PORT and l extra PORTs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            EXTRA_PORTS      8b * l
    Route anouncement
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z (gateway)
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_ANOUNCEMENT
        DATA LENGTH         2 + l
        DATA...             LISTENER_ADDRESS, PORT and l extra PORTs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            EXTRA_PORTS      8b * l
    Reset
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        RESET
        DATA LENGTH         0
    Status
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        STATUS
        DATA LENGTH         l
        DATA...             string of size l
    Combined
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        COMBINED
        DATA LENGTH         l
        DATA...             shortened packets
            PORT 8b
            TYPE 8b
            LEN  8b (l)
            DATA 8b * l
    Custom
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         p
        MESSAGE TYPE        DATA
        DATA LENGTH         l
        DATA...             CUSTOM (l bytes)
*/


#define TM_VERSION 1

//PACKET SIZES
#define TM_HEADER_LENGTH 9
#define TM_DATA_LENGTH   256 - TM_HEADER_LENGTH

//RETURN FLAGS
#define TM_OK                   0b0000000000000000

#define TM_ERR_NULL             0b0000000000000001
#define TM_ERR_PACKET_LEN       0b0000000000000010
#define TM_ERR_TRUNCATED        0b0000000000000100

#define TM_ERR_VERSION          0b0000000000001000
#define TM_ERR_MSG_ID           0b0000000000010000
#define TM_ERR_DEVICE_TYPE      0b0000000000100000
#define TM_ERR_SOURCE_ADDR      0b0000000001000000
#define TM_ERR_DATA_LEN         0b0000000010000000
#define TM_ERR_MSG_TYPE         0b0000000100000000
#define TM_ERR_MSG_TYPE_PORT    0b0000001000000000
#define TM_ERR_MSG_TYPE_ADDRESS 0b0000010000000000
#define TM_ERR_MSG_TYPE_LEN     0b0000100000000000
#define TM_ERR_DATA_NULL        0b0001000000000000

#define TM_ERR_CFG_ADDRESS      0b0010000000000000

/*----(MESSAGE TYPES)----*/
//response
#define TM_MSG_OK            0
#define TM_MSG_ERR           1

//request
#define TM_MSG_PING          2  //ping device
#define TM_MSG_REGISTER      3  //register to the newtwork
//#define TM_MSG_DEVICE_CONFIG 4  //send device configuration back
#define TM_MSG_PORT_ADVERT   4  //advertise custom port for listening/accepting data on
#define TM_MSG_ROUTE_SOLICIT 5  //when user manually routes ports and addresses, send this to output node
#define TM_MSG_ROUTE_ANOUNC  6  //anounc already known routes
#define TM_MSG_RESET         7  //
#define TM_MSG_STATUS        8  //RAW string
#define TM_MSG_COMBINED      9  //data contain multiple messages in format |TYPE|LEN|DATA|TYPE...
#define TM_MSG_CUSTOM        10 //send custom data (to some port)

//PORT DATA TYPES
#define TM_PORT_DATA_NONE   0
#define TM_PORT_DATA_INT8   1
#define TM_PORT_DATA_INT16  2
#define TM_PORT_DATA_INT32  3
#define TM_PORT_DATA_STR    4
#define TM_PORT_DATA_CUSTOM 5

//PORT DIRECTIONS
#define TM_PORT_IN      0b01000000
#define TM_PORT_OUT     0b10000000
#define TM_PORT_INOUT   0b11000000

//NODE TYPES
#define TM_TYPE_GATEWAY 0
#define TM_TYPE_NODE    1
#define TM_TYPE_LP_NODE 2

//ERROR CODES
#define TM_EC_CFG_ADDRESS 1
#define TM_


#define TM_POS_VERSION  0
#define TM_POS_DEV_TYPE 1
#define TM_POS_MSG_ID_M 2
#define TM_POS_MSG_ID_L 3
#define TM_POS_SRC_ADDR 4
#define TM_POS_DST_ADDR 5
#define TM_POS_PORT     6
#define TM_POS_MSG_TYPE 7
#define TM_POS_DATA_LEN 8


//DEFAULT CONFIG
#define TM_DEFAULT_ADDRESS   0
#define TM_BROADCAST_ADDRESS 255
#define TM_DEFAULT_PORT      0

#define PORT_COUNT      10
#define SEND_QUEUE_SIZE 10


typedef union{
    struct {
        uint8_t version;
        uint8_t node_type;
        uint8_t msg_id_msb;
        uint8_t msg_id_lsb; //cannot be zero, must differ from last sent message
        uint8_t src_addr;
        uint8_t dst_addr;
        uint8_t port;
        uint8_t msg_type;
        uint8_t data_len;
        uint8_t data[TM_DATA_LENGTH];
    } fields;
    uint8_t raw[TM_HEADER_LENGTH + TM_DATA_LENGTH];
} packet_t;

//TODO combined
typedef struct {
    uint8_t dst_addr;
    uint8_t port;
    uint8_t msg_type;
    uint8_t length;
    uint8_t data[TM_DATA_LENGTH];
} short_packet_t;

typedef struct {
    uint8_t port;
    uint8_t type;
} port_cfg_t;

class TinyMesh {
private:
    uint8_t version         = TM_VERSION;
    uint8_t address         = 0;
    uint8_t gateway_address = 0;
    uint8_t node_type       = 0;

    uint16_t lcg(uint16_t seed);

public:

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t node_type, uint16_t seed = 42069);

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param address Node address
     * @param node_type Node type
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t address, uint8_t node_type, uint16_t seed = 42069);

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param version Protocol version to be supported
     * @param address Node address
     * @param node_type Node type
     * @param seed Starting seed for LCG
     */
    TinyMesh(uint8_t version, uint8_t address, uint8_t node_type, uint16_t seed = 42069);
    ~TinyMesh();

    /** @brief Set protocol version.
     * Max version is TM_VERSION.
     * @param version 
     */
    void setVersion(uint8_t version);

    /** @brief Set node address.
     * Address 255 will be converted to 0
     * 
     * @param address 
     */
    void setAddress(uint8_t address);
    
    /** @brief Set node type.
     * Unknown node types will be converted to TM_TYPE_NODE.
     * 
     * @param node_type 
     */
    void setDeviceType(uint8_t node_type);

    /** @brief Set message ID for specified packet.
     * 
     * @param packet Packet for which to set message ID
     * @param msg_id New message ID
     */
    void setMessageId(packet_t *packet, uint16_t msg_id);

    uint8_t getVersion();
    uint8_t getAddress();
    uint8_t getDeviceType();

    /** @brief Construct and return the message ID.
     * 
     * @param packet Packet from which to get message ID
     * @return Message ID
     */
    uint16_t getMessageId(packet_t packet);

    /** @brief Build packet from a buffer.
     * 
     * @param packet Packet to which to store header and data
     * @param data Raw packet data
     * @param length Data length
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t *buffer, uint8_t length);

    /** @brief Build packet from specific data.
     * Message ID is generated using builtin LCG.
     * Runs checkHeader() at the end.
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param port Port to which to send data
     * @param data Data which to send
     * @param length Length of data
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port = 0, uint8_t *data = nullptr, uint8_t length = 0);
    
    /** @brief Build packet from specific data.
     * Runs checkHeader() at the end.
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param msg_id Message ID
     * @param port Port to which to send data
     * @param data Data which to send
     * @param length Length of data
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint16_t msg_id, uint8_t port = 0, uint8_t *data = nullptr, uint8_t length = 0);

    /** @brief Check if stored packet has valid header and header data.
     * 
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint16_t checkHeader(packet_t packet);
};
