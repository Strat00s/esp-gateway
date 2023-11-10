#pragma once
#include <stdio.h>


#define SP_VERSION 1


// VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
/*
# Packet structure
------------HEADER------------
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


# Predefined messages packet structure
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
    Device config
        VERSION             1
        DEVICE TYPE         G
        MESSAGE ID          n+1
        SOURCE ADDRESS      GATREWAY_ADDRESS
        DESTINATION ADDRESS 0
        PORT NUMBER         0
        MESSAGE TYPE        CONFIG
        DATA LENGTH         1
        DATA...             DEVICE_ADDRESS
    Data
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         p
        MESSAGE TYPE        DATA
        DATA LENGTH         l
        DATA...             CUSTOM (l bytes)
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
        DESTINATION ADDRESS z
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
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        ROUTE_ANOUNCEMENT
        DATA LENGTH         2 + l
        DATA...             LISTENER_ADDRESS, PORT and l extra PORTs
            LISTENER_ADDRESS 8b
            LISTEN_PORT      8b
            EXTRA_PORTS      8b * l
    Combined
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n
        SOURCE ADDRESS      y
        DESTINATION ADDRESS z
        PORT NUMBER         0
        MESSAGE TYPE        COMBINED
        DATA LENGTH         ?
        DATA...             shortened packets
            PORT 8b
            TYPE 8b
            LEN  8b (l)
            DATA 8b * l
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
    ok
        VERSION             1
        DEVICE TYPE         x
        MESSAGE ID          n+1
        SOURCE ADDRESS      z
        DESTINATION ADDRESS y
        PORT NUMBER         0
        MESSAGE TYPE        OK
        DATA LENGTH         0
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
*/


#define SP_OK                      0
#define SP_ERR_INVALID_VERSION     1
#define SP_ERR_INVALID_MSG_TYPE    2
#define SP_ERR_INVALID_MSG_ID      3
#define SP_ERR_DATA_TOO_LONG       4
#define SP_ERR_INVALID_PACKET_SIZE 5
#define SP_ERR_NULL                6
#define SP_ERR_INVALID_DEVICE_TYPE 7
#define SP_ERR_INVALID_MSG_TYPE    8
#define SP_ERR_PACKET_CHECK_FAILED 9

#define SP_MSG_PING          0  //ping device
#define SP_MSG_REGISTER      1  //register to the newtwork
#define SP_MSG_DEVICE_CONFIG 2  //send device configuration back
#define SP_MSG_DATA          3  //send custom data (to some port)
#define SP_MSG_REQUEST       4
#define SP_MSG_RESEND        5
#define SP_MSG_PORT_ADVERT   6  //advertise custom port for listening/accepting data on
//...|ADVERT|x|PORT_TYPE_PAIR|...
#define SP_MSG_ROUTE_SOLICIT 7  //when user manually routes ports and addresses, send this to output node
//...|SOLIC|x|ADDRESS|PORT|PORT|...
#define SP_MSG_ROUTE_ANOUNC  8  //anounc already known routes
//...|R_ANOUNC|x|ADDRESS|PORT|PORT|...
#define SP_MSG_COMBINED      9  //data contain multiple messages in format |TYPE|LEN|DATA|TYPE...
#define SP_MSG_RESET         10 //
#define SP_MSG_STATUS        11 //RAW string
#define SP_MSG_OK            12
#define SP_MSG_ERR           13

#define SP_PORT_DATA_NONE   0
#define SP_PORT_DATA_INT8   1
#define SP_PORT_DATA_INT16  2
#define SP_PORT_DATA_INT32  3
#define SP_PORT_DATA_STR    4
#define SP_PORT_DATA_CUSTOM 5

#define SP_PORT_IN      0b00000000;
#define SP_PORT_OUT     0b01000000;
#define SP_PORT_INOUT   0b10000000;


#define SP_TYPE_GATEWAY 0
#define SP_TYPE_NODE    1
#define SP_TYPE_LP_NODE 2


#define SP_HEADER_LENGTH 9
#define SP_DATA_LENGTH   256 - SP_HEADER_LENGTH

#define SP_VERSION_POS  0
#define SP_DEVICE_TYPE  1
#define SP_MSG_ID_POS   2
#define SP_SRC_ADDR_POS 4
#define SP_DST_ADDR_POS 5
#define SP_PROTOCOL_POS 6
#define SP_MSG_TYPE_POS 7
#define SP_DATA_LEN_POS 8



/*
When sending a new message, message ID must be randomly generated (or at least last used ID cannot be used)
When responding to a message, increase message ID and dela with possible overflows
If node has adrress 0, it must listen on address 255

Any message not using port 0, gateways must rebroadcast
If message with n+1 msg_id is received before broadcasting, scrap the rebroadcast


# No gateway
Send registration
No response -> probably no gateway
Address is kept 0
Listen on address 255 for all your ports
    //if port 0 and message id == last sent message id + 1 -> listen to message (probably a new gateway)
Start normal operation (sending data to ports) with address 255 (broadcast)
Wait for possible response, but none is expected

### Gateway is added
Send registration
No response -> probably no gateway
Set own address (to anything but 0 and 255)
If any message is intercepted, answer on port 0 with combined message and id of n+1 (so normal answer)
    ...COMBINED|7+x|ERR,1,NETWORK_CHANGE|CFG,2,DID,KEY|...

# Gateway exists
Send registration
Get address (and key)
listen on all own ports
start normal operation
register ports 
    if ports are output, expect gateway to send you port solicitation when manully routed
anounce already registered ports



# Reserved fields:
    ADDRESS 0 - for devices with no address. Only gateways should answer to these
    ADDRESS 255 - broadcast. All devices should listen on this address, but only gateway should answer
    PORT: 0 - all predefined message types other than DATA

if (addr == 0)
    listen to all addresses


ADDR 0 -> reserved for new nodes
ADDR 255 -> broadcasted to all nodes

### Message types
# Check if device is in network
request:  ping (uint32_t timestamp)
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |   n   |SID|RID| 0 |PNG| 0 |
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID

response: OK (timestamp back)
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |  n+1  |DID|SID| 0 | OK| 0 |


# Adding new device
request:  REGISTER
     VER DEV MIM MIL SAR DAR PRT M_T LEN
    | 1 | x |   n   | 0 |255| 0 |REG| 0 |
    x - depends on device
    n - randomly generated other than 0

resposne: DEVICE_CONFIG (device ID, secret key), ERR (8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | G |  n+1  |GID| 0 | 0 |CFG| 2 |DID|KEY|
    | 1 | G |  n+1  |GID| 0 | 0 |ERR| 1 |ERR_CODE|
    G - gateway specific fields
    GID - gateway address/ID
    DID - new device address/ID
    KEY - secret key for usage on network
    ERR - error code

response: OK, ERR(8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |  n+2  |DID|GID| 0 | OK| 0 |
    | 1 | x |  n+2  |DID|GID| 0 |ERR| 1 |ERR|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID


# Sending data:
request:  send (data length, data)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |   n   |SID|DID| y |DTA| z |DATA|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    DID - destination address/ID
    y - service port/ID
    z - data length

    If DID == 255, node automatically registers this port on gateway and user must manually router it to the desired output node (using mqtt)


response  OK, ERR (8bit err code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA...
    | 1 | x |  n+1  |DID|SID| 0 | OK| 0 |
    | 1 | x |  n+1  |DID|SID| 0 |ERR| 1 |ERR_CODE|


# Requesting data:
request:  data request (16bit data id)
response: send (data length, data)
response: OK, ERR (8bit err code)


# port advertisement:
request: port advertisement (port, data_type pairs)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
    | 1 | x |   n   |SID|255| 0 |PRT| z |PORT|TYPE|...|
    x - depends on device
    n - randomly generated other than 0
    SID - source address/ID
    z - data length in multiples of 2

response: OK, ERR (err_code)
     VER DEV MIM MIL SAR DAR PRT M_T LEN DATA...
    | 1 | G |  n+1  |GID|SID| 0 | OK| 0 |
    | 1 | G |  n+1  |GID|SID| 0 |ERR| 1 |ERR_CODE|


# Device types
0 Gateway
1 Node
2 LP Node //is not expected to response



version          -> protocol version
device type      -> idk
msg_id           -> distinguish between repeated messages
source addr      -> to whom data belong, to whom to answser
destination addr -> to whom to send the data, from whom to expect and aswer
port             -> what "service" are we interested in from destination, which service are we supposed to provide
message type     -> prespecified messages


address + port -> define operations and data destination

*/


class simpleProtocol {
private:

    uint16_t lcg(uint16_t seed);

    union {
        uint8_t all = 0;
        struct {
            uint8_t wrong_version  :1;
            uint8_t wrong_dev_type :1;
            uint8_t wrong_msg_type :1;
            uint8_t data_too_long  :1;
            uint8_t data_truncated :1;
            uint8_t none1          :1;
            uint8_t none2          :1;
            uint8_t none3          :1;
        } single;
    } flags;


// VER DEV MIM MIL SAR DAR PRT M_T LEN
//|   |   |   |   |   |   |   |   |   |...
    union {
        struct {
            uint8_t version;
            uint8_t device_type;
            uint8_t msg_id_msb;
            uint8_t msg_id_lsb; //cannot be zero, must differ from last sent message
            uint8_t source_addr;
            uint8_t dest_addr;
            uint8_t port;
            uint8_t msg_type;
            uint8_t data_length;
            uint8_t data[SP_DATA_LENGTH];
        } fields;
        uint8_t raw[SP_HEADER_LENGTH + SP_DATA_LENGTH];
    } packet;
    

public:
    simpleProtocol(uint8_t version);
    simpleProtocol(uint8_t version, uint8_t source_addr);
    ~simpleProtocol();

    //write to eeprom callback
    //read from eeprom callback


    void setVersion(uint8_t version)            {this->packet.fields.version = version;}
    void setDeviceType(uint8_t device_type)     {this->packet.fields.device_type = device_type;}
    void setMessageId(uint16_t id);
    void setSourceAddress(uint8_t address)      {this->packet.fields.source_addr = address;}
    void setDestinationAddress(uint8_t address) {this->packet.fields.dest_addr = address;}
    void setPort(uint8_t port)                  {this->packet.fields.port = port;}
    void setMessageType(uint8_t type)           {this->packet.fields.msg_type = type;}
    uint8_t setData(uint8_t *data, uint8_t length);

    uint8_t getVersion()            {return this->packet.fields.version;}
    uint8_t getDeviceType()         {return this->packet.fields.device_type;}
    uint16_t getMessageId()         {return ((uint16_t)(this->packet.fields.msg_id_msb)) << 8 | (uint16_t)(this->packet.fields.msg_id_lsb);}
    uint8_t getSourceAddress()      {return this->packet.fields.source_addr;}
    uint8_t getDestinationAddress() {return this->packet.fields.dest_addr;}
    uint8_t getMessageType()        {return this->packet.fields.msg_type;}
    uint8_t getDataLength()         {return this->packet.fields.data_length;}
    uint8_t *getData()              {return this->packet.fields.data;}
    uint8_t *getRawPacket()         {return this->packet.raw;}


    /** @brief Check if stored packet has valid header and header data.
     * Sets flags when field contain invalid values
     * 
     * @return SP_OK on succes, SP_ERR_PACKET_CHECK_FAILED on error
     */
    uint8_t checkPacket();

    /** @brief Read packet (raw data), validate and save header and data.
     * Runs checkPacket();
     * Sets flag data_truncated when raw data are shroted than advertised packet length
     * 
     * @param  
     */
    uint8_t readPacket(uint8_t *raw, uint8_t length, bool force = false);

    /** @brief Generate apropriate answer to received packet if possible. Used when custom handling functions not implemented
     * 
     */
    //uint8_t *answer(uint8_t *data, uint8_t length);

    void clearFlags();
};
