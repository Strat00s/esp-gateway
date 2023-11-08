#pragma once
#include <stdio.h>


#define SP_VERSION 1

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

#define SP_MSG_DATA_NONE   0
#define SP_MSG_DATA_INT8   1
#define SP_MSG_DATA_INT16  2
#define SP_MSG_DATA_INT32  3
#define SP_MSG_DATA_STR    4
#define SP_MSG_DATA_CUSTOM 5


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


 VER DEV MIM MIL SAR DAR PRT M_T LEN DATA
|   |   |   |   |   |   |   |   |   |...

# Reserved fields:
    ADDRESS 0 - for devices with no address. Only gateways should answer to these
    ADDRESS 255 - broadcast. All devices should listen on this address, but only gateway should answer
    PORT: 0 - all predefined message types other than DATA



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
