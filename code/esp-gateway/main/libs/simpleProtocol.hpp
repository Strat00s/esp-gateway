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

#define SP_MSG_PING          0
#define SP_MSG_REGISTER      1
#define SP_MSG_DEVICE_CONFIG 2
#define SP_MSG_DATA          3
#define SP_MSG_REQUEST       4
#define SP_MSG_RESEND        5

#define SP_TYPE_GATEWAY 0
#define SP_TYPE_NODE    1
#define SP_TYPE_LP_NODE 2


#define SP_HEADER_LENGTH 8
#define SP_DATA_LENGTH   256 - SP_HEADER_LENGTH

#define SP_VERSION_POS  0
#define SP_DEVICE_TYPE  1
#define SP_MSG_ID_POS   2
#define SP_SRC_ADDR_POS 4
#define SP_DST_ADDR_POS 5
#define SP_MSG_TYPE_POS 6
#define SP_DATA_LEN_POS 7



/*
When sending a new message, message ID must be randomly generated (or at least last used ID cannot be used)
When responding to a message, increase message ID and dela with possible overflows


ADDR 0 -> reserved for new nodes
ADDR 255 -> broadcasted to all nodes

### Message types
# Check if device is in network
request:  ping (uint32_t timestamp)
response: OK (timestamp back)


# Adding new device
request:  REGISTER
resposne: DEVICE_CONFIG (device ID, secret key), ERR (8bit err code)
response: OK, ERR(8bit err code)


# Sending data:
request:  send (data length, data)
response  OK, ERR (8bit err code)


# Requesting data:
request:  data request (16bit data id)
response: send (data length, data)
response: OK, ERR (8bit err code)


# Device types
0 Gateway
1 Node
2 LP Node //is not expected to response


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


    union {
        struct {
            uint8_t version;
            uint8_t device_type;
            uint8_t msg_id_msb;
            uint8_t msg_id_lsb; //cannot be zero, must differ from last sent message
            uint8_t source_addr;
            uint8_t dest_addr;
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
