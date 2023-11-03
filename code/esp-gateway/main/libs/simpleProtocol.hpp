#pragma once
#include <stdio.h>


#define SP_HEADER_LENGTH 7
#define SP_DATA_LENGTH   256 - SP_HEADER_LENGTH

#define SP_VERSION 1

#define SP_ERR_INVALID_VERSION     1
#define SP_ERR_INVALID_MSG_TYPE    2
#define SP_ERR_INVALID_MSG_ID      3
#define SP_ERR_DATA_TOO_LONG       4
#define SP_ERR_INVALID_PACKET_SIZE 5
#define SP_ERR_NULL                6

#define SP_VERSION_POS  0
#define SP_MSG_ID_POS   1
#define SP_SRC_ADDR_POS 3
#define SP_DST_ADDR_POS 4
#define SP_SMG_TYPE_POS 5
#define SP_DATA_LEN_POS 6


class simpleProtocol {
private:
    //uint8_t version              = 0;
    //uint16_t msg_id              = 0;   //cannot be zero
    //uint8_t src_address          = 0;
    //uint8_t dst_address          = 0;
    //uint8_t msg_type             = 0;
    //uint8_t data_length          = 0;
    //uint8_t data[SP_HEADER_LENGTH + SP_DATA_LENGTH] = {0};

    uint16_t lcg(uint16_t seed);

    union {
        struct {
            uint8_t version;
            uint8_t msg_id_msb;
            uint8_t msg_id_lsb;
            uint8_t src_address;
            uint8_t dst_address;
            uint8_t msg_type;
            uint8_t data_length;
            uint8_t data[SP_DATA_LENGTH];
        } fields;
        uint8_t raw[SP_HEADER_LENGTH + SP_DATA_LENGTH];
    } packet;
    

public:
    simpleProtocol(/* args */);
    simpleProtocol(uint8_t version);
    simpleProtocol(uint8_t version, uint8_t src_address);
    ~simpleProtocol();

    //write to eeprom callback
    //read from eeprom callback


    void setVersion(uint8_t version)            {this->packet.fields.version = version;}
    void setMessageId(uint16_t id);
    void setSourceAddress(uint8_t address)      {this->packet.fields.src_address = address;}
    void setDestinationAddress(uint8_t address) {this->packet.fields.dst_address = address;}
    void setMessageType(uint8_t type)           {this->packet.fields.msg_type = type;}
    uint8_t setData(uint8_t *data, uint8_t length);

    uint8_t getVersion()            {return this->packet.fields.version;}
    uint16_t getMessageId()         {return ((uint16_t)(this->packet.fields.msg_id_msb)) << 8 | (uint16_t)(this->packet.fields.msg_id_lsb);}
    uint8_t getSourceAddress()      {return this->packet.fields.src_address;}
    uint8_t getDestinationAddress() {return this->packet.fields.dst_address;}
    uint8_t getMessageType()        {return this->packet.fields.msg_type;}
    uint8_t getDataLength()         {return this->packet.fields.data_length;}
    uint8_t *getData()              {return this->packet.fields.data;}
    uint8_t *getRawPacket()         {return this->packet.raw;}


    /** @brief Read packet (raw data), validate and save header, save data.
     * 
     * @param  
     */
    uint8_t readPacket(uint8_t *raw, uint8_t length, bool force = false);

    /** @brief Builds packet from header and data. Uses internal data array which is 256 bytes long.
     * When setting/getting data, those are writen into this array after 56th byte (header length).
     * This means that you can change data after building the packet. But if any header data are
     * changed, you need to rebuild the packet again
     * 
     * @return Pointer to raw packet of data_length + SP_HEADER_LENGTH
     */
    uint8_t *buildPacket();
    uint8_t *buildPacket(uint8_t msg_id, uint8_t dst, uint8_t msg_type);
    uint8_t *buildPacket(uint8_t msg_id, uint8_t dst, uint8_t msg_type, uint8_t *data, uint8_t length);

    /** @brief Generate apropriate answer to received packet if possible. Used when custom handling functions not implemented
     * 
     */
    uint8_t *answer(uint8_t *data, uint8_t length);
};
