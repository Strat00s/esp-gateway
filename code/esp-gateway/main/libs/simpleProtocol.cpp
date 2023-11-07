#include "simpleProtocol.hpp"
#include <cstring>

simpleProtocol::simpleProtocol(uint8_t version) {
    this->packet.fields.version = version;
}

simpleProtocol::simpleProtocol(uint8_t version, uint8_t source_addr) : simpleProtocol(version) {
    this->packet.fields.source_addr = source_addr;
}

simpleProtocol::~simpleProtocol() {

}


void simpleProtocol::setMessageId(uint16_t id) {
    this->packet.fields.msg_id_msb = (uint8_t)(id >> 8);
    this->packet.fields.msg_id_lsb = (uint8_t)id;
}


uint8_t simpleProtocol::setData(uint8_t *data, uint8_t length) {
    if (data == nullptr)
        return SP_ERR_NULL;
    
    if (length > SP_DATA_LENGTH)
        return SP_ERR_DATA_TOO_LONG;
    
    memcpy(this->packet.fields.data, data, length);
    this->packet.fields.data_length = length;

    return 0;
}

uint8_t simpleProtocol::checkPacket() {
    uint8_t ret = SP_OK;
    
    if (this->packet.fields.version > SP_VERSION) {
        this->flags.single.wrong_version = 1;
        ret = SP_ERR_PACKET_CHECK_FAILED;
    }

        //set flag
        //return SP_ERR_INVALID_VERSION;

    switch (this->packet.fields.device_type) {
        case SP_TYPE_GATEWAY: break;
        case SP_TYPE_NODE:    break;
        case SP_TYPE_LP_NODE: break;
        default:
            this->flags.single.wrong_dev_type = 1;
            ret = SP_ERR_PACKET_CHECK_FAILED;
            break;
        //set flag return SP_ERR_INVALID_DEVICE_TYPE;
    }

    switch (this->packet.fields.msg_type) {
        case SP_MSG_PING:          break;
        case SP_MSG_REGISTER:      break;
        case SP_MSG_DEVICE_CONFIG: break;
        case SP_MSG_DATA:          break;
        case SP_MSG_REQUEST:       break;
        case SP_MSG_RESEND:        break;
        default:
            this->flags.single.wrong_msg_type = 1;
            ret = SP_ERR_PACKET_CHECK_FAILED;
            break;
    }

    if (this->packet.fields.data_length > SP_DATA_LENGTH) {
        this->flags.single.data_too_long = 1;
        ret = SP_ERR_PACKET_CHECK_FAILED;
    }
    
    return ret;
}

uint8_t simpleProtocol::readPacket(uint8_t *raw, uint8_t length, bool force) {
    if (raw == nullptr)
        return SP_ERR_NULL;

    if (length < SP_HEADER_LENGTH)
        return SP_ERR_INVALID_PACKET_SIZE;


    if (raw[SP_DATA_LEN_POS] + SP_HEADER_LENGTH > length)
        this->flags.single.data_too_long = 1;

    if (length > raw[SP_DATA_LEN_POS] + SP_HEADER_LENGTH)
        length = raw[SP_DATA_LEN_POS] + SP_HEADER_LENGTH;

    memcpy(this->packet.raw, raw, length);

    return checkPacket();
}


uint16_t lcg(uint16_t seed) {

}

