#include "simpleProtocol.hpp"
#include <cstring>

simpleProtocol::simpleProtocol(/* args */) {
}

simpleProtocol::simpleProtocol(uint8_t version) {
    this->packet.fields.version = version;
}

simpleProtocol::simpleProtocol(uint8_t version, uint8_t src_address) : simpleProtocol(version) {
    this->packet.fields.src_address = src_address;
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

uint8_t simpleProtocol::readPacket(uint8_t *raw, uint8_t length, bool force) {
    if (raw == nullptr)
        return SP_ERR_NULL;

    if (length < SP_HEADER_LENGTH)
        return SP_ERR_INVALID_PACKET_SIZE;

    if (length > SP_HEADER_LENGTH + SP_DATA_LENGTH && !force)
        return SP_ERR_INVALID_PACKET_SIZE;

    if (raw[SP_VERSION_POS] > SP_VERSION && !force)
        return SP_ERR_INVALID_VERSION;

    if (raw[SP_MSG_ID_POS] == 0 && !force)
        return SP_ERR_INVALID_MSG_ID;
    
    //TODO
    //if (raw[SP_SMG_TYPE_POS] && !force)

    //TODO check length and correctly assign it
    //packet length vs data length
    if (raw[SP_DATA_LEN_POS] > SP_DATA_LENGTH) {
        if (!force)
            return SP_ERR_DATA_TOO_LONG;

        if (length > raw[SP_DATA_LEN_POS] && length < SP_HEADER_LENGTH + SP_DATA_LENGTH) {
            memcpy(this->packet.raw, raw, length);
            this->packet.fields.data_length = SP_DATA_LENGTH;
        }
        memcpy(this->packet.raw, raw, SP_DATA_LENGTH);
        this->packet.fields.data_length = SP_DATA_LENGTH;
    }
    else
        memcpy(this->packet.raw, raw, length);

    return 0;
}



uint8_t *simpleProtocol::buildPacket() {
    this->data[0] = this->version;
    this->data[1] = (uint8_t)(msg_id >> 8);
    this->data[2] = (uint8_t)msg_id;
    this->data[3] = this->src_address;
    this->data[4] = this->dst_address;
    this->data[5] = this->msg_type;
    this->data[6] = this->data_length;
}
uint8_t *simpleProtocol::buildPacket(uint8_t msg_id, uint8_t dst, uint8_t msg_type) {
    this->msg_id      = msg_id;
    this->dst_address = dst;
    this->msg_type    = msg_type;
    return buildPacket();
}
uint8_t *simpleProtocol::buildPacket(uint8_t msg_id, uint8_t dst, uint8_t msg_type, uint8_t *data, uint8_t length) {
    setData(data, length);
    return buildPacket(msg_id, dst, msg_type);
}



uint16_t lcg(uint16_t seed) {

}

