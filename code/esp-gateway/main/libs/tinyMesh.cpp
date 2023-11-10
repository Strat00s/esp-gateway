#include "tinyMesh.hpp"
#include <cstring>

tinyMesh::tinyMesh() {

}

tinyMesh::tinyMesh(uint8_t device_type) {
    setDeviceType(device_type);
}

tinyMesh::tinyMesh(uint8_t address, uint8_t device_type) : tinyMesh(device_type) {
    setAddress(address);
}

tinyMesh::tinyMesh(uint8_t version, uint8_t address, uint8_t device_type) : tinyMesh(address, device_type) {
    setVersion(version);
}

tinyMesh::~tinyMesh() {

}


void tinyMesh::setVersion(uint8_t version) {
    if (version > TM_VERSION)
        this->version = TM_VERSION;
    else
        this->version = version;
}

void tinyMesh::setAddress(uint8_t address) {
    if (address == 255)
        this->address = 0;
    else
        this->address = address;
}

void tinyMesh::setDeviceType(uint8_t device_type) {
    if (device_type != TM_TYPE_GATEWAY && device_type != TM_TYPE_NODE && device_type != TM_TYPE_LP_NODE)
        this->device_type == TM_TYPE_NODE;
    else
        this->device_type == device_type;
}
    
uint8_t tinyMesh::getVersion() {
    return this->version;
}

uint8_t tinyMesh::getAddress() {
    return this->address;
}

uint8_t tinyMesh::getDeviceType() {
    return this->address;
}


uint16_t tinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type) {
    if (packet == nullptr)
        return TM_ERR_NULL;

    //build packet
    packet->fields.version     = this->version;
    packet->fields.device_type = this->device_type;
    //TODO generate message id
    packet->fields.source_addr = this->address;
    packet->fields.dest_addr   = destination;
    packet->fields.port        = 0;
    packet->fields.msg_type    = message_type;
    packet->fields.data_length = 0;

    //check if packet is valid
    return checkPacket(packet);
}

uint16_t tinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port) {

}

uint16_t tinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *data, uint8_t length) {

}



uint8_t tinyMesh::setData(uint8_t *data, uint8_t length) {
    if (data == nullptr)
        return TM_ERR_NULL;
    
    if (length > TM_DATA_LENGTH)
        return TM_ERR_DATA_TOO_LONG;
    
    memcpy(this->packet.fields.data, data, length);
    this->packet.fields.data_length = length;

    return 0;
}

//TODO test
uin168_t tinyMesh::checkPacket(packet_t *packet) {
    uint16_t ret = TM_OK;

    if (packet->fields.version > TM_VERSION)
        ret |= TM_ERR_INVALID_PACKET_VERSION;

    if (packet->fields.device_type > TM_TYPE_LP_NODE)
        ret |= TM_ERR_INVALID_DEVICE_TYPE;
    
    if (packet->fields.msg_id_lsb == 0 && packet->fields.msg_id_msb == 0)
        ret |= TM_ERR_INVALID_MSG_ID;

    //TODO address, port and message type combinations
    if (packet->fields.source_addr == 255)
        ret |= TM_ERR_INVALID_SOURCE_ADDR;
    
    //more than custom packet
    if (packet->fields.msg_type > TM_MSG_CUSTOM)
        ret |= TM_ERR_UNKNOWN_MESSAGE_TYPE;
    
    //custom and port 0
    //predefined and port other than 0
    if (packet->fields.msg_type == TM_MSG_CUSTOM && packet->fields.port == 0 ||
        packet->fields.msg_type <= TM_MSG_COMBINED && packet->fields.port != 0)
        ret |= TM_ERR_INVALID_PORT_MSG_TYPE;

   if (packet->fields.data_length > TM_DATA_LENGTH)
        ret |= TM_ERR_INVALID_DATA_LENGTH;


    //message type, address and data length invalid combinations
    switch (packet->fields.msg_type) {
    case TM_MSG_REGISTER:
        if (packet->fields.source_addr != 0 || packet->fields.dest_addr != 255)
            ret |= TM_ERR_INVALID_ADDRESS_MSG_TYPE;
    case TM_MSG_OK:
    case TM_MSG_PING:
    case TM_MSG_RESET:
        if (packet->fields.data_length != 0)
            ret |= TM_ERR_INVALID_LENGTH_MSG_TYPE;
        break;

    case TM_MSG_DEVICE_CONFIG:
        if (packet->fields.dest_addr != 0)
            ret |= TM_ERR_INVALID_ADDRESS_MSG_TYPE;
    case TM_MSG_ERR:
        if (packet->fields.data_length != 1)
            ret |= TM_ERR_INVALID_LENGTH_MSG_TYPE;
        break;

    case TM_MSG_PORT_ADVERT:
        if (packet->fields.data_length < 2 || packet->fields.data_length % 2 != 0)
            ret |= TM_ERR_INVALID_LENGTH_MSG_TYPE;
        if (packet->fields.dest_addr != this->gateway_address)
            ret |= TM_ERR_INVALID_ADDRESS_MSG_TYPE;
        break;

    case TM_MSG_ROUTE_ANOUNC:
        if (packet->fields.dest_addr != this->gateway_address)
            ret |= TM_ERR_INVALID_ADDRESS_MSG_TYPE;
    case TM_MSG_ROUTE_SOLICIT:
        if (packet->fields.data_length < 2)
            ret |= TM_ERR_INVALID_LENGTH_MSG_TYPE;
        break;
    default: break;
    }

    return ret;
}

uint8_t tinyMesh::readPacket(uint8_t *raw, uint8_t length, bool force) {
    if (raw == nullptr)
        return TM_ERR_NULL;

    if (length < TM_HEADER_LENGTH)
        return TM_ERR_INVALID_PACKET_SIZE;


    if (raw[TM_DATA_LEN_POS] + TM_HEADER_LENGTH > length)
        this->flags.single.data_too_long = 1;

    if (length > raw[TM_DATA_LEN_POS] + TM_HEADER_LENGTH)
        length = raw[TM_DATA_LEN_POS] + TM_HEADER_LENGTH;

    memcpy(this->packet.raw, raw, length);

    return checkPacket();
}


uint16_t lcg(uint16_t seed) {

}

