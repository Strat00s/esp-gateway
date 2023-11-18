#include "tinyMesh.hpp"
#include <string.h>


TinyMesh::TinyMesh(uint8_t node_type, uint16_t seed) {
    setDeviceType(node_type);
    addPort(0, TM_PORT_INOUT);
}

TinyMesh::TinyMesh(uint8_t address, uint8_t node_type, uint16_t seed) : TinyMesh(node_type, seed) {
    setAddress(address);
}

TinyMesh::TinyMesh(uint8_t version, uint8_t address, uint8_t node_type, uint16_t seed) : TinyMesh(address, node_type, seed) {
    setVersion(version);
}

TinyMesh::~TinyMesh() {

}


uint16_t TinyMesh::lcg(uint16_t seed = 0) {
    static uint16_t x = seed;
    uint16_t a = 25214903917 & 0xFFFF;
    uint16_t c = 11;
    x = (a * x + c) % (0xFFFF + 1);
    return x;
}


void TinyMesh::setVersion(uint8_t version) {
    if (version > TM_VERSION)
        this->version = TM_VERSION;
    else
        this->version = version;
}

void TinyMesh::setAddress(uint8_t address) {
    if (address == 255)
        this->address = TM_DEFAULT_ADDRESS;
    else
        this->address = address;
}

void TinyMesh::setDeviceType(uint8_t node_type) {
    if (node_type > TM_TYPE_LP_NODE)
        this->node_type = TM_TYPE_NODE;
    else
        this->node_type = node_type;
}

void TinyMesh::setMessageId(packet_t *packet, uint16_t msg_id) {
    packet->fields.msg_id_msb = msg_id >> 8;
    packet->fields.msg_id_lsb = msg_id & 0xFF;
}

uint8_t TinyMesh::getVersion() {
    return this->version;
}

uint8_t TinyMesh::getAddress() {
    return this->address;
}

uint8_t TinyMesh::getDeviceType() {
    return this->address;
}

uint16_t TinyMesh::getMessageId(packet_t packet) {
    return (((uint16_t)packet.fields.msg_id_msb) << 8) | ((uint16_t)packet.fields.msg_id_lsb);
}


uint8_t TinyMesh::addPort(uint8_t port, uint8_t type) {
    if (this->port_cnt >= TM_PORT_COUNT)
        return TM_ERR_PORT_COUNT;
    
    this->ports[this->port_cnt] = {port, type};
    this->port_cnt++;

    return TM_OK;
}


uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr || buffer == nullptr)
        return TM_ERR_NULL;

    //at least header size
    if (length < TM_HEADER_LENGTH)
        return TM_ERR_HEADER_LEN;

    //copy fields
    memcpy(packet->raw, buffer, TM_HEADER_LENGTH);

    //check packet
    ret |= checkHeader(*packet);

    //data length <= max data length
    if (!(ret & TM_ERR_DATA_LEN)) {
        //input buffer length more than packet data length
        if (length - TM_HEADER_LENGTH > packet->fields.data_len) {
            ret |= TM_ERR_TRUNCATED;
            memcpy(packet->fields.data, buffer, packet->fields.data_len);
        }
        //input buffer length less than packet data length
        if (length - TM_HEADER_LENGTH < packet->fields.data_len) {
            ret |= TM_ERR_DATA_LEN;
            memcpy(packet->fields.data, buffer, length - TM_HEADER_LENGTH);
        }
        else
            memcpy(packet->fields.data, buffer, packet->fields.data_len);
    }
    //data length > max data length -> truncate
    else {
        ret |= TM_ERR_TRUNCATED;
        memcpy(packet->fields.data, buffer, TM_DATA_LENGTH);
    }

    return ret;
}

uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint8_t ret = buildPacket(packet, destination, message_type, lcg(), port, buffer, length);
    return ret;
}

uint16_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint16_t msg_id, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint16_t ret = TM_OK;

    if (packet == nullptr)
        return TM_ERR_NULL;

    //build packet
    packet->fields.version   = this->version;
    packet->fields.node_type = this->node_type;
    setMessageId(packet, msg_id);
    packet->fields.src_addr  = this->address;
    packet->fields.dst_addr  = destination;
    packet->fields.port      = port;
    packet->fields.msg_type  = message_type;
    packet->fields.data_len  = length;

    //check if header is valid
    ret |= checkHeader(*packet);

    //no data, but length is given -> don't copy anything
    if (length != 0 && buffer == nullptr)
        ret |= TM_ERR_NULL;
    //data are given
    else if (buffer != nullptr) {
        //valid length -> copy all data
        if (!(ret & TM_ERR_DATA_LEN))
            memcpy(packet->fields.data, buffer, length);
        //length > max -> truncate
        else {
            memcpy(packet->fields.data, buffer, TM_DATA_LENGTH);
            ret |= TM_ERR_TRUNCATED;
        }
    }

    return ret;
}


uint16_t TinyMesh::checkHeader(packet_t packet) {
    uint16_t ret = TM_OK;

    //unsuported version
    if (packet.fields.version > TM_VERSION)
        ret |= TM_ERR_VERSION;

    //unknown device type
    if (packet.fields.node_type > TM_TYPE_LP_NODE)
        ret |= TM_ERR_DEVICE_TYPE;

    //invalid message id
    //if ()
    //    ret |= TM_ERR_MSG_ID;

    //invalid source address
    if (packet.fields.src_addr == TM_BROADCAST_ADDRESS)
        ret |= TM_ERR_SOURCE_ADDR;

    //unknown message type
    if (packet.fields.msg_type > TM_MSG_CUSTOM)
        ret |= TM_ERR_MSG_TYPE;

    //custom and port 0, predefined and port other than 0
    if ((packet.fields.msg_type == TM_MSG_CUSTOM && packet.fields.port == TM_DEFAULT_PORT) ||
        (packet.fields.msg_type <  TM_MSG_CUSTOM && packet.fields.port != TM_DEFAULT_PORT))
        ret |= TM_ERR_MSG_TYPE_PORT;

    //data too long
    if (packet.fields.data_len > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LEN;

    //message type, address and data length invalid combinations
    switch (packet.fields.msg_type) {
        case TM_MSG_REGISTER:
            if (packet.fields.src_addr != 0 || packet.fields.dst_addr != 255)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            [[fallthrough]];
        case TM_MSG_PING:
        case TM_MSG_RESET:
            if (packet.fields.data_len != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;

        case TM_MSG_ERR:
            if (packet.fields.data_len != 1)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;

        case TM_MSG_PORT_ADVERT:
            if (packet.fields.data_len < 2 || packet.fields.data_len % 2 != 0)
                ret |= TM_ERR_MSG_TYPE_LEN;
            if (packet.fields.dst_addr != this->gateway_address)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            break;

        case TM_MSG_ROUTE_ANOUNC:
            if (packet.fields.dst_addr != this->gateway_address)
                ret |= TM_ERR_MSG_TYPE_ADDRESS;
            [[fallthrough]];
        case TM_MSG_ROUTE_SOLICIT:
            if (packet.fields.data_len < 2)
                ret |= TM_ERR_MSG_TYPE_LEN;
            break;
        default: break;
    }

    return ret;
}

uint32_t TinyMesh::createPacketID(uint16_t message_id, uint8_t src_addr, uint8_t dst_addr) {
    return ((uint32_t)message_id) << 16 | ((uint16_t)src_addr) << 8 | dst_addr;
}

void TinyMesh::savePacket(packet_t packet) {
    for (int i = TM_SENT_Q_SIZE - 1; i > 0; i--) {
        sent_packets[i] = sent_packets[i - 1];
    }
    //requests[0] = getMessageId(packet);
    sent_packets[0] = createPacketID(getMessageId(packet), packet.fields.src_addr, packet.fields.dst_addr);
}

void TinyMesh::clearSavedPackets() {
    for (int i = 0; i < TM_SENT_Q_SIZE; i++)
        sent_packets[i] = 0;
}

uint8_t TinyMesh::checkPacket(packet_t packet) {
    //if packet is in sent queue, it is a duplicate packet
    uint32_t packet_id = createPacketID(getMessageId(packet), packet.fields.src_addr, packet.fields.dst_addr);
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        if (packet_id == sent_packets[i])
            return TM_ERR_IN_DUPLICATE;
    }
    
    //user must save this packet manually
    if (packet.fields.dst_addr != this->address) {
        return TM_ERR_IN_FORWARD;
    }

    //check if packet is a response to our previous request
    packet_id = createPacketID(getMessageId(packet) - 1, packet.fields.dst_addr, packet.fields.src_addr);
    for (int i = 0; i < TM_SENT_Q_SIZE; i++) {
        if (packet_id == sent_packets[i]) {
            //check for valid port
            bool has_port = false;
            for (int j = 0; j < TM_PORT_COUNT; j++) {
                if (ports[j].port == packet.fields.port) {
                    has_port = true;
                    break;
                }
            }
            if (!has_port)
                return TM_ERR_IN_PORT;
            
            return packet.fields.msg_type == TM_MSG_OK ? TM_OK : TM_ERR_IN_TYPE;
        }
    }

    //packet is for us but is not a response -> new packet
    //user must save this packet manually
    return TM_OK;
}

