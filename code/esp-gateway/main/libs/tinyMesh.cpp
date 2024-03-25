/** @file tinyMesh.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief TinyMesh is a simple protocol for IoT devices.
 * @version 2.0
 * @date 27-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "tinyMesh.hpp"
#include <string.h>


uint8_t TinyMesh::buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t repeat_cnt, uint8_t *data, uint8_t length) {
    uint8_t ret = TM_OK;

    if (packet == nullptr)
        return TM_ERR_PACKET_NULL;

    //build packet
    packet->fields.version     = this->version;
    packet->fields.source      = this->address;
    packet->fields.destination = destination;
    packet->fields.data_length = length;
    packet->fields.flags       = repeat_cnt << 6 | message_type << 2 | this->node_type;

    //check if header is valid
    ret |= checkHeader(packet);

    //return now if there are no data to copy
    if (!length)
        return ret;

    //data are null, but some are to be copied -> don't copy anything
    if (data == nullptr && length != 0)
        return ret |= TM_ERR_DATA_NULL;

    if (length > TM_DATA_LENGTH) {
        memcpy(packet->fields.data, data, TM_DATA_LENGTH);
        ret |= TM_ERR_DATA_TRIM;
    }
    else
        memcpy(packet->fields.data, data, length);

    return ret;
}

uint8_t TinyMesh::checkPacket(packet_t *packet) {
    uint8_t ret = TM_OK;

    //unsuported version
    if (packet->fields.version != TM_VERSION)
        ret |= TM_ERR_VERSION;

    if (packet->fields.source == TM_BROADCAST_ADDRESS)
        ret |= TM_ERR_ADDRESS;

    //data too long
    if (packet->fields.data_length > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LENGTH;

    switch (getBits(packet->fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB)) {
        case TM_MSG_OK:     break;
        case TM_MSG_CUSTOM: break;
        case TM_MSG_ERR:
        case TM_MSG_PING:
            if (packet->fields.data_length != 1)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        case TM_MSG_ADDR_REQ:
            if (packet->fields.data_length)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        case TM_MSG_STATUS:
            if (!packet->fields.data_length)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        case TM_MSG_COMBINED:
            if (packet->fields.data_length < 2)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        case TM_MSG_REQUEST:
            if (packet->fields.data_length < 1)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        default:
            ret |= TM_ERR_MSG_TYPE;
            break;
    }

    return ret;
}

packet_id_t TinyMesh::createPacketID(packet_t *packet) {
    packet_id_t packet_id;
    packet_id.source         = packet->fields.source;
    packet_id.destination    = packet->fields.destination;
    packet_id.flags.answered = 0;
    packet_id.flags.repeat   = getBits(packet->fields.flags, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
    packet_id.flags.msg_type = getBits(packet->fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    return packet_id;
}
