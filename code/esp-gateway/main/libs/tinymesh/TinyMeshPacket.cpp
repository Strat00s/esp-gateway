/** @file tinyMesh.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief TinyMesh is a simple protocol for IoT devices.
 * @version 2.0
 * @date 27-11-2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "TinyMeshPacket.hpp"


void TMPacket::setRepeatCount(uint8_t repeat) {
    if (repeat > 3)
        repeat = 3;
    setBits(&raw[TM_FLAGS_POS], repeat, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
}

void TMPacket::setNodeType(uint8_t node_type) {
    if (node_type > 1)
        node_type = 0;
    setBits(&raw[TM_FLAGS_POS], node_type, TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB);
}

void TMPacket::setMessageType(uint8_t msg_type) {
    if (msg_type > 15)
        msg_type = 15;
    setBits(&raw[TM_FLAGS_POS], msg_type, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    setBits(&raw[TM_FLAGS_POS], (msg_type == TM_MSG_OK || msg_type == TM_MSG_ERR), TM_IS_RESP_MSB, TM_IS_RESP_LSB);
}

void TMPacket::setDataLength(uint8_t length) {
    if (length > TM_DATA_LENGTH)
        length = TM_DATA_LENGTH;
    raw[TM_DATA_LEN_POS] = length;
}

/** @brief Copies specified data into the packet.
 *
 * @param data Buffer with data to be written to the packet.
 * @param len Length of the data.
 * @return Number of copied bytes, which can be smaller than len.
 */
uint8_t TMPacket::setData(uint8_t *data, uint8_t len) {
    if (data == nullptr)
        return 0;

    if (len > TM_DATA_LENGTH)
        len = TM_DATA_LENGTH;
    raw[TM_DATA_LEN_POS] = len;
    memcpy(raw + TM_DATA_POS, data, len);
    return len;
}


/** @brief Copies packet data to buffer.
 *
 * @param buffer Buffer to which to copy the data.
 * @param len Length of the buffer.
 * @return Number of copied bytes.
 */
uint8_t TMPacket::getData(uint8_t *buffer, uint8_t len) {
    if (buffer == nullptr)
        return 0;

    if (len > raw[TM_DATA_LEN_POS])
        len = raw[TM_DATA_LEN_POS];
    memcpy(buffer, raw + TM_DATA_POS, len);
    return len;
}


/** @brief Build packet from specified data.
 * Runs checkHeader() at the end.
 *
 * @param source Source node address
 * @param destination Destination node address
 * @param seq Packet sequencing number
 * @param node_type Source node type
 * @param message_type Message type
 * @param repeat_cnt Current repeat count (0-3)
 * @param data Data which to send
 * @param length Length of data
 * @return TM_OK on succes, TM_ERR_... macros on error
 */
uint8_t TMPacket::buildPacket(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t node_type,
                              uint8_t message_type, uint8_t repeat_cnt, uint8_t *data, uint8_t length) {

    // data are null, but some are to be copied -> don't copy anything
    if (data == nullptr && length != 0)
        return TM_ERR_DATA_NULL;

    uint8_t ret = TM_OK;

    if (repeat_cnt > 3)
        repeat_cnt = 3;
    if (node_type > 3)
        node_type = 3;
    if (message_type > 15)
        message_type = 15;

    // build header
    setVersion(TM_VERSION);
    setSource(source);
    setDestination(destination);
    setSequence(sequence);
    setFlags(repeat_cnt << 6 | node_type);
    setMessageType(message_type);
    setDataLength(length);

    // check if header is valid
    ret = checkHeader();

    // return now if there are no data to copy
    if (!length)
        return ret;

    if (setData(data, length) != length)
        return ret | TM_ERR_DATA_TRIM;

    return ret;
};

/** @brief Check if stored packet has valid header and header data.
 *
 * @param packet Packet to check
 * @return TM_OK on succes, TM_ERR_... macros on error
 */
uint8_t TMPacket::checkHeader() {
    uint8_t ret = TM_OK;

    // unsuported version
    if (getVersion() != TM_VERSION)
        ret |= TM_ERR_VERSION;

    if (getSource() == TM_BROADCAST_ADDRESS || getSequence() == getDestination())
        ret |= TM_ERR_ADDRESS;

    // data too long
    if (getDataLength() > TM_DATA_LENGTH)
        ret |= TM_ERR_DATA_LEN;

    switch (getMessageType()) {
    case TM_MSG_OK:
        if (getDestination() == TM_BROADCAST_ADDRESS)
            ret |= TM_ERR_MSG_TYPE;
        break;

    case TM_MSG_ERR:
        if (getDestination() == TM_BROADCAST_ADDRESS)
            ret |= TM_ERR_MSG_TYPE;
    case TM_MSG_PING:
        if (getDataLength() != 1)
            ret |= TM_ERR_MSG_DATA_LEN;
        break;

    case TM_MSG_CUSTOM:
        break;

    default:
        if (getMessageType() > 15)
            ret |= TM_ERR_MSG_TYPE;
        break;
    }

    return ret;
};

