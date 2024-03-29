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

inline void TMPacket::setVersion(uint8_t version) {
    raw[TM_VERSION_POS] = version;
}

inline void TMPacket::setSource(uint8_t source) {
    raw[TM_SOURCE_POS] = source;
}

inline void TMPacket::setDestination(uint8_t destination) {
    raw[TM_DESTINATION_POS] = destination;
}

inline void TMPacket::setSequence(uint16_t sequence) {
    raw[TM_SEQUENCE_POS] = sequence >> 8;
    raw[TM_SEQUENCE_POS + 1] = sequence;
}

inline void TMPacket::setRepeatCount(uint8_t repeat) {
    if (repeat > 3)
        repeat = 3;
    setBits(&raw[TM_FLAGS_POS], repeat, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
}

inline void TMPacket::setNodeType(uint8_t node_type) {
    if (node_type > 3)
        node_type = 3;
    setBits(&raw[TM_FLAGS_POS], node_type, TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB);
}

inline void TMPacket::setMessageType(uint8_t msg_type) {
    if (msg_type > 15)
        msg_type = 15;
    setBits(&raw[TM_FLAGS_POS], msg_type, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
}

/** @brief Set entire flag field at once
 *
 * @param flags
 */
inline void TMPacket::setFlags(uint8_t flags) {
    raw[TM_FLAGS_POS] = flags;
}

inline void TMPacket::setDataLength(uint8_t length) {
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
inline uint8_t TMPacket::setData(uint8_t *data, uint8_t len) {
    if (data == nullptr)
        return 0;

    if (len > TM_DATA_LENGTH)
        len = TM_DATA_LENGTH;
    raw[TM_DATA_LEN_POS] = len;
    memcpy(raw + TM_DATA_POS, data, len);
    return len;
}

inline uint8_t TMPacket::getVersion() {
    return raw[TM_VERSION_POS];
}

inline uint8_t TMPacket::getSource() {
    return raw[TM_SOURCE_POS];
}

inline uint8_t TMPacket::getDestination() {
    return raw[TM_DESTINATION_POS];
}

inline uint16_t TMPacket::getSequence() {
    return ((uint16_t)raw[TM_SEQUENCE_POS]) << 8 | raw[TM_SEQUENCE_POS + 1];
}

inline uint8_t TMPacket::getRepeatCount() {
    return getBits(raw[TM_FLAGS_POS], TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
}

inline uint8_t TMPacket::getNodeType() {
    return getBits(raw[TM_FLAGS_POS], TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB);
}

inline uint8_t TMPacket::getMessageType() {
    return getBits(raw[TM_FLAGS_POS], TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
}

inline uint8_t TMPacket::getFlags() {
    return raw[TM_FLAGS_POS];
}

inline uint8_t TMPacket::getDataLength() {
    return raw[TM_DATA_LEN_POS];
}

/** @brief Get packet data as a pointer.
 * Can be used for direct write instead of copying data into the packet.
 *
 * @return Returns a pointer to the interla packet data structure.
 */
inline uint8_t *TMPacket::getData() {
    return raw + TM_DATA_POS;
}

/** @brief Copies packet data to buffer.
 *
 * @param buffer Buffer to which to copy the data.
 * @param len Length of the buffer.
 * @return Number of copied bytes.
 */
inline uint8_t TMPacket::getData(uint8_t *buffer, uint8_t len) {
    if (buffer == nullptr)
        return 0;

    if (len > raw[TM_DATA_LEN_POS])
        len = raw[TM_DATA_LEN_POS];
    memcpy(buffer, raw + TM_DATA_POS, len);
    return len;
}

/** @brief Size of the used space inside the packet.
 *
 * @return Header size + size of data currently stored inside the packet.
 */
inline uint8_t TMPacket::size() {
    return TM_HEADER_LENGTH + raw[TM_DATA_LEN_POS];
}

inline void TMPacket::clear() {
    memset(raw, 0, TM_PACKET_SIZE);
}

inline bool TMPacket::empty() {
    return !raw[TM_SOURCE_POS] && !raw[TM_DESTINATION_POS];
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
uint8_t TMPacket::buildPacket(uint8_t source, uint8_t destination, uint16_t seq, uint8_t node_type,
                              uint8_t message_type, uint8_t repeat_cnt = 0, uint8_t *data = nullptr, uint8_t length = 0) {

    // data are null, but some are to be copied -> don't copy anything
    if (data == nullptr && length != 0)
        return TM_BUILD_DATA_NULL;

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
    setSequence(seq);
    setFlags(repeat_cnt << 6 | message_type << 2 | node_type);
    setDataLength(length);

    // check if header is valid
    ret = checkHeader();

    // return now if there are no data to copy
    if (!length)
        return ret;

    if (setData(data, length) != length)
        return ret | TM_BUILD_DATA_TRIM;

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
        ret |= TM_CHECK_VERSION;

    if (getSource() == TM_BROADCAST_ADDRESS)
        ret |= TM_CHECK_ADDRESS;

    // data too long
    if (getDataLength() > TM_DATA_LENGTH)
        ret |= TM_CHECK_DATA_LEN;

    switch (getMessageType())
    {
    case TM_MSG_OK:
        if (getDestination() == TM_BROADCAST_ADDRESS)
            ret |= TM_CHECK_MSG_TYPE;
        break;
    case TM_MSG_CUSTOM:
        break;
    case TM_MSG_ERR:
        if (getDestination() == TM_BROADCAST_ADDRESS)
            ret |= TM_CHECK_MSG_TYPE;
    case TM_MSG_PING:
        if (getDataLength() != 1)
            ret |= TM_CHECK_MSG_DATA_LEN;
        break;
    case TM_MSG_REGISTER:
        if (getDataLength())
            ret |= TM_CHECK_MSG_DATA_LEN;
        break;
    case TM_MSG_STATUS:
        if (!getDataLength())
            ret |= TM_CHECK_MSG_DATA_LEN;
        break;
    case TM_MSG_COMBINED:
        if (getDataLength() < 2)
            ret |= TM_CHECK_MSG_DATA_LEN;
        break;
    case TM_MSG_REQUEST:
        if (getDataLength() < 1)
            ret |= TM_CHECK_MSG_DATA_LEN;
        break;
    default:
        ret |= TM_CHECK_MSG_TYPE;
        break;
    }

    return ret;
};


/** @brief Set bits in x from msb to lsb to val */
void TMPacket::setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb) {
    uint8_t mask = (1 << (msb - lsb + 1)) - 1;
    mask <<= lsb;
    *x = (*x & ~mask) | ((val << lsb) & mask);
}

/** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
uint8_t TMPacket::getBits(uint8_t x, uint8_t msb, uint8_t lsb) {
    return (x >> lsb) & ((1 << (msb - lsb + 1)) - 1);
}
