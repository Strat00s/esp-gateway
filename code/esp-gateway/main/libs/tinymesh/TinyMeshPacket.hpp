/** @file tinyMesh.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief TinyMesh is a simple protocol for IoT devices.
 * @version 3.0
 * @date 27-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <stdint.h>
#include <string.h>
#include "../helpers.hpp"


/*# Packet structure
-------------HEADER-------------
    VERSION     8b
    SOURCE      8b
    DESTINATION 8b
    SEQUENCE    8b
    FLAGS       8b
        7-6: REPEAT CNT
        5-2: MESSAGE TYPE
            0000 = OK
            0001 = ERR
            0010 = PING - check if node is in the network. Contains hop counter
            0011 - 1111 = CUSTOM - user defined
        1:  WAS FORWARDED
            0 = FALSE
            1 = TRUE
        0: DEVICE TYPE
            0 = NODE
            1 = LP_NODE
    DATA LENGTH 8b
--------------DATA--------------
    DATA...     256b - len(header)


*/


//Ideas
//TM_ERR BUSY code

#define TM_VERSION 3

//Flag bit locations
#define TM_NODE_TYPE_LSB 0
#define TM_NODE_TYPE_MSB 0
#define TM_IS_FWD_LSB    1
#define TM_IS_FWD_MSB    1
#define TM_MSG_TYPE_LSB  2
#define TM_MSG_TYPE_MSB  5
#define TM_RPT_CNT_LSB   6
#define TM_RPT_CNT_MSB   7


#define TM_VERSION_POS     0
#define TM_SOURCE_POS      1
#define TM_DESTINATION_POS 2
#define TM_SEQUENCE_POS    3
#define TM_FLAGS_POS       4
#define TM_DATA_LEN_POS    5

#define TM_DATA_POS        6


//RETURN FLAGS
#define TM_OK               0b00000000

#define TM_ERR_DATA_NULL    0b00000001
#define TM_ERR_HEADER_ERR   0b00000010
#define TM_ERR_DATA_TRIM    0b00000100

#define TM_ERR_VERSION      0b00001000
#define TM_ERR_ADDRESS      0b00010000
#define TM_ERR_DATA_LEN     0b00100000
#define TM_ERR_MSG_TYPE     0b01000000
#define TM_ERR_MSG_DATA_LEN 0b10000000


/*----(MESSAGE TYPES)----*/
#define TM_MSG_OK     0b0000 //response
#define TM_MSG_ERR    0b0001 //response
#define TM_MSG_PING   0b0010
#define TM_MSG_CUSTOM 0b0011
//rest can be user defined

/*----(NODE TYPES)----*/
//#define TM_NODE_TYPE_GATEWAY 0b00
//#define TM_NODE_TYPE_NODE    0b01
//#define TM_NODE_TYPE_LP_NODE 0b10
//#define TM_NODE_TYPE_OTHER   0b11
#define TM_NODE_TYPE_NORMAL 0
#define TM_NODE_TYPE_LP     1


//Packet header size
#define TM_HEADER_LENGTH 6

//#define TM_PACKET_SIZE TM_HEADER_LENGTH + DATA_LEN


//Default config
#define TM_DEFAULT_ADDRESS   0 //default address
#define TM_BROADCAST_ADDRESS 255 //default broadcast address


template<uint8_t DATA_LEN = 16>
class TMPacket {
    static_assert(DATA_LEN <= 249, "Packet data length exceeds the maximum allowed limit of 249B");
public:
    uint8_t raw[TM_HEADER_LENGTH + DATA_LEN] = {0};

    void setVersion(uint8_t version) {
        raw[TM_VERSION_POS] = version;
    }

    void setSource(uint8_t source) {
        raw[TM_SOURCE_POS] = source;
    }

    void setDestination(uint8_t destination) {
        raw[TM_DESTINATION_POS] = destination;
    }

    void setSequence(uint8_t sequence) {
        raw[TM_SEQUENCE_POS] = sequence;
    }

    void setRepeatCount(uint8_t repeat) {
        if (repeat > 3)
            repeat = 3;
        setBits(&raw[TM_FLAGS_POS], repeat, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
    }

    void setNodeType(uint8_t node_type) {
        if (node_type > 1)
            node_type = 0;
        setBits(&raw[TM_FLAGS_POS], node_type, TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB);
    }

    void setMessageType(uint8_t msg_type) {
        if (msg_type > 15)
            msg_type = 15;
        setBits(&raw[TM_FLAGS_POS], msg_type, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    }

    void setIsForwarded(bool is_forwarded) {
        setBits(&raw[TM_FLAGS_POS], is_forwarded, TM_IS_FWD_MSB, TM_IS_FWD_LSB);
    }


    /** @brief Set entire flag field at once
     *
     * @param flags
     */
    void setFlags(uint8_t flags) {
        raw[TM_FLAGS_POS] = flags;
    }

    void setDataLength(uint8_t length) {
        if (length > DATA_LEN)
            length = DATA_LEN;
        raw[TM_DATA_LEN_POS] = length;
    }

    /** @brief Copies specified data into the packet.
     * 
     * @param data Buffer with data to be written to the packet.
     * @param len Length of the data.
     * @return Number of copied bytes, which can be smaller than len.
     */
    uint8_t setData(uint8_t *data, uint8_t len) {
        if (data == nullptr)
            return 0;

        if (len > DATA_LEN)
            len = DATA_LEN;
        raw[TM_DATA_LEN_POS] = len;
        memcpy(raw + TM_DATA_POS, data, len);
        return len;
    }


    uint8_t getVersion() {
        return raw[TM_VERSION_POS];
    }

    uint8_t getSource() {
        return raw[TM_SOURCE_POS];
    }

    uint8_t getDestination() {
        return raw[TM_DESTINATION_POS];
    }

    uint8_t getSequence() {
        return raw[TM_SEQUENCE_POS];
    }

    uint8_t getRepeatCount() {
        return getBits(raw[TM_FLAGS_POS], TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
    }

    uint8_t getNodeType() {
        return getBits(raw[TM_FLAGS_POS], TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB);
    }

    uint8_t getMessageType() {
        return getBits(raw[TM_FLAGS_POS], TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    }

    bool isResponse() {
        uint8_t msg_type = getBits(raw[TM_FLAGS_POS], TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
        return msg_type == TM_MSG_OK || msg_type == TM_MSG_ERR;
    }

    bool isForwarded() {
        return getBits(raw[TM_FLAGS_POS], TM_IS_FWD_MSB, TM_IS_FWD_LSB);
    }

    uint8_t getFlags() {
        return raw[TM_FLAGS_POS];
    }

    uint8_t getDataLength() {
        return raw[TM_DATA_LEN_POS];
    }

    /** @brief Get packet data as a pointer.
     * Can be used for direct write instead of copying data into the packet.
     *
     * @return Returns a pointer to the interla packet data structure.
     */
    uint8_t *getData() {
        return raw + TM_DATA_POS;
    }

    /** @brief Copies packet data to buffer.
     * 
     * @param buffer Buffer to which to copy the data.
     * @param len Length of the buffer.
     * @return Number of copied bytes. 
     */
    uint8_t getData(uint8_t *buffer, uint8_t len) {
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
    uint8_t size() {
        return TM_HEADER_LENGTH + raw[TM_DATA_LEN_POS];
    }
    
    void clear() {
        memset(raw, 0, TM_HEADER_LENGTH + DATA_LEN);
    }
    
    bool empty() {
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
    uint8_t buildPacket(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t node_type,
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
        setIsForwarded(false);

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
    uint8_t checkHeader() {
        uint8_t ret = TM_OK;

        // unsuported version
        if (getVersion() != TM_VERSION)
            ret |= TM_ERR_VERSION;

        if (getSource() == TM_BROADCAST_ADDRESS || getSource() == getDestination())
            ret |= TM_ERR_ADDRESS;

        // data too long
        if (getDataLength() > DATA_LEN)
            ret |= TM_ERR_DATA_LEN;

        switch (getMessageType()) {
        case TM_MSG_OK:
        case TM_MSG_ERR:
            if (getDestination() == TM_BROADCAST_ADDRESS)
                ret |= TM_ERR_ADDRESS | TM_ERR_MSG_TYPE;
            break;

        case TM_MSG_PING:
            if (getDataLength() != 1)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;

        default:
            if (getMessageType() > 15)
                ret |= TM_ERR_MSG_TYPE;
            break;
        }

        return ret;
    };

};
