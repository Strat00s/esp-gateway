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
#include <stddef.h>
#include <string.h>
#include <deque>

#ifdef ARDUINO
#include <Arduino.h>
#endif


/*# Packet structure
-------------HEADER-------------
    VERSION     8b
        7-4: RESERVED
        3-0: VERSION
    SOURCE      8b
    DESTINATION 8b
    FLAGS       8b
        7-6: REPEAT CNT
        5-2: MESSAGE TYPE
            0000 = OK
            0001 = ERR
            0010 = TM_REGISTER - request address from gateway in the network (if there is any)
            0011 = PING - check if node is in the network. Contains hop counter
            0100 = STATUS
            0101 = TM_MSG_COMBINED
            0110 = TM_MSG_REQUEST - request a specific message from node
            0111 ... 1110 = RESERVED
            1111 = CUSTOM
        1-0: DEVICE TYPE
            00 = GATEWAY
            01 = NODE
            10 = LP_NODE
            11 = OTHER
    DATA LENGTH 8b
--------------DATA--------------
    DATA...     256b - len(header)



excahnges???:
request -> response
request -> combined response -> response


.join function (needs access to interface manager!!!)
send TM_ADRR_REQUEST
wait for response
    use the newly given address
otherwise, set address to 1 and ping
    if response, increase address and repeat
if no response, use that address


# Predefined messages packet structure and flow:
    OK
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_OK
            1-0: DEVICE TYPE
        DATA LENGTH: l
        DATA: l bytes (depends on answered message)
    ERROR
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_ERR
            1-0: DEVICE TYPE
        DATA LENGTH: 1
        DATA: ERROR_CODE
    PING
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_PING
            1-0: DEVICE TYPE
        DATA LENGTH: 1
        DATA: hop counter increased whenever ping is to be forwarded
    ADDRESS REQUEST
        VERSION
        SOURCE: 0
        DESTINATION: 255
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_REGISTER
            1-0: DEVICE TYPE
        DATA LENGTH: 0
    STATUS
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_STATUS
            1-0: DEVICE TYPE
        DATA LENGTH: 1 OR l > 0
        DATA: custom status code OR string of size l
    COMBINED
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_COMBINED
            1-0: DEVICE TYPE
        DATA LENGTH: (1 + 1 + l) * N
        DATA: MESSAGE TYPE | MESSAGE LEN | DATA | ... N times
    REQUEST
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_STATUS
            1-0: DEVICE TYPE
        DATA LENGTH: 1 OR l > 1
        DATA: MESSAGE_TYPE OR MESSAGE_TYPE + extra data
    CUSTOM
        VERSION
        SOURCE
        DESTINATION
        FLAGS
            7-6: REPEAT CNT
            5-2: MESSAGE TYPE: TM_MSG_COMBINED
            1-0: DEVICE TYPE
        DATA LENGTH: l
        DATA: l bytes
*/

namespace tinymesh {

#define TM_VERSION 3

//Flag bit locations
#define TM_NODE_TYPE_LSB 0
#define TM_NODE_TYPE_MSB 1
#define TM_MSG_TYPE_LSB  2
#define TM_MSG_TYPE_MSB  5
#define TM_RPT_CNT_LSB   6
#define TM_RPT_CNT_MSB   7


//RETURN FLAGS
#define TM_OK               0b00000000
#define TM_ERR_PACKET_NULL  0b00000001 //packet is null or given buffer is null
#define TM_ERR_DATA_NULL    0b00000010
#define TM_ERR_VERSION      0b00000100
#define TM_ERR_ADDRESS      0b00001000
#define TM_ERR_DATA_TRIM    0b00010000
#define TM_ERR_DATA_LENGTH  0b00100000
#define TM_ERR_MSG_TYPE     0b01000000
#define TM_ERR_MSG_DATA_LEN 0b10000000

//TM_ERR_MESSAGES 
#define TM_ERR_MSG_UNHANDLED     1
#define TM_ERR_SERVICE_UNHANDLED 2
#define TM_ERR_ADDRESS_LIMIT     3

//Check packet returns
#define TM_PACKET_DUPLICATE    0b00000001
#define TM_PACKET_RESPONSE     0b00000010
#define TM_PACKET_RND_RESPONSE 0b00000100
#define TM_PACKET_REQUEST      0b00001000
#define TM_PACKET_FORWARD      0b00010000
#define TM_PACKET_REPEAT       0b00100000
#define TM_PACKET_INV_RESPONSE 0b01000000

#define TM_MAX_REPEAT 0b11

/*----(MESSAGE TYPES)----*/
#define TM_MSG_OK       0b0000 //response can't be brodcast
#define TM_MSG_ERR      0b0001 //response can't be brodcast
#define TM_MSG_REGISTER 0b0010 //register to gateway (and get address)
#define TM_MSG_PING     0b0011 //ping a node
#define TM_MSG_STATUS   0b0100 //send string status
#define TM_MSG_COMBINED 0b0101 //combine multiple packets into one
#define TM_MSG_REQUEST  0b0110 //message split into multiple packets
#define TM_MSG_CUSTOM   0b1111 //send custom data

/*----(NODE TYPES)----*/
#define TM_NODE_TYPE_GATEWAY 0b00
#define TM_NODE_TYPE_NODE    0b01
#define TM_NODE_TYPE_LP_NODE 0b10
#define TM_NODE_TYPE_OTHER   0b11


//Packet part sizes
#define TM_HEADER_LENGTH 5 //header length in bytes
#define TM_DATA_LENGTH   256 - TM_HEADER_LENGTH
#define TM_PACKET_LENGTH TM_DATA_LENGTH + TM_HEADER_LENGTH


//Default config
#define TM_DEFAULT_ADDRESS   0 //default address
#define TM_BROADCAST_ADDRESS 255 //default broadcast address



#define ARRAY_CMP(a1, a2, len) (memcmp(a1, a2, len) == 0) //compare len of arrays against each other

#define GET_MSG_TYPE(flags)  getBits(flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB)
#define GET_RPT_CNT(flags)   getBits(flags, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB)
#define GET_NODE_TYPE(flags) getBits(flags, TM_NODE_TYPE_MSB, TM_NODE_TYPE_LSB)



typedef union{
    struct {
        uint8_t version;
        uint8_t source;
        uint8_t destination;
        uint8_t flags;
        uint8_t data_length;
        uint8_t data[TM_DATA_LENGTH];
    } fields;
    uint8_t raw[TM_HEADER_LENGTH + TM_DATA_LENGTH];
    uint8_t size() {
        return TM_HEADER_LENGTH + this->fields.data_length;
    }
} packet_t;


typedef struct {
    uint8_t source;
    uint8_t destination;
    uint8_t tts; //time to stale
    union {
        struct {
            uint8_t answered : 1; //answer was received or we sent an answer
            uint8_t repeat   : 2;
            uint8_t msg_type : 4;
        };
        uint8_t raw;
    } flags;

    inline bool isEmpty() {
        return !source && !destination && !flags.raw;
    }

    inline void clear() {
        source = 0;
        destination = 0;
        tts = 0;
        flags.raw = 0;
    }
} packet_id_t;

inline bool operator==(const packet_id_t& lhs, const packet_id_t& rhs) {
    return lhs.source == rhs.source &&
           lhs.destination == rhs.destination &&
           lhs.flags.msg_type == rhs.flags.msg_type;
}


inline uint8_t getVersion() {
    return TM_VERSION;
}


/** @brief Build packet from specific data.
 * Runs checkHeader() at the end.
 * Saves packet if valid.
 * 
 * @param packet Packet poiner where to store header and data
 * @param source Source node address
 * @param destination Destination node address
 * @param message_type Message type
 * @param data Data which to send
 * @param length Length of data
 * @param repeat_cnt Packet repeate counter (0-3)
 * @return TM_OK on succes, TM_ERR_... macros on error
 */
uint8_t buildPacket(packet_t *packet, uint8_t source, uint8_t destination, uint8_t node_type,
                    uint8_t message_type, uint8_t repeat_cnt = 0, uint8_t *data = nullptr, uint8_t length = 0) {
    uint8_t ret = TM_OK;

    if (packet == nullptr)
        return TM_ERR_PACKET_NULL;

    //build packet
    packet->fields.version     = TM_VERSION;
    packet->fields.source      = source;
    packet->fields.destination = destination;
    packet->fields.data_length = length;
    packet->fields.flags       = repeat_cnt << 6 | message_type << 2 | node_type;

    //check if header is valid
    ret |= checkPacket(packet);

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


/** @brief Check if stored packet has valid header and header data.
 * 
 * @param packet Packet to check
 * @return TM_OK on succes, TM_ERR_... macros on error
 */
uint8_t checkPacket(packet_t *packet) {
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
        case TM_MSG_OK:
            if (packet->fields.destination == TM_BROADCAST_ADDRESS)
                ret |= TM_ERR_MSG_TYPE;
                break;
        case TM_MSG_CUSTOM: break;
        case TM_MSG_ERR:
            if (packet->fields.destination == TM_BROADCAST_ADDRESS)
                ret |= TM_ERR_MSG_TYPE;
        case TM_MSG_PING:
            if (packet->fields.data_length != 1)
                ret |= TM_ERR_MSG_DATA_LEN;
            break;
        case TM_MSG_REGISTER:
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


/** @brief Create a packet ID from packet
 *
 * @param packet 
 * @return packet_id_t
 */
packet_id_t createPacketID(packet_t *packet) {
    packet_id_t packet_id;
    packet_id.source         = packet->fields.source;
    packet_id.destination    = packet->fields.destination;
    packet_id.flags.answered = 0;
    packet_id.flags.repeat   = getBits(packet->fields.flags, TM_RPT_CNT_MSB, TM_RPT_CNT_LSB);
    packet_id.flags.msg_type = getBits(packet->fields.flags, TM_MSG_TYPE_MSB, TM_MSG_TYPE_LSB);
    return packet_id;
}


//flag management helpers
/** @brief Set specific bits in x to val
 * 
 * @tparam T1 
 * @tparam T2 
 * @param x 
 * @param val 
 * @param msb 
 * @param lsb 
 */
template<typename T1, typename T2>
inline void setBits(T1 *x, T2 val, uint8_t msb, uint8_t lsb) {
    T1 mask = ((T1)1 << (msb - lsb + 1)) - 1;
    mask <<= lsb;
    *x = (*x & ~mask) | ((val << lsb) & mask);
}

/** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
template<typename T>
inline T getBits(T x, uint8_t msb, uint8_t lsb) {
    return (x >> lsb) & (((T)1 << (msb - lsb + 1)) - 1);
}
};
