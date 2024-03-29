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


#define TM_VERSION 3

//Flag bit locations
#define TM_NODE_TYPE_LSB 0
#define TM_NODE_TYPE_MSB 1
#define TM_MSG_TYPE_LSB  2
#define TM_MSG_TYPE_MSB  5
#define TM_RPT_CNT_LSB   6
#define TM_RPT_CNT_MSB   7

#define TM_VERSION_POS     0
#define TM_SOURCE_POS      1
#define TM_DESTINATION_POS 2
#define TM_SEQUENCE_POS    3
#define TM_FLAGS_POS       5
#define TM_DATA_LEN_POS    6
#define TM_DATA_POS        7


//RETURN FLAGS
#define TM_OK               0b00000000

#define TM_BUILD_DATA_NULL    0b00000001
#define TM_BUILD_HEADER_ERR   0b00000010
#define TM_BUILD_DATA_TRIM    0b00000100

#define TM_CHECK_VERSION      0b00001000
#define TM_CHECK_ADDRESS      0b00010000
#define TM_CHECK_DATA_LEN     0b00100000
#define TM_CHECK_MSG_TYPE     0b01000000
#define TM_CHECK_MSG_DATA_LEN 0b10000000


//Check packet returns
//#define TM_PACKET_DUPLICATE    0b00000001
//#define TM_PACKET_RESPONSE     0b00000010
//#define TM_PACKET_RND_RESPONSE 0b00000100
//#define TM_PACKET_REQUEST      0b00001000
//#define TM_PACKET_FORWARD      0b00010000
//#define TM_PACKET_REPEAT       0b00100000
//#define TM_PACKET_INV_RESPONSE 0b01000000

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


//Packet header size
#define TM_HEADER_LENGTH 7
#define TM_DATA_LENGTH 16

#define TM_PACKET_SIZE TM_HEADER_LENGTH + TM_DATA_LENGTH


//Default config
#define TM_DEFAULT_ADDRESS   0 //default address
#define TM_BROADCAST_ADDRESS 255 //default broadcast address


class TMPacket {
private:
    /** @brief Set bits in x from msb to lsb to val */
    void setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb);

    /** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
    uint8_t getBits(uint8_t x, uint8_t msb, uint8_t lsb);

public:
    uint8_t raw[TM_PACKET_SIZE];

    TMPacket();
    ~TMPacket();


    inline void setVersion(uint8_t version);

    inline void setSource(uint8_t source);

    inline void setDestination(uint8_t destination);

    inline void setSequence(uint16_t sequence);

    inline void setRepeatCount(uint8_t repeat);

    inline void setNodeType(uint8_t node_type);

    inline void setMessageType(uint8_t msg_type);

    /** @brief Set entire flag field at once
     *
     * @param flags 
     */
    inline void setFlags(uint8_t flags);

    inline void setDataLength(uint8_t length);

    /** @brief Copies specified data into the packet.
     * 
     * @param data Buffer with data to be written to the packet.
     * @param len Length of the data.
     * @return Number of copied bytes, which can be smaller than len.
     */
    inline uint8_t setData(uint8_t *data, uint8_t len);


    inline uint8_t getVersion();
    
    inline uint8_t getSource();
    
    inline uint8_t getDestination();
    
    inline uint16_t getSequence();
    
    inline uint8_t getRepeatCount();
    
    inline uint8_t getNodeType();
    
    inline uint8_t getMessageType();
    
    inline uint8_t getFlags();
    
    inline uint8_t getDataLength();

    /** @brief Get packet data as a pointer.
     * Can be used for direct write instead of copying data into the packet.
     * 
     * @return Returns a pointer to the interla packet data structure.
     */
    inline uint8_t *getData();

    /** @brief Copies packet data to buffer.
     * 
     * @param buffer Buffer to which to copy the data.
     * @param len Length of the buffer.
     * @return Number of copied bytes. 
     */
    inline uint8_t getData(uint8_t *buffer, uint8_t len);


    /** @brief Size of the used space inside the packet.
     * 
     * @return Header size + size of data currently stored inside the packet.
     */
    inline uint8_t size();

    inline void clear();

    inline bool empty();


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
    uint8_t buildPacket(uint8_t source, uint8_t destination, uint16_t seq, uint8_t node_type,
                        uint8_t message_type, uint8_t repeat_cnt = 0, uint8_t *data = nullptr, uint8_t length = 0);

    /** @brief Check if stored packet has valid header and header data.
     * 
     * @param packet Packet to check
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint8_t checkHeader();

};
