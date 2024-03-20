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
            0010 = TM_ADDR_REQUEST - request address from gateway in the network (if there is any)
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

#define TM_MAX_REPEAT 0b11

/*----(MESSAGE TYPES)----*/
#define TM_MSG_OK       0b0000 //response can't be brodcast
#define TM_MSG_ERR      0b0001 //response can't be brodcast
#define TM_MSG_ADDR_REQ 0b0010 //register to gateway (and get address)
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
#ifndef TM_DATA_LENGTH
    #define TM_DATA_LENGTH   256 - TM_HEADER_LENGTH
#endif

//Default config
#ifndef TM_DEFAULT_ADDRESS
    #define TM_DEFAULT_ADDRESS   0 //default address
#endif
#ifndef TM_BROADCAST_ADDRESS
    #define TM_BROADCAST_ADDRESS 255 //default broadcast address
#endif
#ifndef TM_SENT_QUEUE_SIZE
    #define TM_SENT_QUEUE_SIZE   10 //array of uint32
#endif
#ifndef TM_CLEAR_TIME
    #define TM_CLEAR_TIME 3000 //time before clearing entire sent queue in ms
#endif


#define ARRAY_CMP(a1, a2, len) (memcmp(a1, a2, len) == 0) //compare len of arrays against each other


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
} packet_t;


typedef struct {
    uint8_t source;
    uint8_t destination;
    union {
        struct {
            uint8_t answered : 1;
            uint8_t repeat   : 2;
            uint8_t msg_type : 4;
        };
        uint8_t raw;
    } flags;
} packet_id_t;



class TinyMesh {
private:
    uint8_t version   = TM_VERSION;           //supported TinyMesh version
    uint8_t address   = TM_DEFAULT_ADDRESS;   //this NODE address
    uint8_t gateway   = TM_BROADCAST_ADDRESS; //gateway address
    uint8_t node_type = TM_NODE_TYPE_NODE;    //this NODE type

public:
    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     */
    TinyMesh(uint8_t node_type);

    /** @brief Create TinyMesh instance.
     * The class containes a simple LCG for pseudo random message ID generation where the seed is used.
     * @param node_type Node type
     * @param address Node address
     */
    TinyMesh(uint8_t node_type, uint8_t address);

    TinyMesh(uint8_t node_type, uint8_t address, uint8_t gateway_address);

    ~TinyMesh();


    /** @brief Set node address.
     * Broadcast address will be skipped
     * 
     * @param address 
     */
    void setAddress(uint8_t address);

    /** @brief Set gateway address after registration
     * 
     * @param address Gateway address
     */
    void setGatewayAddress(uint8_t address);

    uint8_t getVersion();
    uint8_t getAddress();
    uint8_t getGatewayAddress();
    uint8_t getNodeType();


    /** @brief Build packet from specific data.
     * Runs checkHeader() at the end.
     * Saves packet if valid.
     * 
     * @param packet Packet poiner where to store header and data
     * @param destination Destination node address
     * @param message_type Message type
     * @param data Data which to send
     * @param length Length of data
     * @param repeat_cnt Packet repeate counter (0-3)
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint8_t buildPacket(packet_t *packet, uint8_t destination, uint8_t message_type, uint8_t repeat_cnt = 0, uint8_t *data = nullptr, uint8_t length = 0);


    /** @brief Check if stored packet has valid header and header data.
     * 
     * @param packet Packet to check
     * @return TM_OK on succes, TM_ERR_... macros on error
     */
    uint8_t checkHeader(packet_t *packet);


    /** @brief Create a Packet ID from packet
     * 
     * @param packet 
     * @return packet_id_t
     */
    packet_id_t createPacketID(packet_t *packet);


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
    void setBits(T1 *x, T2 val, uint8_t msb, uint8_t lsb) {
        T1 mask = ((T1)1 << (msb - lsb + 1)) - 1;
        mask <<= lsb;
        *x = (*x & ~mask) | ((val << lsb) & mask);
    }

    /** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
    template<typename T>
    T getBits(T x, uint8_t msb, uint8_t lsb) {
        return (x >> lsb) & (((T)1 << (msb - lsb + 1)) - 1);
    }
};
