#pragma once
#include <stdint.h>
#include <string.h>
#include "../helpers.hpp"


//Flag bit locations
#define TMPID_NODE_TYPE_LSB 0
#define TMPID_NODE_TYPE_MSB 0
#define TMPID_IS_RESP_LSB   1
#define TMPID_IS_RESP_MSB   1
#define TMPID_MSG_TYPE_LSB  2
#define TMPID_MSG_TYPE_MSB  5
#define TMPID_RPT_CNT_LSB   6
#define TMPID_RPT_CNT_MSB   7

#define TMPID_SOURCE_POS      0
#define TMPID_DESTINATION_POS 1
#define TMPID_SEQUENCE_POS    2
#define TMPID_FLAGS_POS       3

//TODO pid structure
//specifically flags -> I don't need all that information
//store something usefull instead (type of packet like forward and such)

/*# PID structure
-------------HEADER-------------
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
        1:  IS RESPONSE
            0 = FALSE - request
            1 = TRUE - response
        0: DEVICE TYPE
            0 = NODE
            1 = LP_NODE
*/

class TMPacketID {
public:
    uint8_t raw[4];


    void setAll(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t message_type, uint8_t repeat_cnt) {
        setSource(source);
        setDestination(destination);
        setSequence(sequence);
        setMessageType(message_type);
        setRepeatCount(repeat_cnt);
    }

    void setSource(uint8_t source){
        raw[TMPID_SOURCE_POS] = source;
    }

    void setDestination(uint8_t destination){
        raw[TMPID_DESTINATION_POS] = destination;
    }

    void setSequence(uint8_t sequence){
        raw[TMPID_SEQUENCE_POS] = sequence;
    }

    void setFlags(uint8_t flags){
        raw[TMPID_FLAGS_POS] = flags;
    }

    void setIsResponse(bool is_response) {
        setBits(&raw[TMPID_FLAGS_POS], is_response, TMPID_IS_RESP_MSB, TMPID_IS_RESP_LSB);

    }

    void setNodeType(bool normal_node = true) {
        setBits(&raw[TMPID_FLAGS_POS], normal_node, TMPID_NODE_TYPE_MSB, TMPID_NODE_TYPE_LSB);
    }

    void setRepeatCount(uint8_t repeat_cnt){
        if (repeat_cnt > 3)
            repeat_cnt = 3;
        setBits(&raw[TMPID_FLAGS_POS], repeat_cnt, TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
    }

    void setMessageType(uint8_t message_type){
        if (message_type > 15)
            message_type = 15;
        setBits(&raw[TMPID_FLAGS_POS], message_type, TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
    }

    uint8_t getSource(){
        return raw[TMPID_SOURCE_POS];
    }

    uint8_t getDestination(){
        return raw[TMPID_DESTINATION_POS];
    }

    uint8_t getSequence(){
        return raw[TMPID_SEQUENCE_POS];
    }

    uint8_t getFlags(){
        return raw[TMPID_FLAGS_POS];
    }

    uint8_t getRepeatCount(){
        return getBits(raw[TMPID_FLAGS_POS], TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
    }

    uint8_t getMessageType(){
        return getBits(raw[TMPID_FLAGS_POS], TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
    }

    void clear(){
        memset(raw, 0, 4);
    }

    bool empty(){
        return raw[TMPID_SOURCE_POS] == 0 && raw[TMPID_DESTINATION_POS] == 0;
    }
};