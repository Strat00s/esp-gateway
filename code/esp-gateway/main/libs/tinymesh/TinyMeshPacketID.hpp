#pragma once
#include <stdint.h>
#include <string.h>
#include "../helpers.hpp"


//Flag bit locations
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
*/

class TMPacketID {
public:
    uint8_t raw[4];


    void setAll(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t repeat_cnt) {
        setSource(source);
        setDestination(destination);
        setSequence(sequence);
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

    void setRepeatCount(uint8_t repeat_cnt){
        if (repeat_cnt > 3)
            repeat_cnt = 3;
        setBits(&raw[TMPID_FLAGS_POS], repeat_cnt, TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
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


    void clear(){
        memset(raw, 0, 4);
    }

    bool empty(){
        return raw[TMPID_SOURCE_POS] == 0 && raw[TMPID_DESTINATION_POS] == 0;
    }
};