#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>


//Flag bit locations
#define TMPID_MSG_TYPE_LSB  0
#define TMPID_MSG_TYPE_MSB  3
#define TMPID_RPT_CNT_LSB   4
#define TMPID_RPT_CNT_MSB   5

#define TMPID_SOURCE_POS      0
#define TMPID_DESTINATION_POS 1
#define TMPID_SEQUENCE_POS    2
#define TMPID_FLAGS_POS       3


class TMPacketID {
private:
    /** @brief Set bits in x from msb to lsb to val */
    void setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb);

    /** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
    inline uint8_t getBits(uint8_t x, uint8_t msb, uint8_t lsb) {
        return (x >> lsb) & ((1 << (msb - lsb + 1)) - 1);
    }

public:
    uint8_t raw[4];


    //TMPacketID();
    //~TMPacketID();


    void setAll(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t message_type, uint8_t repeat_cnt);

    inline void setSource(uint8_t source){
        raw[TMPID_SOURCE_POS] = source;
    }

    inline void setDestination(uint8_t destination){
        raw[TMPID_DESTINATION_POS] = destination;
    }

    inline void setSequence(uint8_t sequence){
        raw[TMPID_SEQUENCE_POS] = sequence;
    }

    inline void setFlags(uint8_t flags){
        raw[TMPID_FLAGS_POS] = flags;
    }

    void setRepeatCount(uint8_t repeat_cnt);

    void setMessageType(uint8_t message_type);


    inline uint8_t getSource(){
        return raw[TMPID_SOURCE_POS];
    }

    inline uint8_t getDestination(){
        return raw[TMPID_DESTINATION_POS];
    }

    inline uint8_t getSequence(){
        return raw[TMPID_SEQUENCE_POS];
    }

    inline uint8_t getFlags(){
        return raw[TMPID_FLAGS_POS];
    }

    inline uint8_t getRepeatCount(){
        return getBits(raw[TMPID_FLAGS_POS], TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
    }

    inline uint8_t getMessageType(){
        return getBits(raw[TMPID_FLAGS_POS], TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
    }

    inline void clear(){
        memset(raw, 0, 4);
    }

    inline bool empty(){
        return raw[TMPID_SOURCE_POS] == 0 && raw[TMPID_DESTINATION_POS] == 0;
    }

};