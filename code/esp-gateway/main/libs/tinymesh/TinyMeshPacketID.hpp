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
    uint8_t getBits(uint8_t x, uint8_t msb, uint8_t lsb);

public:
    uint8_t raw[4];


    TMPacketID();

    ~TMPacketID();


    inline void setAll(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t message_type, uint8_t repeat_cnt) {
        setSource(source);
        setDestination(destination);
        setSequence(sequence);
        setMessageType(message_type);
        setRepeatCount(repeat_cnt);
    }

    inline void setSource(uint8_t source);

    inline void setDestination(uint8_t destination);

    inline void setSequence(uint8_t sequence);
    
    inline void setFlags(uint8_t flags);

    inline void setRepeatCount(uint8_t repeat_cnt);

    inline void setMessageType(uint8_t message_type);


    inline uint8_t getSource();

    inline uint8_t getDestination();

    inline uint8_t getSequence();

    inline uint8_t getFlags();

    inline uint8_t getRepeatCount();

    inline uint8_t getMessageType();

    inline void clear();

    inline bool empty();

};