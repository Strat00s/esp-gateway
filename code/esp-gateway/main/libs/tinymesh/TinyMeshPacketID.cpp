#include "TinyMeshPacketID.hpp"


inline void TMPacketID::setSource(uint8_t source){
    raw[TMPID_SOURCE_POS] = source;
}

inline void TMPacketID::setDestination(uint8_t destination){
    raw[TMPID_DESTINATION_POS] = destination;
}

inline void TMPacketID::setSequence(uint8_t sequence){
    raw[TMPID_SEQUENCE_POS] = sequence;
}

inline void TMPacketID::setFlags(uint8_t flags){
    raw[TMPID_FLAGS_POS] = flags;
}

inline void TMPacketID::setRepeatCount(uint8_t repeat_cnt){
    if (repeat_cnt > 3)
        repeat_cnt = 3;
    setBits(&raw[TMPID_FLAGS_POS], repeat_cnt, TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
}

inline void TMPacketID::setMessageType(uint8_t message_type){
    if (message_type > 15)
        message_type = 15;
    setBits(&raw[TMPID_FLAGS_POS], message_type, TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
}

inline uint8_t TMPacketID::getSource(){
    return raw[TMPID_SOURCE_POS];
}

inline uint8_t TMPacketID::getDestination(){
    return raw[TMPID_DESTINATION_POS];
}

inline uint8_t TMPacketID::getSequence(){
    return raw[TMPID_SEQUENCE_POS];
}

inline uint8_t TMPacketID::getFlags(){
    return raw[TMPID_FLAGS_POS];
}

inline uint8_t TMPacketID::getRepeatCount(){
    return getBits(raw[TMPID_FLAGS_POS], TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
}

inline uint8_t TMPacketID::getMessageType(){
    return getBits(raw[TMPID_FLAGS_POS], TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
}

inline void TMPacketID::clear(){
    memset(raw, 0, 4);
}

inline bool TMPacketID::empty(){
    return raw[TMPID_SOURCE_POS] == 0 && raw[TMPID_DESTINATION_POS] == 0;
}


void TMPacketID::setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb){
    uint8_t mask = (1 << (msb - lsb + 1)) - 1;
    mask <<= lsb;
    *x = (*x & ~mask) | ((val << lsb) & mask);
}

/** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
uint8_t TMPacketID::getBits(uint8_t x, uint8_t msb, uint8_t lsb){
    return (x >> lsb) & ((1 << (msb - lsb + 1)) - 1);
}
