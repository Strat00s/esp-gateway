#include "TinyMeshPacketID.hpp"


//private
void TMPacketID::setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb){
    uint8_t mask = (1 << (msb - lsb + 1)) - 1;
    mask <<= lsb;
    *x = (*x & ~mask) | ((val << lsb) & mask);
}


//public
void TMPacketID::setAll(uint8_t source, uint8_t destination, uint8_t sequence, uint8_t message_type, uint8_t repeat_cnt) {
    setSource(source);
    setDestination(destination);
    setSequence(sequence);
    setMessageType(message_type);
    setRepeatCount(repeat_cnt);
}

void TMPacketID::setRepeatCount(uint8_t repeat_cnt){
    if (repeat_cnt > 3)
        repeat_cnt = 3;
    setBits(&raw[TMPID_FLAGS_POS], repeat_cnt, TMPID_RPT_CNT_MSB, TMPID_RPT_CNT_LSB);
}

void TMPacketID::setMessageType(uint8_t message_type){
    if (message_type > 15)
        message_type = 15;
    setBits(&raw[TMPID_FLAGS_POS], message_type, TMPID_MSG_TYPE_MSB, TMPID_MSG_TYPE_LSB);
}
