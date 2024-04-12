#pragma once
#include <stdint.h>

/** @brief Set bits in x from msb to lsb to val */
inline void setBits(uint8_t *x, uint8_t val, uint8_t msb, uint8_t lsb) {
    uint8_t mask = (1 << (msb - lsb + 1)) - 1;
    mask <<= lsb;
    *x = (*x & ~mask) | ((val << lsb) & mask);
}

/** @brief Get specific bits from x shifted to start from 1st (lsb) bit*/
inline uint8_t getBits(uint8_t x, uint8_t msb, uint8_t lsb) {
    return (x >> lsb) & ((1 << (msb - lsb + 1)) - 1);
}