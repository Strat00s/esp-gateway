/**
 * @file SX127X.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX1278 library
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sx127x.hpp"

SX127X::SX127X(uint8_t cs, uint8_t rst, uint8_t dio0) {
    this->cs     = cs;
    this->rst    = rst;
    this->dio0   = dio0;
}

SX127X::~SX127X() {

}


void SX127X::registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output) {
    this->pinMode = func;
    this->input = input;
    this->output = output;
    this->has_pin_mode = true;
}
void SX127X::registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high, uint8_t low) {
    this->pinWrite = func;
    this->high = high;
    this->low = low;
    this->has_pin_write = true;
}
void SX127X::registerDelay(void (*func)(uint32_t)) {
    this->delay = func;
    this->has_delay = true;
}

void SX127X::startSpiTransfer(void (*func)()) {
    this->startTransfer = func;
    this->has_start = true;
}
void SX127X::endSpiTransfer(void (*func)()) {
    this->endTransfer = func;
    this->has_end = true;
}
void SX127X::registerSpiTransfer(uint8_t (*func)(uint8_t*, uint8_t*, uint8_t)) {
    this->spiTransfer = func;
    this->has_transfer = true;
}


//initialize module with following default settings:
//mode: LoRa
//frequency: 434MHz
//bandwidth: 125KHz
//spreading factor: 9 (512 chips/symbol)
//crc: enabled and set to 4/7
//gain: automatic
//frequency hopping: off
//TODO FSK
uint8_t SX127X::begin(uint32_t frequency, uint8_t sync_word, uint16_t preamble_len) {
    //toggle pin modes if it was setup
    if (has_pin_mode) {
        pinMode(cs, output);
        pinMode(rst, output);
        pinMode(dio0, input);
    }

    if (!has_pin_write || !has_delay || !has_transfer)
        return 1;

    //set pins to their default state
    pinWrite(cs, high);
    pinWrite(rst, low);

    //check chip type/version
    this->chip_version = getVersion();
    if (chip_version != SX1272_CHIP_VERSION &&
        chip_version != SX1276_CHIP_VERSION && 
        chip_version != RFM95_CHIP_VERSION && 
        chip_version != RFM69_CHIP_VERSION) {
        return 2;
    }

    float freq   = 434.0F;
    float bw     = 125.0F;
    uint8_t sf   = 7U;
    uint8_t cr   = 7U;

    //go to standby to change settings and set default settings
    setMode(SX1278_STANDBY);

    //set lora mode
    //TODO check reg value for other chips
    //TODO implement FSK
    setModemMode(SX1278_LORA);

    //set default settings
    //turn off frequency hopping
    setFrequencyHopping(0);

    //set LoRa sync word
    setSyncWord(sync_word);

    //TODO current limit

    //set preamble length
    setPreambleLength(preamble_len);

    //125khz bandwidth
    setBandwidth(LORA_BANDWIDTH_125kHz);

    //set spreading factor to 7
    setSpreadingFactor(7);

    //set frequency
    setFrequency(frequency);

    //explicit header
    setRegister(REG_MODEM_CONFIG_1, LORA_EXPLICIT_HEADER, 0, 0);

    //set spreading factor 7
    //is already set by default
    //setRegister(REG_MODEM_CONFIG_2, LORA_SPREADING_FACTOR_7, 4, 7);
    setRegister(REG_DETECT_OPTIMIZE, DETECT_OPTIMIZE_SF_7_12, 0, 2);
    setRegister(REG_DETECTION_THRESHOLD, DETECTION_THRESHOLD_SF_7_12);

    //set error coding rate to 4/5
    setRegister(REG_MODEM_CONFIG_1, LORA_CODING_RATE_4_5, 1, 3);

    //module I have does not have RFO pin connected
    //so let's just enable PA_BOOST and set lowest possible power
    writeRegister(REG_PA_CONFIG, SX1278_PA_SELECT_BOOST);

    //enable automatic gain control
    setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_ON, 2, 2);

    //enable crc
    setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_ON, 2, 2);

    //disable all IO pins
    writeRegister(REG_DIO_MAPPING_1, 0xFF);
    setRegister(REG_DIO_MAPPING_2, 0b11110000);

    return 0;
}


void SX127X::reset() {
    pinMode(this->rst, output);
    pinWrite(this->rst, low);
    delay(1);
    pinWrite(this->rst, high);
    delay(5);
}

uint8_t SX127X::getVersion() {
    reset();
    delay(10);
    return readRegister(REG_VERSION);
}

void SX127X::setMode(uint8_t mode) {
    setRegister(REG_OP_MODE, mode, 0, 2);
}

void SX127X::setModemMode(uint8_t modem) {
    setMode(SX1278_SLEEP);

    setRegister(REG_OP_MODE, modem, 7, 7);

    setMode(SX1278_STANDBY);
}

//TODO
void SX127X::setFrequencyHopping(uint8_t period) {
    writeRegister(REG_HOP_PERIOD, HOP_PERIOD_OFF);
}

//TODO 
void SX127X::setSyncWord(uint8_t sync_word) {
    writeRegister(REG_SYNC_WORD, sync_word);
}

//TODO FSK has different register
uint8_t SX127X::setPreambleLength(uint16_t preamble_len) {
    if (preamble_len < 6)
        return 1;
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)((preamble_len >> 8) & 0xFF));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
}

uint8_t SX127X::setBandwidth(uint8_t bandwidth) {
    switch (bandwidth) {
        case LORA_BANDWIDTH_7_8kHz   : this->bw =   7.8;  break;
        case LORA_BANDWIDTH_10_4kHz  : this->bw =  10.4;  break;
        case LORA_BANDWIDTH_15_6kHz  : this->bw =  15.6;  break;
        case LORA_BANDWIDTH_20_8kHz  : this->bw =  20.8;  break;
        case LORA_BANDWIDTH_31_25kHz : this->bw =  31.25; break;
        case LORA_BANDWIDTH_41_7kHz  : this->bw =  41.7;  break;
        case LORA_BANDWIDTH_62_5kHz  : this->bw =  62.5;  break;
        case LORA_BANDWIDTH_125kHz   : this->bw = 125.0;  break;
        case LORA_BANDWIDTH_250kHz   : this->bw = 250.0;  break;
        case LORA_BANDWIDTH_500kHz   : this->bw = 500.0;  break;
        default: return 1;  //TODO return types
    }

    setRegister(REG_MODEM_CONFIG_1, bandwidth, 4, 7);
    setLowDataOptimalization();
}

//TODO read it from register or store it?
/** @brief 
 * 
 * @return uint32_t bandwidth in Hz
 */
uint32_t SX127X::getBandwidth() {
    return uint32_t(bw * 1000);
}

uint8_t SX127X::setSpreadingFactor(uint8_t spreading_factor) {
    switch (spreading_factor) {
        case LORA_SPREADING_FACTOR_6  :
        case LORA_SPREADING_FACTOR_7  :
        case LORA_SPREADING_FACTOR_8  :
        case LORA_SPREADING_FACTOR_9  :
        case LORA_SPREADING_FACTOR_10 :
        case LORA_SPREADING_FACTOR_11 :
        case LORA_SPREADING_FACTOR_12 : break;
        default: return 1;  //TODO return types
    }

    this->sf = spreading_factor >> 4;

    setRegister(REG_MODEM_CONFIG_2, this->sf, 4, 7);
    setLowDataOptimalization();
}

void SX127X::setFrequency(uint32_t frequency) {
    uint32_t FRF = (frequency * (uint32_t(1) << 19)) / 32.0;
    setRegister(REG_FRF_MSB, (FRF & 0xFF0000) >> 16);
    setRegister(REG_FRF_MID, (FRF & 0x00FF00) >> 8);
    setRegister(REG_FRF_LSB, FRF & 0x0000FF);
}

/** @brief Set low data rate optimalization on if time on air is more than 16ms
 * 
 */
void SX127X::setLowDataOptimalization() {
    this->ts = (float(uint16_t(1) << this->sf) / this->bw);
    if(this->ts >= 16.0)
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_ON, 3, 3);
    else
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_OFF, 3, 3);
}



uint8_t SX127X::readRegister(uint8_t addr) {
    addr &= ~(1 << 7);
    uint8_t reg;
    pinWrite(this->cs, low);
    spiTransfer(&addr, &reg, 1);
    pinWrite(this->cs, high);
    return reg;
}

void SX127X::writeRegister(uint8_t addr, uint8_t data) {
    addr |= SX1278_WRITE;
    pinWrite(this->cs, low);
    spiTransfer(&addr, &data, 1);
    pinWrite(this->cs, high);
}

void SX127X::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
    uint8_t reg = readRegister(addr);
    
    uint8_t mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & ~mask) | (data & mask);

    pinWrite(this->cs, low);
    writeRegister(addr, data);
    pinWrite(this->cs, high);
}

uint8_t SX127X::transmit(uint8_t *data, uint8_t length) {
    setMode(SX1278_STANDBY);

    //set IO mapping for dio0 to be end of transmission
    setRegister(REG_DIO_MAPPING_1, DIO0_LORA_TX_DONE, 6, 7);

    //clear interrupt flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    //set packet length
    setRegister(REG_PAYLOAD_LENGTH, length);

    //set FIFO pointers (all 256 bytes used for TX)
    setRegister(REG_FIFO_TX_BASE_ADDR, 0);
    setRegister(REG_FIFO_ADDR_PTR, 0);

    //write data to FIFO
    //pinWrite(this->cs, LOW);
    //this->spi->writeRegisterBurst(REG_FIFO, data, length, SX1278_WRITE);
    //pinWrite(this->cs, HIGH);

    //start transmission
    setMode(SX1278_TX);
    //while(!digitalRead(this->dio0));    //sometimes not enough
    while(!(readRegister(REG_IRQ_FLAGS) & 0b00001000));
    setMode(SX1278_STANDBY);

    //read and clear interrupts
    uint8_t reg = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    return reg;
}
