/**
 * @file SX127X.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX127X library
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sx127x.hpp"


static const char* TAG = "sx127x";


SX127X::SX127X(uint8_t cs, uint8_t rst, uint8_t dio0) {
    this->cs     = cs;
    this->rst    = rst;
    this->dio0   = dio0;
}

SX127X::~SX127X() {

}

//callback functions
void SX127X::registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output) {
    this->pinMode                   = func;
    this->input                     = input;
    this->output                    = output;
    this->flags.single.has_pin_mode = true;
}
void SX127X::registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high, uint8_t low) {
    this->pinWrite                   = func;
    this->high                       = high;
    this->low                        = low;
    this->flags.single.has_pin_write = true;
}
void SX127X::registerPinRead(uint8_t (*func)(uint8_t)) {
    this->pinRead = func;
    this->flags.single.has_pin_read  = true;
}
void SX127X::registerDelay(void (*func)(uint32_t)) {
    this->delay                  = func;
    this->flags.single.has_delay = true;
}
void SX127X::registerSPIStartTransaction(void (*func)()) {
    this->SPIBeginTransaction           = func;
    this->flags.single.has_spi_start_tr = true;
}
void SX127X::registerSPIEndTransaction(void (*func)()) {
    this->SPIEndTransaction           = func;
    this->flags.single.has_spi_end_tr = true;
}
void SX127X::registerSpiTransfer(void (*func)(uint8_t, uint8_t *, size_t)) {
    this->spiTransfer               = func;
    this->flags.single.has_transfer = true;
}


uint8_t SX127X::begin(uint16_t frequency, uint8_t sync_word, uint16_t preamble_len) {
    //toggle pin modes if it was setup
    if (this->flags.single.has_pin_mode) {
        pinMode(cs, output);
        pinMode(rst, output);
        pinMode(dio0, input);
    }

    //check that all required callbacks were set
    if (!this->flags.single.has_pin_write ||
        !this->flags.single.has_transfer ||
        !this->flags.single.has_delay)
        return 1;   //TODO return values

    //set pins to their default state
    pinWrite(this->cs, this->high);
    pinWrite(this->rst, this->high);

    //check chip type/version
    //TODO extend for all versions
    //this->chip_version = getVersion();
    //if (chip_version != SX1272_CHIP_VERSION &&
    //    chip_version != SX1276_CHIP_VERSION && 
    //    chip_version != RFM95_CHIP_VERSION && 
    //    chip_version != RFM69_CHIP_VERSION) {
    //    return 2;
    //}

    float freq   = 434.0F;
    float bw     = 125.0F;
    uint8_t sf   = 7U;
    uint8_t cr   = 7U;

    //go to standby to change and set default settings
    setMode(SX127X_OP_MODE_STANDBY);

    //set lora mode
    //TODO check reg version for all chips
    //TODO implement FSK
    setModemMode(SX127X_MODEM_LORA);

    //TODO current limit
    setCurrentLimit(100);

    //module I have does not have RFO pin connected
    //so let's just enable PA_BOOST and set lowest possible power
    //TODO power
    writeRegister(REG_PA_CONFIG, SX127X_PA_SELECT_BOOST);

    //enable automatic gain control
    //TODO gain
    setGain(SX127X_LNA_GAIN_AUTOMATIC);
    setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_ON, 2, 2);

    //set LoRa sync word
    setSyncWord(sync_word);

    //set preamble length
    setPreambleLength(preamble_len);

    //set default settings
    //turn off frequency hopping
    setFrequencyHopping(0);

    //set frequency
    setFrequency(frequency);

    //125khz bandwidth
    setBandwidth(LORA_BANDWIDTH_125kHz);

    //set spreading factor to 7
    setSpreadingFactor(LORA_SPREADING_FACTOR_7);

    //set error coding rate to 4/5
    setCodingRate(LORA_CODING_RATE_4_5);

    //enable crc
    //TODO CRC
    setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_ON, 2, 2);

    //TODO
    //disable all IO pins
    writeRegister(REG_DIO_MAPPING_1, 0xFF);
    setRegister(REG_DIO_MAPPING_2, 0b11110000);

    return 0;
}


void SX127X::reset() {
    pinWrite(this->rst, low);
    delay(1);
    pinWrite(this->rst, high);
    delay(5);
}

uint8_t SX127X::getVersion() {
    //reset();
    //delay(10);
    return readRegister(REG_VERSION);
}

void SX127X::setMode(uint8_t mode) {
    setRegister(REG_OP_MODE, mode, 0, 2);
}

void SX127X::setModemMode(uint8_t modem) {
    setMode(SX127X_OP_MODE_SLEEP);

    setRegister(REG_OP_MODE, modem, 7, 7);

    setMode(SX127X_OP_MODE_STANDBY);
}

uint8_t SX127X::getModemMode() {
    //TODO test masking
    return readRegister(REG_OP_MODE, 7, 7);
}

void SX127X::setFrequencyHopping(uint8_t period) {
    writeRegister(REG_HOP_PERIOD, HOP_PERIOD_OFF);
}

void SX127X::setSyncWord(uint8_t sync_word) {
    writeRegister(REG_SYNC_WORD, sync_word);
}

//TODO FSK has different register
uint8_t SX127X::setPreambleLength(uint16_t preamble_len) {
    if (preamble_len < 6)
        return ERR_INVALID_PREAMBLE_LEN;   //TODO return values
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)((preamble_len >> 8) & 0xFF));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(preamble_len & 0xFF));
    return 0;
}

uint8_t SX127X::setBandwidth(uint8_t bandwidth) {
    //TODO check modem type
    if (getModemMode() != SX127X_MODEM_LORA)
        return ERR_INVALID_MODEM_MODE;

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
        default: return ERR_INVALID_BANDWIDTH;  //TODO return types
    }

    setRegister(REG_MODEM_CONFIG_1, bandwidth, 4, 7);

    //check and enable/disable LDRO
    setLowDataRateOptimalization();
    return 0;
}


uint32_t SX127X::getBandwidth() {
    return uint32_t(bw * 1000);
}

uint8_t SX127X::setSpreadingFactor(uint8_t spreading_factor) {
    if (getModemMode() != SX127X_MODEM_LORA)
        return ERR_INVALID_MODEM_MODE;

    switch (spreading_factor) {
        case LORA_SPREADING_FACTOR_6  :
        case LORA_SPREADING_FACTOR_7  :
        case LORA_SPREADING_FACTOR_8  :
        case LORA_SPREADING_FACTOR_9  :
        case LORA_SPREADING_FACTOR_10 :
        case LORA_SPREADING_FACTOR_11 :
        case LORA_SPREADING_FACTOR_12 : break;
        default: return ERR_INVALID_SPREADING_FACTOR;  //TODO return types
    }

    setRegister(REG_MODEM_CONFIG_2, spreading_factor, 4, 7);
    //special configuration for spreading factor 6
    if (spreading_factor == LORA_SPREADING_FACTOR_6) {
        setRegister(REG_MODEM_CONFIG_1, LORA_IMPLICIT_HEADER, 0, 0);
        setRegister(REG_DETECT_OPTIMIZE, LORA_DETECTION_OPTIMIZE_SF6, 0, 2);
        setRegister(REG_DETECTION_THRESHOLD, LORA_DETECTION_THRESHOLD_SF6);
    }
    //default everything when sf != 6
    else {
        setRegister(REG_MODEM_CONFIG_1, LORA_EXPLICIT_HEADER, 0, 0);
        setRegister(REG_DETECT_OPTIMIZE, LORA_DETECTION_OPTIMIZE_SF7_12, 0, 2);
        setRegister(REG_DETECTION_THRESHOLD, LORA_DETECTION_THRESHOLD_SF7_12);
    }

    this->sf = spreading_factor >> 4;

    //check and enable/disable LDRO
    setLowDataRateOptimalization();
    return 0;
}

uint8_t SX127X::setFrequency(uint16_t frequency) {
    //TODO check frequency range based on selected module

    //go to standby first
    setMode(SX127X_OP_MODE_STANDBY);

    //calculate new frequency reg value and set it
    uint32_t f_rf = frequency * ((uint32_t(1) << 19) / 32);
    setRegister(REG_FRF_MSB, f_rf >> 16);
    setRegister(REG_FRF_MID, f_rf >> 8);
    setRegister(REG_FRF_LSB, f_rf);

    return 0;
}

uint8_t SX127X::setCodingRate(uint8_t coding_rate) {
    if (getModemMode() != SX127X_MODEM_LORA)
        return ERR_INVALID_MODEM_MODE;

    switch(coding_rate) {
        case LORA_CODING_RATE_4_5:
        case LORA_CODING_RATE_4_6:
        case LORA_CODING_RATE_4_7:
        case LORA_CODING_RATE_4_8: break;
        default: return ERR_INVALID_CODING_RATE;
    }

    this->cr = (coding_rate + 0b00001000) >> 1;
    setRegister(REG_MODEM_CONFIG_1, coding_rate, 1, 3);
    return 0;
}

uint8_t SX127X::setCurrentLimit(uint8_t max_current) {
    if ((max_current < 45 || max_current > 240) && max_current != 0)
        return ERR_INVALID_CURRENT_LIMIT;

    setMode(SX127X_OP_MODE_STANDBY);

    //disable OCP
    if (max_current == 0)
        setRegister(REG_OCP, OCP_OFF, 5, 5);
    if (max_current <= 120)
        setRegister(REG_OCP, ((max_current - 45) / 5) | OCP_ON, 0, 5);
    else if (max_current <= 240)
        setRegister(REG_OCP, ((max_current + 30) / 10) | OCP_ON, 0, 5);
    return 0;
}

uint8_t SX127X::setGain(uint8_t gain) {
    if ((gain >> 5) > 6)
        return ERR_INVALID_GAIN;

    setMode(SX127X_OP_MODE_STANDBY);

    //TODO FSK
    //get mode

    //set automatic gain control
    if (gain == 0)
        setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_ON, 2, 2);
    else {
        setRegister(REG_MODEM_CONFIG_3, LORA_AGC_AUTO_OFF, 2, 2);
        setRegister(REG_LNA, gain, 5, 7);
    }

    return 0;
}

//TODO
void SX127X::setPower(int8_t power, bool pa_boost) {
    if (pa_boost && (power < 2 || power > 17))
        return ERR_INVALID_POWER;
    if (!pa_boost && (power < -4 || power > 15))
        return ERR_INVALID_POWER;

    setMode(SX127X_OP_MODE_STANDBY);

    if (pa_boost) {
        if (power == 20) {
            setRegister(REG_PA_CONFIG, SX127X_PA_SELECT_BOOST | )
            setRegister(REG_PA_DAC, SX127X_PA_SELECT_BOOST)
        }
        else {

        }
    }
    else {

    }
}

void SX127X::setCRC(bool enable) {
    //TODO FSK
    if (getModemMode() != SX127X_MODEM_LORA)
        return;
    
    if(enable)
        setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_ON, 2, 2);
    else
        setRegister(REG_MODEM_CONFIG_2, LORA_RX_PAYLOAD_CRC_OFF, 2, 2);
}

void SX127X::setLowDataRateOptimalization() {
    this->ts = float(uint16_t(1) << this->sf) / this->bw;
    if(this->ts >= 16.0)
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_ON, 3, 3);
    else
        setRegister(REG_MODEM_CONFIG_3, LORA_LOW_DATA_RATE_OPT_OFF, 3, 3);
}


uint8_t SX127X::transmit(uint8_t *data, uint8_t length) {
    setMode(SX127X_OP_MODE_STANDBY);

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
    writeRegistersBurst(REG_FIFO, data, length);

    //start transmission
    setMode(SX127X_OP_MODE_TX);
    while(!pinRead(this->dio0));    //sometimes not enough
    while(!(readRegister(REG_IRQ_FLAGS) & 0b00001000));
    setMode(SX127X_OP_MODE_STANDBY);

    //read and clear interrupts
    uint8_t reg = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    return reg;
}


void SX127X::SPIMakeTransaction(uint8_t addr, uint8_t *data, size_t length) {
    if (this->flags.single.has_spi_start_tr)
        this->SPIBeginTransaction();

    pinWrite(this->cs, this->low);
    
    this->spiTransfer(addr, data, length);
    
    pinWrite(this->cs, this->high);

    if (this->flags.single.has_spi_end_tr)
        this->SPIEndTransaction();
}

uint8_t SX127X::readRegister(uint8_t addr, uint8_t mask_lsb, uint8_t mask_msb) {
    addr &= SX127X_READ_MASK;
    uint8_t reg;
    SPIMakeTransaction(addr, &reg);

    uint8_t keep_mask = ~(0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    return reg & keep_mask;
}

void SX127X::readRegistersBurst(uint8_t addr, uint8_t *data, size_t length) {
    addr &= SX127X_READ_MASK;
    SPIMakeTransaction(addr, data, length);
}

void SX127X::writeRegister(uint8_t addr, uint8_t data) {
    addr |= SX127X_WRITE_MASK;
    SPIMakeTransaction(addr, &data);
}

void SX127X::writeRegistersBurst(uint8_t addr, uint8_t *data, size_t length) {
    addr |= SX127X_WRITE_MASK;
    SPIMakeTransaction(addr, data, length);
}

void SX127X::setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb, uint8_t mask_msb) {
    uint8_t reg = readRegister(addr);
    
    //calculate register clear mask
    uint8_t clr_mask = (0xFF >> (8 - mask_lsb) | 0xFF << (mask_msb + 1));
    data = (reg & clr_mask) | (data & ~clr_mask);

    writeRegister(addr, data);
}

