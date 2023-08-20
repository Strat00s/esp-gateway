/**
 * @file SX1278.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX1278 header file
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


//TODO all registers
//TODO return types


#pragma once
#include <stdio.h>

/*----(register fields)----*/
//SX127X chip version
#define SX1272_CHIP_VERSION 0x22
#define SX1273_CHIP_VERSION 0x22

#define SX1276_CHIP_VERSION 0x12
#define SX1277_CHIP_VERSION 0x12
#define SX1278_CHIP_VERSION 0x12
#define SX1279_CHIP_VERSION 0x12

#define RFM95_CHIP_VERSION  0x11
#define RFM96_CHIP_VERSION  0x11

#define RFM69_CHIP_VERSION  0x24


//TODO registers
//config 1
#define LORA_BANDWIDTH_7_8kHz    0b00000000
#define LORA_BANDWIDTH_10_4kHz   0b00010000
#define LORA_BANDWIDTH_15_6kHz   0b00100000
#define LORA_BANDWIDTH_20_8kHz   0b00110000
#define LORA_BANDWIDTH_31_25kHz  0b01000000
#define LORA_BANDWIDTH_41_7kHz   0b01010000
#define LORA_BANDWIDTH_62_5kHz   0b01100000
#define LORA_BANDWIDTH_125kHz    0b01110000
#define LORA_BANDWIDTH_250kHz    0b10000000
#define LORA_BANDWIDTH_500kHz    0b10010000

#define LORA_CODING_RATE_4_5              0b00000010
#define LORA_CODING_RATE_4_6              0b00000100
#define LORA_CODING_RATE_4_7              0b00000110
#define LORA_CODING_RATE_4_8              0b00001000

#define LORA_IMPLICIT_HEADER     0b00000001
#define LORA_EXPLICIT_HEADER     0b00000000

//config 2
#define LORA_SPREADING_FACTOR_6         0b01100000
#define LORA_SPREADING_FACTOR_7         0b01110000
#define LORA_SPREADING_FACTOR_8         0b10000000
#define LORA_SPREADING_FACTOR_9         0b10010000
#define LORA_SPREADING_FACTOR_10        0b10100000
#define LORA_SPREADING_FACTOR_11        0b10110000
#define LORA_SPREADING_FACTOR_12        0b11000000
#define LORA_TX_CONTINUOUS_MODE_ON      0b00001000
#define LORA_TX_CONTINUOUS_MODE_OFFT    0b00000000
//#define LORA_SYMB_TIMEOUT               0b00000000
#define LORA_RX_PAYLOAD_CRC_ON          0b00000100
#define LORA_RX_PAYLOAD_CRC_OFF         0b00000000

//config 3
#define LORA_LOW_DATA_RATE_OPT_OFF   0b00000000
#define LORA_LOW_DATA_RATE_OPT_ON    0b00001000
#define LORA_AGC_AUTO_OFF            0b00000000
#define LORA_AGC_AUTO_ON             0b00000100

//modes
#define SX1278_SLEEP                0b00000000
#define SX1278_STANDBY              0b00000001
#define SX1278_TX                   0b00000011
#define SX1278_LORA                 0b10000000

//power config
#define SX1278_PA_SELECT_BOOST   0b10000000
#define SX1278_PA_SELECT_RFO     0b00000000
//overcurrent protection
#define OCP_OFF                                 0b00000000
#define OCP_ON                                  0b00100000
#define OCP_TRIM                                0b00001011

//hopping config
#define HOP_PERIOD_OFF                          0b00000000

//Detection optimize
#define DETECT_OPTIMIZE_SF_7_12                 0b00000011
//Detection treshold
#define DETECTION_THRESHOLD_SF_7_12             0b00001010

//pinmaps
#define DIO0_LORA_TX_DONE                       0b01000000

#define FIFO_TX_BASE_ADDR_MAX                   0b00000000


//SX1278 registers
#define REG_FIFO                                0x00
#define REG_OP_MODE                             0x01
#define REG_FRF_MSB                             0x06
#define REG_FRF_MID                             0x07
#define REG_FRF_LSB                             0x08
#define REG_PA_CONFIG                           0x09
#define REG_OCP                                 0x0B
#define REG_FIFO_ADDR_PTR                       0x0D
#define REG_FIFO_TX_BASE_ADDR                   0x0E
#define REG_IRQ_FLAGS                           0x12
#define REG_MODEM_CONFIG_1                      0x1D
#define REG_MODEM_CONFIG_2                      0x1E
#define REG_PREAMBLE_MSB                        0x20
#define REG_PREAMBLE_LSB                        0x21
#define REG_PAYLOAD_LENGTH                      0x22
#define REG_HOP_PERIOD                          0x24
#define REG_MODEM_CONFIG_3                      0x26
#define REG_DETECT_OPTIMIZE                     0x31
#define REG_DETECTION_THRESHOLD                 0x37
#define REG_SYNC_WORD                           0x39
#define REG_DIO_MAPPING_1                       0x40
#define REG_DIO_MAPPING_2                       0x41
#define REG_VERSION                             0x42
#define REG_PA_DAC                              0x4D


//hopping config
#define HOP_PERIOD_OFF                          0b00000000

//Detection optimize
#define DETECT_OPTIMIZE_SF_7_12                 0b00000011
//Detection treshold
#define DETECTION_THRESHOLD_SF_7_12             0b00001010

//pinmaps
#define DIO0_LORA_TX_DONE                       0b01000000

#define FIFO_TX_BASE_ADDR_MAX                   0b00000000


#define SX1278_READ  0
#define SX1278_WRITE 1


class SX127X {
private:
    uint8_t chip_version = 0;
    /* data */
    //TODO SPIClass *spi;
    uint8_t dio0 = 0;
    uint8_t cs   = 0;
    uint8_t rst  = 0;

    uint8_t input;
    uint8_t output;
    uint8_t high;
    uint8_t low;

    uint8_t has_pin_mode  = false;
    uint8_t has_pin_write = false;
    uint8_t has_delay     = false;
    uint8_t has_start     = false;
    uint8_t has_end       = false;
    uint8_t has_transfer  = false;

    uint8_t sf  = 7;    //spreading factor
    float bw    = 125;  //bandwidth in kHz
    float ts;           //symbol period/time on air (ms)


    void (*pinMode)(uint8_t pin, uint8_t mode);
    void (*pinWrite)(uint8_t pin, uint8_t lvl);
    void (*delay)(uint32_t delay_ms);

    void (*startTransfer)();
    void (*endTransfer)();
    uint8_t (*spiTransfer)(uint8_t*, uint8_t*, uint8_t);

public:
    //SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    ~SX127X();


    //TODO
    void registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output);
    //TODO
    void registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high, uint8_t low);
    //TODO
    void registerDelay(void (*func)(uint32_t));

    //TODO
    void startSpiTransfer(void (*func)());
    //TODO
    void endSpiTransfer(void (*func)());
    //TODO
    void registerSpiTransfer(uint8_t (*func)(uint8_t*, uint8_t*, uint8_t));


    //TODO
    uint8_t begin(uint32_t frequency, uint8_t sync_word, uint16_t preamble_len);

    //TODO
    void reset();

    //TODO
    uint8_t getVersion();

    //TODO
    void setMode(uint8_t mode);

    //TODO
    void setModemMode(uint8_t modem);

    //TODO
    void setFrequencyHopping(uint8_t period);

    //TODO
    void setSyncWord(uint8_t sync_word);

    //TODO
    void setCurrentLimit();

    //TODO
    uint8_t setPreambleLength(uint16_t preamble_length);

    //WARNING check
    /** @brief Set bandwidth. If needed, enable/disable low data rate optimalization
     * 
     * @param bandwidth LORA_BANDWIDTH_... macros
     * @return TODO if invalid bandwidth is provided
     */
    uint8_t setBandwidth(uint8_t bandwidth);
    //TODO
    uint32_t getBandwidth();

    //WARNING check
    /**
     * @brief Set spreading factor. If needed, enable/disable low data rate optimalization
     * 
     * @param spreading_factor LORA_SPREADING_FACTOR_... macros
     * @return TODO if invalid bandwidth is provided
     */
    uint8_t setSpreadingFactor(uint8_t spreading_factor);

    //TODO
    void setFrequency(uint32_t frequency);

    //TODO
    void setCodingRate();

    //TODO
    void setGain();

    //TODO
    void setCRC();


    //TODO
    void setLowDataOptimalization();
    //TODO
    void setPower();


    //TODO
    uint8_t readRegister(uint8_t addr);
    //TODO
    void writeRegister(uint8_t addr, uint8_t data);
    //TODO
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);

    //TODO
    uint8_t transmit(uint8_t *data, uint8_t length);

    //TODO
    /** @brief Polling data receive
     * 
     * @param data array to which to store the data
     * @return 
     */
    uint8_t receive(uint8_t* data);
};
