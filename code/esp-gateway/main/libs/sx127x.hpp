/**
 * @file SX127X.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX127X header file
 * @version 0.1
 * @date 24-06-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */


//TODO unify macro names
//TODO test all functions
//TODO all registers
//TODO return types
//TODO FSK
//TODO somehow add possible values to all arguments
//TODO comments for every function

#pragma once
#include <stdio.h>

/*----(register fields)----*/
//SX127X chip version
#define SX1272_CHIP_VERSION 0x22
#define SX1273_CHIP_VERSION 0x22

#define SX1276_CHIP_VERSION 0x12
#define SX1277_CHIP_VERSION 0x12
#define SX127X_CHIP_VERSION 0x12
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

//operation modes
#define SX127X_OP_MODE_SLEEP        0b00000000
#define SX127X_OP_MODE_STANDBY      0b00000001
#define SX127X_OP_MODE_FSTX         0b00000010
#define SX127X_OP_MODE_TX           0b00000011
#define SX127X_OP_MODE_FSRX         0b00000100
#define SX127X_OP_MODE_RXCONTINUOUS 0b00000101
#define SX127X_OP_MODE_RXSINGLE     0b00000110
#define SX127X_OP_MODE_CAD          0b00000111

//modem modes
#define SX127X_MODEM_FSK_OOK    0b00000000
#define SX127X_MODEM_LORA       0b10000000


//power config
#define SX127X_PA_SELECT_BOOST   0b10000000
#define SX127X_PA_SELECT_RFO     0b00000000
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


//SX127X registers
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

//detection threashold (for spreading factor only)
#define LORA_DETECTION_THRESHOLD_SF6    0x0C
#define LORA_DETECTION_THRESHOLD_SF7_12 0x0A
//detection optimizalization (for spreading factor only)
#define LORA_DETECTION_OPTIMIZE_SF6    0x05
#define LORA_DETECTION_OPTIMIZE_SF7_12 0x03


#define SX127X_READ  0
#define SX127X_WRITE 1


class SX127X {
private:
    uint8_t chip_version = 0;   //TODO store it?

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

    //TODO store them raw or as human readable?
    uint8_t sf  = 7;    //spreading factor
    float bw    = 125;  //bandwidth in kHz
    float ts;           //symbol period/time on air (ms)
    uint8_t cr  = 5;    //coding rate


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


    //TODO finish
    //TODO frequency range check for each module
    /** @brief Initialize module to it's default settings
     * 
     * @param frequency Desired radio frequency in MHz
     * @param sync_word sync word for keeping modules on different "networks"
     * @param preamble_len length of the preamble
     * @return uint8_t 
     */
    uint8_t begin(uint16_t frequency, uint8_t sync_word, uint16_t preamble_len);

    //DONE
    /** @brief Reset the module*/
    void reset();

    //DONE
    /** @brief Get the module version
     * 
     * @return Module version stored in register 
     */
    uint8_t getVersion();

    //DONE
    /** @brief Set operation mode of the module
     * 
     * @param mode operation mode
     * @param SX127X_OP_MODE_SLEEP
     * @param SX127X_OP_MODE_STANDBY
     * @param SX127X_OP_MODE_FSTX
     * @param SX127X_OP_MODE_TX
     * @param SX127X_OP_MODE_FSRX
     * @param SX127X_OP_MODE_RXCONTINUOUS
     * @param SX127X_OP_MODE_RXSINGLE
     * @param SX127X_OP_MODE_CAD
     */
    void setMode(uint8_t mode);

    //DONE
    /** \brief Set operation mode of the modem (LoRa or FSK/OOK)
     * 
     * @param modem mode mode
     * @param SX127X_MODEM_FSK_OOK
     * @param SX127X_MODEM_LORA
     */
    void setModemMode(uint8_t modem);

    //DONE
    uint8_t getModemMode();

    //DONE ???
    /** @brief Set the frequency hopping period
     * @attention Hopping period (time between channel change) is then defined as Ts*frequency hopping period. Ts = symbol period
     * @param period period in ms in which to change the 
     */
    void setFrequencyHopping(uint8_t period);

    //DONE
    /** @brief Set the sync word
     * 
     * @param sync_word sync word for keeping modules on different "networks"
     */
    void setSyncWord(uint8_t sync_word);

    //TODO
    /** @brief Set the Current Limit object
     * 
     */
    void setCurrentLimit();

    //DONE
    //TODO FSK
    /** @brief Set the preamble length
     * 
     * @param preamble_length 
     * @return 1 if invalid preamble length given //TODO return types
     */
    uint8_t setPreambleLength(uint16_t preamble_length);

    //DONE
    /** @brief Set bandwidth. If needed, enable/disable low data rate optimalization
     * 
     * @param bandwidth LORA_BANDWIDTH_... macros
     * @return //TODO if invalid bandwidth is provided
     */
    uint8_t setBandwidth(uint8_t bandwidth);
    
    //DONE
    /** @brief Get configured bandwidth
    * 
    * @return Bandwidth in Hz
    */
    uint32_t getBandwidth();

    //DONE
    /** @brief Set spreading factor. If needed, enable/disable low data rate optimalization
     * 
     * @param spreading_factor LORA_SPREADING_FACTOR_... macros
     * @return TODO if invalid bandwidth is provided
     */
    uint8_t setSpreadingFactor(uint8_t spreading_factor);

    //DONE
    /** @brief Set the module radio frequency
     * 
     * @param frequency Frequency in MHz
     */
    uint8_t setFrequency(uint16_t frequency);

    //DONE
    uint8_t setCodingRate(uint8_t coding_rate);

    //TODO
    void setGain();

    //DONE
    void setCRC(bool enable);


    //DONE
    /** @brief Enable low data rate optimalization if symbol length exceeds 16ms, disable otherwise.
    *
    */
    void setLowDataRateOptimalization();

    //TODO
    void setPower();


    //TODO
    //TODO add masking
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
