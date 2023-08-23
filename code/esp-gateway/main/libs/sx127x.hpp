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

//TODO check chip versions
//TODO unify naming scheme (chip/module)
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

//TODO unify naming scheme of values (LoRa, FSK, SX127X)
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


//TODO finish registers
//SX127X registers
#define REG_FIFO                                0x00
#define REG_OP_MODE                             0x01
//                                              0x02
//                                              0x03
//                                              0x04
//                                              0x05
#define REG_FRF_MSB                             0x06
#define REG_FRF_MID                             0x07
#define REG_FRF_LSB                             0x08
#define REG_PA_CONFIG                           0x09
//                                              0x0A
#define REG_OCP                                 0x0B
//                                              0x0C
#define REG_FIFO_ADDR_PTR                       0x0D
#define REG_FIFO_TX_BASE_ADDR                   0x0E
//                                              0x0F
//                                              0x10
//                                              0x11
#define REG_IRQ_FLAGS                           0x12
//                                              0x13
//                                              0x14
//                                              0x15
//                                              0x16
//                                              0x17
//                                              0x18
//                                              0x19
//                                              0x1A
//                                              0x1B
//                                              0x1C
#define REG_MODEM_CONFIG_1                      0x1D
#define REG_MODEM_CONFIG_2                      0x1E
//                                              0x1F
#define REG_PREAMBLE_MSB                        0x20
#define REG_PREAMBLE_LSB                        0x21
#define REG_PAYLOAD_LENGTH                      0x22
//                                              0x23
#define REG_HOP_PERIOD                          0x24
//                                              0x25
#define REG_MODEM_CONFIG_3                      0x26
//                                              0x27
//                                              0x28
//                                              0x29
//                                              0x2A
//                                              0x2B
//                                              0x2C
//                                              0x2D
//                                              0x2E
//                                              0x2F
//                                              0x30
#define REG_DETECT_OPTIMIZE                     0x31
//                                              0x32
//                                              0x33
//                                              0x34
//                                              0x35
//                                              0x36
#define REG_DETECTION_THRESHOLD                 0x37
//                                              0x38
#define REG_SYNC_WORD                           0x39
//                                              0x3A
//                                              0x3B
//                                              0x3C
//                                              0x3D
//                                              0x3E
//                                              0x3F
#define REG_DIO_MAPPING_1                       0x40
#define REG_DIO_MAPPING_2                       0x41
#define REG_VERSION                             0x42
//                                              0x43
//                                              0x44
//                                              0x45
//                                              0x46
//                                              0x47
//                                              0x48
//                                              0x49
//                                              0x4A
//                                              0x4B
//                                              0x4C
#define REG_PA_DAC                              0x4D
//                                              0x4E
//                                              0x4F


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


#define SX127X_READ_MASK  0b01111111
#define SX127X_WRITE_MASK 0b10000000


//ERRORS
#define ERR_INVALID_PREAMBLE_LEN     1
#define ERR_INVALID_BANDWIDTH        2
#define ERR_INVALID_MODEM_MODE       3
#define ERR_INVALID_SPREADING_FACTOR 4
#define ERR_INVALID_CODING_RATE      5


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
    
    //TODO rename
    //TODO constructor?
    union Bits {
        char all;
        struct bits {
            uint8_t has_pin_mode      :1;
            uint8_t has_pin_write     :1;
            uint8_t has_pin_read      :1;
            uint8_t has_delay         :1;
            uint8_t has_spi_start_tr  :1;
            uint8_t has_spi_end_tr    :1;
            uint8_t has_transfer      :1;
            uint8_t has_burstTransger :1;
        } single;
        Bits() {bits{false, false, false, false, false, false, false, false};}
    } flags;

    //TODO store them raw or as human readable?
    uint8_t sf  = 7;    //spreading factor
    float bw    = 125;  //bandwidth in kHz
    float ts;           //symbol period/time on air (ms)
    uint8_t cr  = 5;    //coding rate


    void (*pinMode)(uint8_t pin, uint8_t mode);
    void (*pinWrite)(uint8_t pin, uint8_t lvl);
    uint8_t (*pinRead)(uint8_t pin);
    void (*delay)(uint32_t delay_ms);

    void (*SPIStartTransaction)();
    void (*SPIEndTransaction)();
    uint8_t (*spiTransfer)(uint8_t data);

public:
    //SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    ~SX127X();


    //TODO
    void registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output);
    //TODO
    void registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high, uint8_t low);
    //TODO
    void registerPinRead(uint8_t (*func)(uint8_t));
    //TODO
    void registerDelay(void (*func)(uint32_t));

    //TODO
    void registerSPIStartTransaction(void (*func)());
    //TODO
    void registerSPIEndTransaction(void (*func)());
    //TODO
    void registerSpiTransfer(uint8_t (*func)(uint8_t));


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
    //TODO description
    /** @brief Set the frequency hopping period
     * @attention Hopping period (time between channel change) is then defined as Ts*frequency hopping period. Ts = symbol period
     * @param period period in ms in which to change the 
     */
    void setFrequencyHopping(uint8_t period);

    //DONE
    /** @brief Set the sync word for keeping modules on different "networks"
     * 
     * @param sync_word desired sync word. Can be anything. 0x12/0x1424 is default. 0x34/0x3444 is reserved for LoRaWAN.
     */
    void setSyncWord(uint8_t sync_word);

    //TODO
    /** @brief Set the current limit of the module
     * 
     */
    void setCurrentLimit();

    //DONE
    //TODO FSK
    /** @brief Set the preamble length used to syncrhonize receiver with the incoming data
     * 
     * @param preamble_length preamble length between 6 and 65535
     * @return 0 on success. ERR_INVALID_PREAMBLE_LEN if preamble length is less than 6.
     */
    uint8_t setPreambleLength(uint16_t preamble_length);

    //DONE
    /** @brief Set bandwidth. If needed, enable/disable low data rate optimalization
     * 
     * @param bandwidth desired bandwidth
     * @param LORA_BANDWIDTH_7_8kHz
     * @param LORA_BANDWIDTH_10_4kHz
     * @param LORA_BANDWIDTH_15_6kHz
     * @param LORA_BANDWIDTH_20_8kHz
     * @param LORA_BANDWIDTH_31_25kHz
     * @param LORA_BANDWIDTH_41_7kHz
     * @param LORA_BANDWIDTH_62_5kHz
     * @param LORA_BANDWIDTH_125kHz
     * @param LORA_BANDWIDTH_250kHz
     * @param LORA_BANDWIDTH_500kHz
     * @return 0 on success. ERR_INVALID_MODEM_MODE if not in LoRa mode. ERR_INVALID_BANDWIDTH if invalid bandwidth is provided.
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
     * @param spreading_factor desired spreading factor
     * @param LORA_SPREADING_FACTOR_6 
     * @param LORA_SPREADING_FACTOR_7 
     * @param LORA_SPREADING_FACTOR_8 
     * @param LORA_SPREADING_FACTOR_9 
     * @param LORA_SPREADING_FACTOR_10
     * @param LORA_SPREADING_FACTOR_11
     * @param LORA_SPREADING_FACTOR_12
     * @return 0 on success. ERR_INVALID_MODEM_MODE if not in LoRa mode. ERR_INVALID_SPREADING_FACTOR if invalid spreading factor is provided.
     */
    uint8_t setSpreadingFactor(uint8_t spreading_factor);

    //DONE
    /** @brief Set the module radio frequency
     * 
     * @param frequency Frequency in MHz
     * @return 0 on success, ERR_INVALID_FREQUENCY if invalid frequency is provided for current module version
     */
    uint8_t setFrequency(uint16_t frequency);

    //DONE
    /** @brief Set the coding rate
     * 
     * @param coding_rate desired coding rate
     * @param LORA_CODING_RATE_4_5
     * @param LORA_CODING_RATE_4_6
     * @param LORA_CODING_RATE_4_7
     * @param LORA_CODING_RATE_4_8
     * @return 0 on success. ERR_INVALID_MODEM_MODE if not in LoRa mode. ERR_INVALID_CODING_RATE if invalid coding rate is provided.
     */
    uint8_t setCodingRate(uint8_t coding_rate);

    //TODO
    uint8_t setGain(uint8_t gain);

    //DONE
    /** @brief Enable or disable CRC 
     * 
     * @param enable 
     */
    void setCRC(bool enable);


    //DONE
    /** @brief Enable low data rate optimalization if symbol length exceeds 16ms, disable otherwise.
    *
    */
    void setLowDataRateOptimalization();

    //TODO
    void setPower();


    //TODO test
    /** @brief Make entire SPI transaction 
     * 
     * @param addr Address to read from/start to read from
     * @param data Data to send/receive
     * @param length Length of data to send/receive
     */
    void SPIMakeTransaction(uint8_t addr, uint8_t *data, size_t length = 1);

    //TODO merge burst and single?

    //TODO test
    uint8_t readRegister(uint8_t addr, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);
    void readRegistersBurst(uint8_t addr, uint8_t *data, size_t length);

    //TODO test
    void writeRegister(uint8_t addr, uint8_t data);
    void writeRegistersBurst(uint8_t addr, uint8_t *data, size_t length);

    //TODO test
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
