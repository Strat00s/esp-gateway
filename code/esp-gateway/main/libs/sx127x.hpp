/**
 * @file SX127X.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief SX127X header file
 * @version 0.2
 * @date 23-08-2023
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

//TODO unify naming scheme (chip/module)
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

//LNA gains
#define SX127X_LNA_GAIN_AUTOMATIC 0b00000000
#define SX127X_LNA_GAIN_G1        0b00100000   //highest gain
#define SX127X_LNA_GAIN_G2        0b01000000
#define SX127X_LNA_GAIN_G3        0b01100000
#define SX127X_LNA_GAIN_G4        0b10000000
#define SX127X_LNA_GAIN_G5        0b10100000
#define SX127X_LNA_GAIN_G6        0b11000000

#define SX127X_LNA_GAIN_0DB  0b00100000 //highest gain
#define SX127X_LNA_GAIN_6DB  0b01000000
#define SX127X_LNA_GAIN_12DB 0b01100000
#define SX127X_LNA_GAIN_24DB 0b10000000
#define SX127X_LNA_GAIN_36DB 0b10100000
#define SX127X_LNA_GAIN_48DB 0b11000000

//power config
#define SX127X_PA_SELECT_BOOST 0b10000000
#define SX127X_PA_SELECT_RFO   0b00000000
#define SX127X_PA_BOOST_OFF    0b00000100
#define SX127X_PA_BOOST_ON     0b00000111

//overcurrent protection
#define SX127X_OCP_OFF  0b00000000
#define SX127X_OCP_ON   0b00100000
#define SX127X_OCP_TRIM 0b00001011

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
#define REG_LNA                                 0x0C
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
#define ERR_INVALID_CURRENT_LIMIT    6
#define ERR_INVALID_GAIN             7
#define ERR_INVALID_POWER            8
#define ERR_MISSING_CALLBACK         9
#define ERR_INVALID_CHIP_VERSION     10 //wrong chip version was read. Check you connection.


class SX127X {
private:
    uint8_t chip_version = 0;

    uint8_t dio0 = 0;
    uint8_t cs   = 0;
    uint8_t rst  = 0;

    uint8_t input;
    uint8_t output;
    uint8_t high;
    uint8_t low;
    
    union Bits {
        char all;
        struct bits {
            uint8_t has_pin_mode       :1;
            uint8_t has_pin_write      :1;
            uint8_t has_pin_read       :1;
            uint8_t has_delay          :1;
            uint8_t has_spi_start_tr   :1;
            uint8_t has_spi_end_tr     :1;
            uint8_t has_transfer       :1;
            uint8_t has_burst_transfer :1;
        } single;
        Bits() {bits{false, false, false, false, false, false, false, false};}
    } flags;


    uint8_t sf  = LORA_SPREADING_FACTOR_7 >> 4;// 7;    //spreading factor
    float bw    = 125;  //bandwidth in kHz


    void (*pinMode)(uint8_t pin, uint8_t mode);
    void (*pinWrite)(uint8_t pin, uint8_t lvl);
    uint8_t (*pinRead)(uint8_t pin);
    void (*delay)(uint32_t delay_ms);

    void (*SPIBeginTransaction)();
    void (*SPIEndTransaction)();
    /** @brief To be implemented by user. Transfer function for sending and receiveing data over SPI
     *
     * @param addr Register address. MSB of the address defines if it is read or write transaction (0 for read, 1 for write)
     * @param buffer Buffer for in/out data (contains data to be written or will be filled with read data)
     * @param length Length of buffer
     */
    void (*spiTransfer)(uint8_t addr, uint8_t *buffer, size_t length);

public:
    //SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    SX127X(uint8_t cs, uint8_t rst, uint8_t dio0);
    ~SX127X();

    void registerPinMode(void (*func)(uint8_t, uint8_t), uint8_t input, uint8_t output);
    void registerPinWrite(void (*func)(uint8_t, uint8_t), uint8_t high = 1, uint8_t low = 0);
    void registerPinRead(uint8_t (*func)(uint8_t));
    void registerDelay(void (*func)(uint32_t));

    void registerSPIStartTransaction(void (*func)());
    void registerSPIEndTransaction(void (*func)());
    void registerSpiTransfer(void (*func)(uint8_t, uint8_t *, size_t));


    /** @brief Initialize module to it's default settings
     * 
     * @param frequency Desired radio frequency in MHz
     * @param sync_word sync word for keeping modules on different "networks"
     * @param preamble_len length of the preamble
     * @return uint8_t 
     */
    uint8_t begin(uint16_t frequency, uint8_t sync_word, uint16_t preamble_len);


    /** @brief Reset the module*/
    void reset();

    /** @brief Get the module version
     * 
     * @return Module version stored in register 
     */
    uint8_t getVersion();

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

    /** @brief Set operation mode of the modem (LoRa or FSK/OOK)
     * 
     * @param modem mode mode
     * @param SX127X_MODEM_FSK_OOK
     * @param SX127X_MODEM_LORA
     */
    void setModemMode(uint8_t modem);

    /** @brief Get modem mode
     * 
     * @return Current modem mdoe  
     */
    uint8_t getModemMode();


    /** @brief Set the sync word for keeping modules on different "networks"
     * 
     * @param sync_word desired sync word. Can be anything. 0x12/0x1424 is default. 0x34/0x3444 is reserved for LoRaWAN.
     */
    void setSyncWord(uint8_t sync_word);

    /** @brief Set the preamble length used to syncrhonize receiver with the incoming data
     * 
     * @param preamble_length preamble length between 6 and 65535
     * @return 0 on success. ERR_INVALID_PREAMBLE_LEN if preamble length is less than 6.
     */
    uint8_t setPreambleLength(uint16_t preamble_length);


    /** @brief Set the module radio frequency
     * 
     * @param frequency Frequency in MHz
     * @return 0 on success, ERR_INVALID_FREQUENCY if invalid frequency is provided for current module version
     */
    uint8_t setFrequency(uint16_t frequency);

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

    /** @brief Set the receiver Low-Noise Amplifier gain. G1 is the highest and G6 is the lowest.
     * 
     * @param gain Desired gain setting. Automatic is recommended (default)
     * @param SX127X_LNA_GAIN_AUTOMATIC
     * @param SX127X_LNA_GAIN_G1
     * @param SX127X_LNA_GAIN_G2
     * @param SX127X_LNA_GAIN_G3
     * @param SX127X_LNA_GAIN_G4
     * @param SX127X_LNA_GAIN_G5
     * @param SX127X_LNA_GAIN_G6
     * @return uint8_t 
     */
    uint8_t setGain(uint8_t gain);

    /** @brief Enable or disable CRC 
     * 
     * @param enable 
     */
    uint8_t setCRC(bool enable);

    /** @brief Enable low data rate optimalization if symbol length exceeds 16ms, disable otherwise.
    *
    */
    void setLowDataRateOptimalization();

    //TODO proper description
    /** @brief Set the frequency hopping period
     * @attention Hopping period (time between channel change) is then defined as Ts*frequency hopping period. Ts = symbol period
     * @param period period in ms(???) in which to change the band
     */
    void setFrequencyHopping(uint8_t period);


    /** @brief Set the current limit of the module's power amplifier. Minimum is 45mA, maximum 240mA.
     *  5mA steps between 45mA to 120mA. 10mA steps between 120mA and 240mA. Set to 0 to disable overload current protection.
     * 
     * @param max_current maximum current drain of the power amplifier (if the module has one).
     * @return ERR_INVALID_CURRENT_LIMIT if invalid current value is provided
     */
    uint8_t setCurrentLimit(uint8_t max_current);

    /** @brief Set the module output power. Some modules do not have RFO pin connected and only use PA_BOOSt pin.
     * 
     * @param power Desired power from -4dBm to 15dBm when using RFO. From 2dBm to 17dBm or 20dBm when using PA_BOOST.
     * @param pa_boost Wheter to use PA_BOOST pin or RFO pin.
     * @return 0 on success. ERR_INVALID_POWER when invalid power for specified pin is provided.
     */
    uint8_t setPower(int8_t power, bool pa_boost = true);



    //TODO finish
    /** @brief Transmit data and wait for the transmission to finish
     * 
     * @param data Data buffer
     * @param length Length of data to be sent (max 256B)
     * @return uint8_t //TODO return value
     */
    uint8_t transmit(uint8_t *data, uint8_t length);

    //TODO finish
    /** @brief Polling data receive
     * 
     * @param data Buffer to which to store the data (must be at least as long as the received data length)
     * @return //TODO return value
     */
    uint8_t receive(uint8_t* data);


    /** @brief Make entire SPI transaction 
     * 
     * @param addr Address to read from/start to read from
     * @param data Data to send/receive
     * @param length Length of data to send/receive
     */
    void SPIMakeTransaction(uint8_t addr, uint8_t *data, size_t length = 1);
    
    /** @brief Read single register from the module. If only specific range change is required, use mask_xsb arguments to specify a mask of bits which will be kept from the register
     * 
     * @param addr Register address to be read from
     * @param mask_lsb LSB mask bit
     * @param mask_msb MSB mask bit
     * @return Data stored in the register
     */
    uint8_t readRegister(uint8_t addr, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);
    
    /** @brief Read multiple registers one after another
     * 
     * @param addr Starting address (is automatically incremented)
     * @param data Buffer for storing read data
     * @param length Length of the data buffer (how many registers to read from)
     */
    void readRegistersBurst(uint8_t addr, uint8_t *data, size_t length);
    
    /** @brief Write data to register
     * 
     * @param addr Register address to be writen to
     * @param data Data to be writen
     */
    void writeRegister(uint8_t addr, uint8_t data);
    
    /** @brief Write multiple reigster one after another
     * 
     * @param addr Starting address (is automatically incremented)
     * @param data Buffer to be writen to the registers
     * @param length Length of the data buffer (how many register to write to)
     */
    void writeRegistersBurst(uint8_t addr, uint8_t *data, size_t length);
    
    /** @brief Set register to a specific value. If only a specific range change is required, use mask_xsb arguments to specify a mask o bits which will be overwriten in the register (the rest of the register will remain the same).
     * 
     * @param addr Register address to set
     * @param data Data to write
     * @param mask_lsb LSB mask bit
     * @param mask_msb MSB mask bit
     */
    void setRegister(uint8_t addr, uint8_t data, uint8_t mask_lsb = 0, uint8_t mask_msb = 7);
};
