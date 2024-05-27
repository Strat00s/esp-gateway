#pragma once
#include <stdio.h>


#define IF_TYPE_NONE       0
//#define IF_TYPE_MQTT
#define IF_TYPE_SX127X     1
//#define IF_TYPE_LORA_2_4G
//#define IF_TYPE_NRF24
//#define IF_TYPE_RF_443
//#define IF_TYPE_ESP_NOW
#define IF_TYPE_MULTI 99

#define IFW_ALLOC_FAILED  255
#define IFW_ONGOING_RX    254
#define IFW_OK            0


class InterfaceWrapperBase {
private:
    uint8_t if_type = IF_TYPE_NONE;

public:
    InterfaceWrapperBase(uint8_t interface_type) {
        if_type = interface_type;
    }
    //~InterfaceWrapperBase() {};

    inline uint8_t getType() {
        return if_type;
    }


    /** @brief Start reception on interface (if supported)
     * 
     * @return 0 on success.
     */
    virtual uint8_t startReception() = 0;

    /** @brief Stop reception on interface (if supported)
     * 
     * @return 0 on success.
     */
    virtual uint8_t stopReception() = 0;

    /** @brief Check if transmission medium is currently free (if interface supports it and is implemented)
     * 
     * @return True if free, false otherwise.
     */
    virtual bool isMediumBusy() = 0;

    /** @brief Send data to interface and transmit them.
     * 
     * @param data Data to be transmitted.
     * @param len Data lenght.
     * @param copy Create a copy of the data which will be sent instead of the data.
     * @return 0 on success.
     * 255 if allocation fails when copying data.
     */
    virtual uint8_t sendData(uint8_t *data, uint8_t len) = 0;

    /** @brief Get data from interface.
     * 
     * @param data Buffer for storing received data.
     * @param len Length of the buffer.
     * Overwriten with the recieved data length that can fit to the buffer.
     * @return 0 on success.
     */
    virtual uint8_t getData(uint8_t *data, uint8_t *len) = 0;

    /** @brief Check if interface has any new data.
     *
     * @return Number of received data (packets).
     */
    virtual uint8_t hasData() = 0;

    virtual double getTimeOnAir(uint8_t payload_length) = 0;

    virtual bool reset() = 0;
};
