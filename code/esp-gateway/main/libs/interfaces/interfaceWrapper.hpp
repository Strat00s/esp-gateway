#pragma once
#include <stdio.h>

//interface type

enum IfTypes {
    IF_TYPE_NONE,
    IF_TYPE_MQTT,
    IF_TYPE_SX127X,
    IF_TYPE_LORA_2_4G,
    IF_TYPE_NRF24,
    IF_TYPE_RF_443,
    IF_TYPE_ESP_NOW,
};


class InterfaceWrapper {
private:
    IfTypes if_type = IF_TYPE_NONE;

protected:
    uint32_t last_ret = 0;

public:
    InterfaceWrapper(IfTypes interface_type) {
        this->if_type = interface_type;
    }
    ~InterfaceWrapper() {};

    void setType(IfTypes interface_type) {
        this->if_type = interface_type;
    }
    IfTypes getType() {
        return this->if_type;
    }


    virtual uint8_t begin() = 0;

    /** @brief Send data to interface and transmit them.
     * 
     * @param data Data to be transmitted.
     * @param len Data lenght.
     * @return 0 on success.
     * Other values are interface specific.
     */
    virtual uint8_t sendData(uint8_t *data, uint8_t len) = 0;

    /** @brief Get data from interface.
     * 
     * @param data Buffer for storing received data.
     * @param len Length of the buffer.
     * Overwriten with the recieved data length that can fit to the buffer.
     * @return 0 on succes.
     */
    virtual uint8_t getData(uint8_t *data, uint8_t *len) = 0;

    /** @brief Check if interface has any new data.
     *
     * @return Number of received data (packets).
     */
    virtual uint8_t hasData() = 0;


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

    /** @brief Get interface specific status.
     * Should return at least the last return value of any previously run function.
     * Preferably should return more specific status information.
     * 
     * @return Interface specific status code.
     */
    virtual uint32_t getStatus() {
        return last_ret;
    }
};
