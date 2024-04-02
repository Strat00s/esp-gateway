#pragma once
#include <stdio.h>


#define IF_TYPE_NONE       0
//#define IF_TYPE_MQTT
#define IF_TYPE_SX127X     1
//#define IF_TYPE_LORA_2_4G
//#define IF_TYPE_NRF24
//#define IF_TYPE_RF_443
//#define IF_TYPE_ESP_NOW


class InterfaceWrapper {
private:
    uint8_t if_type = IF_TYPE_NONE;

protected:
    uint32_t last_ret = 0;

public:
    InterfaceWrapper(uint8_t interface_type) {
        this->if_type = interface_type;
    }
    ~InterfaceWrapper() {};

    inline uint8_t getType() {
        return this->if_type;
    }


    /** @brief Start reception on interface (if supported)
     * 
     * @return True on success.
     */
    virtual bool startReception() = 0;

    /** @brief Stop reception on interface (if supported)
     * 
     * @return True on success.
     */
    virtual bool stopReception() = 0;

    /** @brief Get interface specific status.
     * Should return at least the last return value of any previously run function.
     * Preferably should return more specific status information.
     * 
     * @return Last stored return value from interface.
     */
    virtual uint32_t getStatus() {
        return last_ret;
    }

    virtual void clearStatus() {
        last_ret = 0;
    }


    /** @brief Send data to interface and transmit them.
     * 
     * @param data Data to be transmitted.
     * @param len Data lenght.
     * @return True on success, false otherwise.
     */
    virtual bool sendData(uint8_t *data, uint8_t len) = 0;

    /** @brief Get data from interface.
     * 
     * @param data Buffer for storing received data.
     * @param len Length of the buffer.
     * Overwriten with the recieved data length that can fit to the buffer.
     * @return True on success, false otherwise.
     */
    virtual bool getData(uint8_t *data, uint8_t *len) = 0;

    /** @brief Check if interface has any new data.
     *
     * @return Number of received data (packets).
     */
    virtual uint8_t hasData() = 0;

};
