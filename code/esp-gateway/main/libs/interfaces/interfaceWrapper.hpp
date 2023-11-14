#pragma once
#include <stdio.h>

//interface type

#define IW_TYPE_MQTT     0
#define IW_TYPE_LORA_434 1
#define IW_TYPE_LORA_868 2
#define IW_TYPE_LORA_2_4 3
#define IW_TYPE_NRF24    4
#define IW_TYPE_RF_443   5
#define IW_TYPE_ESP_NOW  6


class interfaceWrapper {
private:
    /* data */
    uint8_t i_type;
public:
    interfaceWrapper(uint8_t interface_type) {
        this->i_type = interface_type;
    }
    ~interfaceWrapper();

    void setType(uint8_t interface_type) {
        this->i_type = interface_type;
    }
    uint8_t getType() {
        return this->i_type;
    }

    virtual uint8_t transmitData(uint8_t *data, uint8_t len) = 0;
    virtual uint8_t getData(uint8_t *data, uint8_t *len) = 0;
    virtual uint8_t startReception() = 0;
    virtual uint8_t hasData() = 0;
};
