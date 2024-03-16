#pragma once
#include <stdio.h>
#include "InterfaceWrapper.hpp"
#include "libs/sx127x.hpp"


class lora24InterfaceWrapper : public ifWrapper::InterfaceWrapper{
private:
    /* data */
public:
    uint8_t transmitData(uint8_t *data, uint8_t len);
    uint8_t receiveData(uint8_t *data, uint8_t *len);
    uint8_t hasData();
};
