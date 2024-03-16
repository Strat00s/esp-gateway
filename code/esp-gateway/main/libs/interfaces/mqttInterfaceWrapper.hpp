#pragma once
#include <stdio.h>
#include "InterfaceWrapper.hpp"
#include "../containers/simpleQueue.hpp"
#include <cstring>


class mqttInterfaceWrapper : public ifWrapper::InterfaceWrapper{
private:
    SimpleQueue<std::string> in;

public:
    mqttInterfaceWrapper() : InterfaceWrapper(IW_TYPE_MQTT) {};

    uint8_t transmitData(uint8_t *data, uint8_t len) {return 1;}
    uint8_t getData(uint8_t *data, uint8_t *len) {
        std::string tmp;
        if(in.read(&tmp, 0)) {
            data = nullptr;
            len = nullptr;
            return 1;
        }

        if (tmp.size() > 0xFF)
            *len = 0xFF;
        else
            *len = tmp.size();
        memcpy(data, tmp.c_str(), *len);
        return 0;
    }
    uint8_t startReception() {return 1;}
    uint8_t hasData() {
        return in.size() ? 1 : 0;
    }
    uint8_t clearIRQ() {return 1;}

    void addData(std::string data) {
        in.write(data);
    }
};
