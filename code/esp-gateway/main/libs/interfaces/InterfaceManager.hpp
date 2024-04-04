

#pragma once
#include <stdio.h>
#include "interfaceWrapper.hpp"


//class InterfaceManagerBase {
//protected:
//    uint8_t if_cnt = 0;
//
//public:
//    virtual ~InterfaceManagerBase() {}
//    // Define virtual methods that will be implemented by InterfaceManager<N>
//
//    virtual uint8_t addInterface(InterfaceWrapper *interface) = 0;
//    virtual uint8_t getInterfaceCount() = 0;
//    virtual uint8_t hasData() = 0;
//    virtual uint8_t getNextData(uint8_t *data, uint8_t *len) = 0;
//    virtual uint8_t sendData(uint8_t *data, uint8_t len) = 0;
//    virtual uint8_t sendData(uint8_t if_id, uint8_t *data, uint8_t *len) = 0;
//};


#define IF_COUNT 1


class InterfaceManager {
private:
    InterfaceWrapper *interfaces[IF_COUNT] = {nullptr};
    uint8_t if_cnt = 0;

public:
    //InterfaceManager(/* args */);
    //~InterfaceManager();

    /** @brief Add interface wrapper
     * @return 0 on success, 1 if no space left
     */
    uint8_t addInterface(InterfaceWrapper *interface) {
        if (if_cnt >= IF_COUNT)
            return 1;
        
        interfaces[if_cnt] = interface;
        if_cnt++;
        return 0;
    }

    inline InterfaceWrapper *getInterface(uint8_t index) {
        return interfaces[index];
    }

    uint8_t getInterfaceCount() {
        return if_cnt;
    }

    /** @brief Check if there are any data and if so, how many.
     * 
     * @return How many new data (packets) are available,
     */
    uint8_t hasData() {
        uint8_t data_cnt = 0;
        for (uint8_t i = 0; i < IF_COUNT; i++) {
            if (interfaces[i] == nullptr)
                continue;

            data_cnt += interfaces[i]->hasData();
        }

        return data_cnt;
    }

    /** @brief Go through all interfaces one by one and get first next available data
     * Stops reception (if possible) when retrieving data.
     * Start reception once 
     * 
     * @param data Buffer where to store data.
     * @param len Length of the buffer.
     * Overwriten with the recieved data length that can fit to the buffer.
     * @return 0 on success, 1 on failure, 2 on no data
     */
    bool getNextData(uint8_t *data, uint8_t *len) {
        for (uint8_t i = 0; i < IF_COUNT; i++) {
            if (interfaces[i] == nullptr)
                continue;

            if (interfaces[i]->hasData())
                return interfaces[i]->getData(data, len);
        }

        return 2;
    }


    /** @brief Send data on all interfaces.
     * 
     * @param data Data to be sent.
     * @param len Length of the data.
     * @return 0 on success.
     * 255 if allocation failed.
     * Other values are interface specific.
     */
    uint8_t sendData(uint8_t *data, uint8_t len) {
        uint8_t *tmp = new uint8_t[len];
        if (tmp == nullptr)
            return 255;

        for (uint8_t i = 0; i < if_cnt; i++) {
            if (interfaces[i] == nullptr)
                continue;

            memcpy(tmp, data, len);
            if (!interfaces[i]->sendData(tmp, len))
                return interfaces[i]->getStatus();
        }
        delete[] tmp;
        return 0;
    }

    /** @brief Send data on interface.
     * 
     * @param index Index of the interface (order by which they were added)
     * @param data Data to be sent.
     * @param len Length of the data.
     * @return 0 on success.
     * 255 if allocation failed.
     * Other values are interface specific.
     */
    uint8_t sendData(uint8_t index, uint8_t *data, uint8_t len) {
        uint8_t *tmp = new uint8_t[len];
        if (tmp == nullptr)
            return 255;
        memcpy(tmp, data, len);
        bool ret = interfaces[index]->sendData(tmp, len);
        delete[] tmp;
        return ret;
    }
};

