#pragma once
#include <stdio.h>
#include "interfaceWrapper.hpp"



#define IFM_OK           0
#define IFM_ALLOC_FAILED 255

#define IF_COUNT 1


class InterfaceManager {
private:
    InterfaceWrapper *interfaces[IF_COUNT] = {nullptr};
    uint8_t if_cnt = 0;

public:
    //InterfaceManager(/* args */);
    //~InterfaceManager();

    /** @brief Add interface wrapper
     * @return 0 on success.
     * 1 if no space left.
     */
    uint8_t addInterface(InterfaceWrapper *interface) {
        if (if_cnt >= IF_COUNT)
            return 1;
        
        interfaces[if_cnt] = interface;
        if_cnt++;
        return IFM_OK;
    }

    inline InterfaceWrapper *getInterface(uint8_t index) {
        return interfaces[index];
    }

    uint8_t getInterfaceCount() {
        return if_cnt;
    }

    bool isMediumFree() {
        for (uint8_t i = 0; i < IF_COUNT; i++)
            if (!interfaces[i]->isMediumFree())
                return false;
        return true;
    }

    /** @brief Check if there are any data and if so, how many.
     * 
     * @return How many new data (packets) are available,
     */
    uint8_t hasData() {
        uint8_t data_cnt = 0;
        for (uint8_t i = 0; i < IF_COUNT; i++) {
            if (interfaces[i])
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
     * @return 0 on success. 255 on no data. Other values are interface specific for getData().
     */
    bool getNextData(uint8_t *data, uint8_t *len) {
        for (uint8_t i = 0; i < IF_COUNT; i++) {
            if (interfaces[i] && interfaces[i]->hasData())
                return interfaces[i]->getData(data, len);
        }

        return IFM_ALLOC_FAILED;
    }


    /** @brief Send data on all interfaces.
     * It first allocates extra buffer to which to copy all data before sending them.
     * 
     * @param data Data to be sent.
     * @param len Length of the data.
     * @return 0 on success.
     * 255 if allocation failed.
     * Other values are interface specific (sendData).
     */
    uint8_t sendData(uint8_t *data, uint8_t len) {
        uint8_t *tmp = new uint8_t[len];
        if (!tmp)
            return 255;

        uint8_t ret = 0;
        for (uint8_t i = 0; i < if_cnt; i++) {
            if (!interfaces[i])
                continue;

            memcpy(tmp, data, len);
            ret = interfaces[i]->sendData(tmp, len);
            IF_TRUE_RET_X(ret);
        }
        delete[] tmp;
        return ret;
    }

    /** @brief Send data on interface.
     * 
     * @param index Index of the interface (order by which they were added)
     * @param data Data to be sent.
     * @param len Length of the data.
     * @return 0 on success.
     * Other values are interface specific (sendData).
     */
    inline uint8_t sendData(uint8_t index, uint8_t *data, uint8_t len) {
        return interfaces[index]->sendData(data, len, true);
    }
};

