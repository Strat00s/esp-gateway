

#pragma once
#include <stdio.h>
#include "interfaceWrapper.hpp"



template <uint8_t N>
class InterfaceManager {
private:
    InterfaceWrapper *interfaces[N] = {nullptr};
    uint8_t if_cnt = 0;

public:
    InterfaceManager(/* args */);
    ~InterfaceManager();

    /** @brief Add interface wrapper
     * @return 0 on success, 1 if no space left
     */
    uint8_t addInterface(InterfaceWrapper *interface) {
        if (if_cnt >= N)
            return 1;
        
        interfaces[if_cnt] = interface;
        if_cnt++;
        return 0;
    }

    /** @brief Check if there are any data and if so, how many.
     * 
     * @return How many new data (packets) are available,
     */
    uint8_t hasData() {
        uint8_t data_cnt = 0;
        for (uint8_t i = 0; i < N; i++) {
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
     * @return 0 on success.
     * 256 if there are no data.
     * Other values are interface specific.
     */
    uint8_t getNextData(uint8_t *data, uint8_t *len) {
        for (uint8_t i = 0; i < N; i++) {
            if (interfaces[i] == nullptr)
                continue;

            if (interfaces[i]->hasData())
                return interfaces[i]->getData(data, len);
        }

        return 256;
    }


    /** @brief Send data on interface.
     * 
     * @param if_id Index of the interface (order by which they were added)
     * @param data Data to be sent.
     * @param len Length of the data.
     * @return 0 on success.
     * Other values are interface specific.
     */
    uint8_t sendData(uint8_t if_id, uint8_t *data, uint8_t *len) {
        return interfaces[if_id]->senData(data, len);
    }
};

