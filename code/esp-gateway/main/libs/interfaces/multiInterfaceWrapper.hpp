#pragma once
#include <stdio.h>
#include "interfaceWrapperBase.hpp"


#define SKIP_NULL(interface) {if (!interface) continue;}


template <size_t N>
class multiInterfaceWrapper : public InterfaceWrapperBase{
private:
    InterfaceWrapperBase *interfaces[N] = {nullptr};
    size_t if_cnt = 0;

public:

    multiInterfaceWrapper() : InterfaceWrapperBase(IF_TYPE_MULTI) {}


    uint8_t addInterface(InterfaceWrapperBase *interface) {
        if (if_cnt >= N)
            return 1;
        
        interfaces[if_cnt] = interface;
        if_cnt++;
        return 0;
    }


    /** @brief Start continuous LORA recepetion
     * 
     * @return Always returns true
     */
    inline uint8_t startReception() {
        for (auto &interface : interfaces) {
            if (!interface)
                continue;
            
            interface->startReception();
        }
        return 0;
    }

    /** @brief Stop LORA recepetion
     * 
     * @return Always returns true
     */
    inline uint8_t stopReception() {
        for (auto &interface : interfaces) {
            if (!interface)
                continue;
            
            interface->stopReception();
        }
        return 0;
    }

    inline bool isMediumBusy() {
        for (auto &interface : interfaces) {
            if (!interface)
                continue;
            
            if (interface->isMediumBusy())
                return true;
        }
        return false;
    }


    /** @brief Transmit data on underlying LORA device.
     * 
     * @param data Data to send/transmit. They will be overwritten by "junk" since the same buffer is used to save returned SPI transaction data.
     * @param len Length of data to be transmitted.
     * @return 0 on success.
     * 1 on failure to transmit.
     * 255 if allocation fails when copying data.
     */
    uint8_t sendData(uint8_t *data, uint8_t len) {
        uint8_t ret = IFW_OK;
        for (auto &interface : interfaces) {
            if (!interface)
                continue;
            
            ret |= interface->sendData(data, len);
        }

        return ret;
    }

    /** @brief Retrieve data from underlying LORA device.
     * Automatically starts reception if it is terminated by reading data.
     * 
     * @param buf Buffer to which to copy received data. Must be 256B long (maximal data length).
     * @param len Length of received data.
     * @return 0 on success
     * 1 on failure.
     */
    uint8_t getData(uint8_t *buf, uint8_t *len) {
        for (auto &interface : interfaces) {
            if (!interface || !interface->hasData())
                continue;
            
            interface->getData(buf, len);
            return 0;
        }
        return 1;
    }

    /** @brief Check if interface has any new data, and if so, how many
     * 
     */
    inline uint8_t hasData() {
        uint8_t cnt = 0;
        for (auto &interface : interfaces) {
            if (!interface)
                continue;
            
            cnt += interface->hasData();
        }

        return cnt;
    }
};



//#define IFM_OK           0
//#define IFM_ALLOC_FAILED 255
//
//#define IF_COUNT 1
//
//
//class InterfaceManager {
//private:
//    InterfaceWrapperBase *interfaces[IF_COUNT] = {nullptr};
//    uint8_t if_cnt = 0;
//
//public:
//    //InterfaceManager(/* args */);
//    //~InterfaceManager();
//
//    /** @brief Add interface wrapper
//     * @return 0 on success.
//     * 1 if no space left.
//     */
//    uint8_t addInterface(InterfaceWrapperBase *interface) {
//        if (if_cnt >= IF_COUNT)
//            return 1;
//        
//        interfaces[if_cnt] = interface;
//        if_cnt++;
//        return IFM_OK;
//    }
//
//    inline InterfaceWrapperBase *getInterface(uint8_t index) {
//        return interfaces[index];
//    }
//
//    uint8_t getInterfaceCount() {
//        return if_cnt;
//    }
//
//    bool isMediumBusy() {
//        for (uint8_t i = 0; i < IF_COUNT; i++)
//            if (interfaces[i]->isMediumBusy())
//                return true;
//        return false;
//    }
//
//    /** @brief Check if there are any data and if so, how many.
//     * 
//     * @return How many new data (packets) are available,
//     */
//    uint8_t hasData() {
//        uint8_t data_cnt = 0;
//        for (uint8_t i = 0; i < IF_COUNT; i++) {
//            if (interfaces[i])
//                data_cnt += interfaces[i]->hasData();
//        }
//
//        return data_cnt;
//    }
//
//    /** @brief Go through all interfaces one by one and get first next available data
//     * Stops reception (if possible) when retrieving data.
//     * Start reception once 
//     * 
//     * @param data Buffer where to store data.
//     * @param len Length of the buffer.
//     * Overwriten with the recieved data length that can fit to the buffer.
//     * @return 0 on success. 255 on no data. Other values are interface specific for getData().
//     */
//    bool getNextData(uint8_t *data, uint8_t *len) {
//        for (uint8_t i = 0; i < IF_COUNT; i++) {
//            if (interfaces[i] && interfaces[i]->hasData())
//                return interfaces[i]->getData(data, len);
//        }
//
//        return IFM_ALLOC_FAILED;
//    }
//
//
//    /** @brief Send data on all interfaces.
//     * It first allocates extra buffer to which to copy all data before sending them.
//     * 
//     * @param data Data to be sent.
//     * @param len Length of the data.
//     * @return 0 on success.
//     * 255 if allocation failed.
//     * Other values are interface specific (sendData).
//     */
//    uint8_t sendData(uint8_t *data, uint8_t len) {
//        uint8_t *tmp = new uint8_t[len];
//        if (!tmp)
//            return 255;
//
//        uint8_t ret = 0;
//        for (uint8_t i = 0; i < if_cnt; i++) {
//            if (!interfaces[i])
//                continue;
//
//            memcpy(tmp, data, len);
//            ret = interfaces[i]->sendData(tmp, len);
//            IF_TRUE_RET_X(ret);
//        }
//        delete[] tmp;
//        return ret;
//    }
//
//    /** @brief Send data on interface.
//     * 
//     * @param index Index of the interface (order by which they were added)
//     * @param data Data to be sent.
//     * @param len Length of the data.
//     * @return 0 on success.
//     * Other values are interface specific (sendData).
//     */
//    inline uint8_t sendData(uint8_t index, uint8_t *data, uint8_t len) {
//        return interfaces[index]->sendData(data, len, true);
//    }
//};

