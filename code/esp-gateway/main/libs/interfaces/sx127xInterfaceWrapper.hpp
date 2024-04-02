#pragma once
#include <stdio.h>
#include "InterfaceWrapper.hpp"
#include "libs/sx127x.hpp"


class sx127xInterfaceWrapper : public InterfaceWrapper{
private:
    SX127X *lora;

public:

    sx127xInterfaceWrapper(SX127X *lora) : InterfaceWrapper(IF_TYPE_SX127X) {
        this->lora = lora;
    }

    //bool begin(float freq, uint8_t sync_word, uint16_t preamble_len, uint8_t bw, uint8_t sf, uint8_t cr) {
    //    lora->reset();
    //    last_ret = lora->begin(freq, sync_word, preamble_len, bw, sf, cr);
    //    return last_ret ? false : true;
    //}


    /** @brief Start continuous LORA recepetion
     * 
     * @return Always returns true
     */
    bool startReception() {
        lora->receiveContinuous();
        return true;
    }

    /** @brief Stop LORA recepetion
     * 
     * @return Always returns true
     */
    bool stopReception() {
        lora->setMode(SX127X_OP_MODE_STANDBY);
        return true;
    }


    /** @brief Transmit data on underlying LORA device.
     * 
     * @param data Data to send/transmit. They will be overwritten by "junk" since the same buffer is used to save returned SPI transaction data.
     * @param len Length of data to be transmitted.
     * @return True on success, false otherwise. IRQ flags stored as status.
     */
    bool sendData(uint8_t *data, uint8_t len) {
        last_ret = lora->transmit(data, len);
        return last_ret & IRQ_FLAG_TX_DONE;
    }

    /** @brief Retrieve data from underlying LORA device.
     * Automatically starts reception if it is terminated by reading data.
     * 
     * @param buf Buffer to which to copy received data. Must be 256B long (maximal data length).
     * @param len Length of received data.
     * @return 0 on success, LORA ERR_... defines on error.
     */
    bool getData(uint8_t *buf, uint8_t *len) {
        lora->setMode(SX127X_OP_MODE_STANDBY);
        last_ret = lora->checkPayloadIntegrity();
        if (last_ret) {
            lora->clearIrqFlags();
            return false;
        }

        last_ret = lora->readData(buf, *len);
        *len = lora->getPayloadLength();
        lora->clearIrqFlags();

        lora->receiveContinuous();

        return true;
    }

    /** @brief Check if interface has any new data
     * 
     */
    inline uint8_t hasData() {
        return lora->hasData();
    }
};
