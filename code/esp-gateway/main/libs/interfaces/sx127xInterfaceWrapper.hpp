#pragma once
#include <stdio.h>
#include "InterfaceWrapper.hpp"
#include "libs/sx127x.hpp"


class sx127xInterfaceWrapper : protected InterfaceWrapper{
private:
    SX127X *lora;

public:

    sx127xInterfaceWrapper(SX127X *lora) : InterfaceWrapper(IF_TYPE_SX127X) {
        this->lora = lora;
    }

    /** @brief Transmit data on underlying LORA device.
     * 
     * @param data Data to send/transmit. They will be overwritten by "junk" since the same buffer is used to save returned SPI transaction data.
     * @param len Length of data to be transmitted.
     * @return 0 on success. IRQ flags on failure.
     */
    uint8_t sendData(uint8_t *data, uint8_t len) {
        uint8_t ret = lora->transmit(data, len);
        if (ret & IRQ_FLAG_TX_DONE)
            return 0;
        return ret;
    }

    /** @brief Retrieve data from underlying LORA device.
     * Automatically starts reception if it is terminated by reading data.
     * 
     * @param buf Buffer to which to copy received data. Must be 256B long (maximal data length).
     * @param len Length of received data.
     * @return 0 on success, LORA ERR_... defines on error.
     */
    uint8_t getData(uint8_t *buf, uint8_t *len){
        lora->setMode(SX127X_OP_MODE_STANDBY);
        uint8_t ret = lora->checkPayloadIntegrity();
        if (ret) {
            lora->clearIrqFlags();
            return ret;
        }

        lora->readData(buf);
        *len = lora->getPayloadLength();
        lora->clearIrqFlags();

        lora->receiveContinuous();

        return ret;
    }

    /** @brief Start continuous LORA recepetion
     * 
     * @return Always returns 0;
     */
    uint8_t startReception(){
        lora->receiveContinuous();
        return 0;
    }

    /** @brief Check if interface has any new data
     * 
     */
    uint8_t hasData() {
        return lora->readRegister(REG_IRQ_FLAGS) & IRQ_FLAG_RX_DONE;
    }
};
