#pragma once
#include <stdio.h>
#include "interfaceWrapper.hpp"
#include "libs/sx127x.hpp"


class lora434InterfaceWrapper : public interfaceWrapper{
private:
    SX127X *lora;

public:

    lora434InterfaceWrapper(SX127X *lora) : interfaceWrapper(IW_TYPE_LORA_434M) {
        this->lora = lora;
    }

    /** @brief 
     * 
     * @param data Data to send/transmit.
     * @param len Length of data to be transmitted.
     * @return 0 on success, LORA ERR_... defines on error.
     */
    uint8_t transmitData(uint8_t *data, uint8_t len) {
        return lora->transmit(data, len);
    }

    /** @brief Retrieve data from underlying LORA device. Automatically starts reception if it is terminated by reading data.
     * 
     * @param buf Buffer to which to copy received data. Must be 256B long (maximal data length).
     * @param len Length of received data.
     * @return 0 on success, LORA ERR_... defines on error.
     */
    uint8_t getData(uint8_t *buf, uint8_t *len){
        uint8_t ret = lora->checkPayloadIntegrity();
        if (ret) {
            lora->clearIrqFlags();
            return ret;
        }

        lora->readData(buf);
        *len = lora->getPayloadLength();
        lora->clearIrqFlags();
        
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
