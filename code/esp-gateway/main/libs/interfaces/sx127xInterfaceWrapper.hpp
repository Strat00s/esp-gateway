#pragma once
#include <stdio.h>
#include "InterfaceWrapper.hpp"
#include "libs/sx127x.hpp"

//TODO remove last_ret a getstatus
//TODO wait for reception
//TODO copy data on transmission to buffer


class sx127xInterfaceWrapper : public InterfaceWrapper{
private:
    SX127X *lora;

public:

    sx127xInterfaceWrapper(SX127X *lora) : InterfaceWrapper(IF_TYPE_SX127X) {
        this->lora = lora;
    }


    /** @brief Start continuous LORA recepetion
     * 
     * @return Always returns true
     */
    inline uint8_t startReception() {
        lora->receiveContinuous();
        return 0;
    }

    /** @brief Stop LORA recepetion
     * 
     * @return Always returns true
     */
    inline uint8_t stopReception() {
        lora->setMode(SX127X_OP_MODE_STANDBY);
        return 0;
    }

    inline bool isMediumFree() {
        return !(lora->readRegister(REG_MODEM_STAT) & 0b00001011);
    }


    /** @brief Transmit data on underlying LORA device.
     * 
     * @param data Data to send/transmit. They will be overwritten by "junk" since the same buffer is used to save returned SPI transaction data.
     * @param len Length of data to be transmitted.
     * @return 0 on success.
     * 1 on failure to transmit.
     * 255 if allocation fails when copying data.
     */
    uint8_t sendData(uint8_t *data, uint8_t len, bool copy) {
        uint8_t ret = IFW_OK;
        if (copy) {
            uint8_t *tmp = new uint8_t[len];
            if (!tmp)
                return IFW_ALLOC_FAILED;
            memcpy(tmp, data, len);
            ret = lora->transmit(tmp, len);
            delete[] tmp;
        }
        else
            ret = lora->transmit(data, len);

        lora->receiveContinuous();
        return ret & IRQ_FLAG_TX_DONE ? 0 : 1;
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
        lora->setMode(SX127X_OP_MODE_STANDBY);
        uint8_t ret = lora->checkPayloadIntegrity();
        if (ret) {
            lora->clearIrqFlags();
            return 1;
        }

        lora->readData(buf, *len);
        *len = lora->getPayloadLength();
        lora->clearIrqFlags();

        lora->receiveContinuous();
        return IFW_OK;
    }

    /** @brief Check if interface has any new data, and if so, how many
     * 
     */
    inline uint8_t hasData() {
        return lora->readRegister(REG_IRQ_FLAGS) & IRQ_FLAG_RX_DONE;
    }
};
