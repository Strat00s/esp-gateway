#pragma once
#include <stdio.h>
#include "InterfaceWrapperBase.hpp"
#include "libs/sx127x.hpp"

//TODO remove last_ret a getstatus
//TODO wait for reception
//TODO copy data on transmission to buffer


class sx127xInterfaceWrapper : public InterfaceWrapperBase{
private:
    SX127X *lora;

public:

    sx127xInterfaceWrapper(SX127X *lora) : InterfaceWrapperBase(IF_TYPE_SX127X) {
        this->lora = lora;
    }


    /** @brief Start continuous LORA recepetion
     * 
     * @return Always returns true
     */
    uint8_t startReception() {
        lora->receive();
        return 0;
    }

    /** @brief Stop LORA recepetion
     * 
     * @return Always returns true
     */
    uint8_t stopReception() {
        lora->setMode(SX127X_OP_MODE_STANDBY);
        return 0;
    }

    bool isMediumBusy() {
        return (lora->readRegister(REG_MODEM_STAT) & 0b00001011);
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
        uint8_t *tmp = new uint8_t[len];
        if (!tmp)
            return IFW_ALLOC_FAILED;
        memcpy(tmp, data, len);
        ret = lora->transmit(tmp, len);
        delete[] tmp;

        lora->receive();
        return (ret & IRQ_FLAG_TX_DONE) ? 0 : 1;
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
        
        if (lora->checkPayloadIntegrity()) {
            lora->clearIrqFlags();
            lora->receive();
            return 1;
        }

        lora->readData(buf, *len);
        *len = lora->getPayloadLength();
        
        lora->clearIrqFlags();
        lora->receive();
        return IFW_OK;
    }

    /** @brief Check if interface has any new data, and if so, how many
     * 
     */
    uint8_t hasData() {
        return lora->readRegister(REG_IRQ_FLAGS) & IRQ_FLAG_RX_DONE;
    }

    double getTimeOnAir(uint8_t payload_length) {
        uint8_t spreading_factor = lora->getSpreadingFactor();
        double symbol_time = static_cast<double>((uint16_t(1) << spreading_factor)) / lora->getBandwidth();
        
        int payload_symbols = 8 * payload_length;
        payload_symbols -= 4 * spreading_factor;
        payload_symbols += 28;
        payload_symbols += 16 * (lora->readRegister(REG_MODEM_CONFIG_2, 2, 2) >> 2);
        payload_symbols -= 20 * lora->readRegister(REG_MODEM_CONFIG_1, 0, 0);

        if (payload_symbols < 0)
            return ((uint16_t(lora->readRegister(REG_PREAMBLE_MSB)) << 8 | lora->readRegister(REG_PREAMBLE_LSB)) + 4.25 + 8) * symbol_time;

        payload_symbols /= (4.0 * (spreading_factor - 2.0 * (lora->readRegister(REG_MODEM_CONFIG_3, 3, 3) >> 3)));
        payload_symbols += 1; //cheap ceil
        payload_symbols *= (lora->readRegister(REG_MODEM_CONFIG_1, 1, 3) >> 1) + 4;
        payload_symbols += 8;
        return ((uint16_t(lora->readRegister(REG_PREAMBLE_MSB)) << 8 | lora->readRegister(REG_PREAMBLE_LSB)) + 4.25 + payload_symbols) * symbol_time;
    }

    bool reset() {
        lora->reset();
        return true;
    }
};
