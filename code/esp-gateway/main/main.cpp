#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>

#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
#include "libs/sx127x.hpp"


//I2C
#define SDA GPIO_NUM_6
#define SCL GPIO_NUM_7

//SPI for modlues
#define SCK  GPIO_NUM_36
#define MISO GPIO_NUM_37
#define MOSI GPIO_NUM_38

//lora modules
#define CS1   GPIO_NUM_26
#define RST1  GPIO_NUM_33
#define DIO01 GPIO_NUM_5
#define CS2   GPIO_NUM_34
#define RST2  GPIO_NUM_35
#define DIO02 GPIO_NUM_4
#define CS3   GPIO_NUM_41
#define RST3  GPIO_NUM_42
#define DIO03 GPIO_NUM_2
#define CS4   GPIO_NUM_39
#define RST4  GPIO_NUM_40
#define DIO04 GPIO_NUM_3

//NRF240L1+ module
#define NRF_IRQ GPIO_NUM_17
#define NRF_CE  GPIO_NUM_18
#define NRF_CS  GPIO_NUM_21

//ESP-01
#define TX1    GPIO_NUM_14
#define RX1    GPIO_NUM_16
#define IRG_01 GPIO_NUM_15

#define STATUS_LED GPIO_NUM_1

//434MHz rf modules
#define RF_IN  GPIO_NUM_9
#define RF_OUT GPIO_NUM_8

//SD card
#define SD_SCK  GPIO_NUM_12
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_CS   GPIO_NUM_10


static const char* TAG = "main";


SX127X lora(CS1, RST1, DIO01);

spi_device_handle_t dev_handl;


void pinMode(uint8_t pin, uint8_t mode) {
    //TODO pullup/down on input
    ESP_LOGI(TAG, "Set pin %d to mode %d", pin, mode);
    gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
}

void pinWrite(uint8_t pin, uint8_t lvl) {
    ESP_LOGI(TAG, "Set pin %d lvl to %d", pin, lvl);
    gpio_set_level((gpio_num_t)pin, lvl);
}

uint8_t pinRead(uint8_t pin) {
    ESP_LOGI(TAG, "Reading pin %d", pin);
    return gpio_get_level((gpio_num_t)pin);
}

void delay(uint32_t ms) {
    ESP_LOGI(TAG, "Delay of %lums", ms);
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void SPIBeginTransaction() {
    auto ret = spi_device_acquire_bus(dev_handl, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
}

void SPIEndTransaction() {
    spi_device_release_bus(dev_handl);
}

void SPITransfer(uint8_t addr, uint8_t *buffer, size_t length) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &addr;
    
    auto ret = spi_device_polling_transmit(dev_handl, &t);
    ESP_ERROR_CHECK(ret);

    memset(&t, 0, sizeof(t));
    t.length = length * 8;

    //write
    if (addr & 0b10000000)
        t.tx_buffer = buffer;
    //read
    else
        t.rx_buffer = buffer;

    ret = spi_device_polling_transmit(dev_handl, &t);
    ESP_ERROR_CHECK(ret);
}


extern "C" void app_main() {
    ESP_LOGI(TAG, "App start");
    ESP_LOGI(TAG, "Reset all pins");
    
    gpio_reset_pin(CS1);
    gpio_reset_pin(RST1);
    gpio_reset_pin(CS4);
    gpio_reset_pin(RST4);
    gpio_reset_pin(STATUS_LED);

    pinMode(CS1,  GPIO_MODE_OUTPUT);
    pinMode(RST1, GPIO_MODE_OUTPUT);
    pinMode(CS4,  GPIO_MODE_OUTPUT);
    pinMode(RST4, GPIO_MODE_OUTPUT);

    pinWrite(CS1, 1);
    pinWrite(RST1, 1);
    pinWrite(CS4, 1);
    pinWrite(RST4, 1);

    pinMode(STATUS_LED, GPIO_MODE_INPUT_OUTPUT);

    //vTaskDelay(10 / portTICK_PERIOD_MS);


    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 4000000,
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL
    };
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &dev_handl);
    ESP_ERROR_CHECK(ret);

    lora.registerDelay(delay);
    lora.registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_OUTPUT);
    lora.registerPinWrite(pinWrite);
    lora.registerPinRead(pinRead);
    lora.registerSPIStartTransaction(SPIBeginTransaction);
    lora.registerSPIEndTransaction(SPIEndTransaction);
    lora.registerSpiTransfer(SPITransfer);

    while(true) {
        ESP_LOGI(TAG, "LoRa reset");
        lora.reset();
        //ESP_LOGI(TAG, "Testing 2s delay");
        uint8_t reg = lora.readRegister(REG_FRF_MSB);
        printf("REG_FRF_MSB: 0x%02x\n", reg);
        delay(1000);
        lora.writeRegister(REG_FRF_MSB, 0xBB);
        reg = lora.readRegister(REG_FRF_MSB);
        printf("REG_FRF_MSB: 0x%02x\n", reg);
        delay(1000);
        reg =  lora.readRegister(REG_OP_MODE);
        printf("REG_OP_MODE: 0x%02x\n", reg);
        delay(1000);
        lora.setRegister(REG_OP_MODE, 0x00, 0, 2);
        reg =  lora.readRegister(REG_OP_MODE);
        printf("REG_OP_MODE: 0x%02x\n", reg);
        delay(1000);
        lora.setRegister(REG_OP_MODE, 0x01, 0, 2);
        reg =  lora.readRegister(REG_OP_MODE);
        printf("REG_OP_MODE: 0x%02x\n", reg);
        delay(1000);
        return;
        /*
        gpio_set_level(STATUS_LED, 0);//pinWrite(STATUS_LED, 1);

        uint8_t out[2] = {0x06 | 0b10000000, 0xBB};
        uint8_t in[3] = {0, 0, 0};
    
        SPIBeginTransaction();
        pinWrite(CS4, 0);
        SPITransfer(out, in, 2, 0);
        pinWrite(CS4, 1);
        SPIEndTransaction();

        ESP_LOGI(TAG, "Sent data: 0x%02X 0x%02X", out[0], out[1]);
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X", in[0], in[1], in[2]);

        out[1] = 0xAA;
        memset(in, 0, 3);// in[3] = {0, 0, 0};

        SPIBeginTransaction();
        pinWrite(CS1, 0);
        SPITransfer(out, in, 2, 0);
        pinWrite(CS1, 1);
        SPIEndTransaction();

        ESP_LOGI(TAG, "Sent data: 0x%02X 0x%02X", out[0], out[1]);
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X", in[0], in[1], in[2]);

        out[0] = 0x06 & 0b01111111;
        ESP_LOGI(TAG, "Reading it back");
        SPIBeginTransaction();
        pinWrite(CS4, 0);
        SPITransfer(out, in, 1, 1);
        pinWrite(CS4, 1);
        SPIEndTransaction();

        ESP_LOGI(TAG, "Sent data: 0x%02X 0x%02X", out[0], out[1]);
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X", in[0], in[1], in[2]);

        memset(in, 0, 3);
        SPIBeginTransaction();
        pinWrite(CS1, 0);
        SPITransfer(out, in, 1, 1);
        pinWrite(CS1, 1);
        SPIEndTransaction();

        ESP_LOGI(TAG, "Sent data: 0x%02X 0x%02X", out[0], out[1]);
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X", in[0], in[1], in[2]);
        */
    }
}
