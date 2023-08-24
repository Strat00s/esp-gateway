#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
//#include "libs/sx127x.hpp"

/*
//I2C
#define SDA 6
#define SCL 7

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
#define NRF_IRQ 17
#define NRF_CE  18
#define NRF_CS  21

//ESP-01
#define TX1    14
#define RX1    16
#define IRG_01 15

#define STATUS_LED GPIO_NUM_1

//434MHz rf modules
#define RF_IN  9
#define RF_OUT 8

//SD card
#define SD_SCK  12
#define SD_MISO 13
#define SD_MOSI 11
#define SD_CS   10
*/


#define CS1  GPIO_NUM_26
#define RST1 GPIO_NUM_33
#define CS4  GPIO_NUM_39
#define RST4 GPIO_NUM_40

#define SCK  GPIO_NUM_36
#define MISO GPIO_NUM_37
#define MOSI GPIO_NUM_38


static const char* TAG = "main";


//SX127X lora(26, 33, 5);

spi_device_handle_t dev_handl;


//void pinMode(uint8_t pin, uint8_t mode) {
//    //TODO pullup/down on input
//    ESP_LOGI(TAG, "Set pin %d to mode %d", pin, mode);
//    gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
//}
//
//void pinWrite(uint8_t pin, uint8_t lvl) {
//    ESP_LOGI(TAG, "Set pin %d lvl to %d", pin, lvl);
//    gpio_set_level((gpio_num_t)pin, lvl);
//}
//
//uint8_t pinRead(uint8_t pin) {
//    ESP_LOGI(TAG, "Reading pin %d", pin);
//    return gpio_get_level((gpio_num_t)pin);
//}
//
//void delay(uint32_t ms) {
//    ESP_LOGI(TAG, "Delay of %lums", ms);
//    vTaskDelay(ms / portTICK_PERIOD_MS);
//}

void SPIBeginTransaction() {
    auto ret = spi_device_acquire_bus(dev_handl, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
}

void SPIEndTransaction() {
    spi_device_release_bus(dev_handl);
}

void SPITransfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t tx_length, size_t rx_length) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = tx_length * 8;
    t.tx_buffer = tx_buffer;
    
    auto ret = spi_device_polling_transmit(dev_handl, &t);
    ESP_ERROR_CHECK(ret);
    
    //write only
    if (rx_length == 0)
        return;
    
    memset(&t, 0, sizeof(t));
    t.length = rx_length * 8;
    t.rx_buffer = rx_buffer;
    
    ret = spi_device_polling_transmit(dev_handl, &t);
    ESP_ERROR_CHECK(ret);
}


extern "C" void app_main() {
    ESP_LOGI(TAG, "App start");

    vTaskDelay(2000 / portTICK_PERIOD_MS);


    gpio_set_direction(CS1, GPIO_MODE_OUTPUT);// pinMode(CS1,  GPIO_MODE_OUTPUT);
    gpio_set_direction(RST1, GPIO_MODE_OUTPUT);// pinMode(RST1, GPIO_MODE_OUTPUT);
    gpio_set_direction(CS4, GPIO_MODE_OUTPUT);//pinMode(CS4,  GPIO_MODE_OUTPUT);
    gpio_set_direction(RST4, GPIO_MODE_OUTPUT);//pinMode(RST4, GPIO_MODE_OUTPUT);
    
    gpio_set_level(CS1, 1);// pinWrite(CS1, 1);
    gpio_set_level(RST1, 1);// pinWrite(RST1, 1);
    gpio_set_level(CS4, 1);// pinWrite(CS4, 1);
    gpio_set_level(RST4, 1);// pinWrite(RST4, 1);

    //gpio_set_direction(STATUS_LED, GPIO_MODE_OUTPUT);//pinMode(STATUS_LED, GPIO_MODE_INPUT_OUTPUT);

    vTaskDelay(10 / portTICK_PERIOD_MS);


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
    //lora.registerDelay(delay);
    //lora.registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_OUTPUT);
    //lora.registerPinWrite(pinWrite);
    //lora.registerPinRead(pinRead);
    //lora.registerSPIStartTransaction();
    //lora.registerSPIEndTransaction();
    //lora.registerSpiTransfer();

    while(true) {
        //ESP_LOGI(TAG, "Testing 2s delay");
        //gpio_set_level(STATUS_LED, 1);//pinWrite(STATUS_LED, 1);

        //ESP_LOGI(TAG, "Module 1 reset");
        //ret = gpio_set_level(RST1, 0); //pinWrite(RST1, 0);
        //ESP_ERROR_CHECK(ret);
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        //ret = gpio_set_level(RST1, 0); //pinWrite(RST1, 1);
        //ESP_ERROR_CHECK(ret);
        //vTaskDelay(100 / portTICK_PERIOD_MS);


        uint8_t out = 0x6 & 0b01111111;
        uint8_t in[3];
    
        //spi_transaction_t t;
        //memset(&t, 0, sizeof(t));
        //t.length = 8;
        //t.tx_buffer = &out;
        //t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

        //ret = spi_device_acquire_bus(dev_handl, portMAX_DELAY);
        //ESP_ERROR_CHECK(ret);
        
        SPIBeginTransaction();

        gpio_set_level(CS1, 0);// pinWrite(CS1, 0);
        
        //ret = spi_device_polling_transmit(dev_handl, &t);
        //ESP_ERROR_CHECK(ret);

        //memset(&t, 0, sizeof(t));
        //t.length = 8*3;
        //t.rx_buffer = &in;

        SPITransfer(&out, in, 1, 3);

        //ret = spi_device_polling_transmit(dev_handl, &t);
        //ESP_ERROR_CHECK(ret);

        gpio_set_level(CS1, 1); //pinWrite(CS1, 1);
        

        SPIEndTransaction();

        //spi_device_release_bus(dev_handl);
        
        ESP_LOGI(TAG, "Sent data: 0x%02X", out);
        ESP_LOGI(TAG, "Received data: 0x%02X 0x%02X 0x%02X", in[0], in[1], in[2]);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //gpio_set_level(STATUS_LED, 0);//pinWrite(STATUS_LED, 0);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
