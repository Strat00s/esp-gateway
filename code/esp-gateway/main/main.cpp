#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <esp_log.h>

#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_intr_alloc.h>

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

volatile bool rx_done = false;


SX127X lora(CS1, RST1, DIO01);
spi_device_handle_t dev_handl;


void pinMode(uint8_t pin, uint8_t mode) {
    //TODO pullup/down on input
    //ESP_LOGI(TAG, "Set pin %d to mode %d", pin, mode);
    gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
}
void pinWrite(uint8_t pin, uint8_t lvl) {
    gpio_set_level((gpio_num_t)pin, lvl);
}
uint8_t pinRead(uint8_t pin) {
    return gpio_get_level((gpio_num_t)pin);
}
void delay(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
uint32_t micros() {
    return esp_timer_get_time();
}
void SPIBeginTransaction() {
    auto ret = spi_device_acquire_bus(dev_handl, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
}
void SPIEndTransaction() {
    spi_device_release_bus(dev_handl);
}
void SPITransfer(uint8_t addr, uint8_t *buffer, size_t length) {
    //ESP_LOGI(TAG, "SPI read length: %d", length);
    //ESP_LOGI(TAG, "Address 0x%02X", addr);
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


static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    switch (gpio_num) {
        case DIO01: rx_done = true; break;
        case DIO02: rx_done = true; break;
        case DIO03: rx_done = true; break;
        case DIO04: rx_done = true; break;
        default: break;
    }
}


extern "C" void app_main() {
    ESP_LOGI(TAG, "App start");
    ESP_LOGI(TAG, "Reset all pins");

    //configure default pin states
    gpio_reset_pin(CS1);
    gpio_reset_pin(RST1);
    gpio_reset_pin(CS4);
    gpio_reset_pin(RST4);
    gpio_reset_pin(STATUS_LED);

    pinMode(CS4,  GPIO_MODE_OUTPUT);
    pinMode(RST4, GPIO_MODE_OUTPUT);

    pinWrite(CS4, 1);
    pinWrite(RST4, 1);

    pinMode(STATUS_LED, GPIO_MODE_INPUT_OUTPUT);
    pinWrite(STATUS_LED, 0);


    //configure SPI
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
        .clock_speed_hz = 10000000,
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL
    };
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &dev_handl);
    ESP_ERROR_CHECK(ret);


    //register callbacks
    lora.registerMicros(micros);
    lora.registerDelay(delay);
    lora.registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_OUTPUT);
    lora.registerPinWrite(pinWrite);
    lora.registerPinRead(pinRead);
    lora.registerSPIStartTransaction(SPIBeginTransaction);
    lora.registerSPIEndTransaction(SPIEndTransaction);
    lora.registerSpiTransfer(SPITransfer);

    //start lora and reconfigure it
    uint8_t rc = lora.begin(434.0, 0x12, 8);
    if (rc) {
        printf("BEGIN ERROR: %d\n", rc);
        return;
    }
    rc = lora.setBandwidth(LORA_BANDWIDTH_125kHz);
    if (rc) {
        printf("BW ERROR: %d\n", rc);
        return;
    }
    rc = lora.setSpreadingFactor(LORA_SPREADING_FACTOR_9);
    if (rc) {
        printf("SF ERROR: %d\n", rc);
        return;
    }
    rc = lora.setCodingRate(LORA_CODING_RATE_4_7);
    if (rc) {
        printf("BCR ERROR: %d\n", rc);
        return;
    }
    rc = lora.setRxTimeout(1023);
    if (rc) {
        printf("RXT ERROR: %d\n", rc);
        return;
    }

    rc = lora.getVersion();
    printf("Chip version: 0x%02X\n", rc);

    //enable interupt
    gpio_set_intr_type(DIO01, GPIO_INTR_POSEDGE);
    gpio_intr_enable(DIO01);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DIO01, gpio_isr_handler, (void*) DIO01);

    printf("Starting in 3s...\n");
    delay(1000);
    printf("2s...\n");
    delay(1000);
    printf("1s...\n");
    delay(1000);

    lora.receiveContinuous();

    uint32_t start = micros();
    uint32_t bytes_total = 0;

    while(true) {
        if (rx_done) {
            rx_done = false;
            uint8_t status = lora.checkPayloadIntegrity();
            if (status) {
                switch (status) {
                    case ERR_RX_TIMEOUT:   printf("ERR_RX_TIMEOUT\n"); break;
                    case ERR_CRC_MISMATCH: printf("ERR_CRC_MISSMATCH\n"); break;
                    default: printf("UNKNOWN\n"); break;
                }
                lora.clearIrqFlags();
                continue;
            }
            uint8_t payload[256] = {0};
            lora.readData(payload);
            bytes_total += lora.getPayloadLength();
            lora.clearIrqFlags();
            printf("Received payload: %s\n", payload);
            printf("Total bytes received: %lu\n", bytes_total);
        }

        if (micros() - start > 1000*1000) {
            start = micros();
            pinWrite(STATUS_LED, !pinRead(STATUS_LED));
            printf("STATUS LED: %d\n", pinRead(STATUS_LED));
        }
    }
}
