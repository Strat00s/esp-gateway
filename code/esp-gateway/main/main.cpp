#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <esp_log.h>

#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_intr_alloc.h>

#include "libs/sx127x.hpp"


//I2C
#define SDA GPIO_NUM_6
#define SCL GPIO_NUM_7
//status led
#define STATUS_LED GPIO_NUM_1
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
//434MHz rf modules
#define RF_IN  GPIO_NUM_9
#define RF_OUT GPIO_NUM_8
//SD card
#define SD_SCK  GPIO_NUM_12
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_CS   GPIO_NUM_10


volatile bool irq1 = false;
volatile bool irq2 = false;
volatile bool irq3 = false;
volatile bool irq4 = false;


SX127X lora_434(CS1, RST1, DIO01);
SX127X lora_868(CS4, RST4, DIO04);

spi_device_handle_t dev_handl;

TaskHandle_t status_task_handle;
//TaskHandle_t log_task_handl;
//TaskHandle_t display_task_handl;
//TaskHandle_t status_task;


void pinMode(uint8_t pin, uint8_t mode) {
    gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
}
void pinWrite(uint8_t pin, uint8_t lvl) {
    gpio_set_level((gpio_num_t)pin, lvl);
}
uint8_t pinRead(uint8_t pin) {
    return gpio_get_level((gpio_num_t)pin);
}
void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
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


//gpio irq
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    switch (gpio_num) {
        case DIO01: irq1 = true; break;
        case DIO02: irq2 = true; break;
        case DIO03: irq3 = true; break;
        case DIO04: irq4 = true; break;
        default: break;
    }
}



#define PTRN_ID_ERR  0
#define PTRN_ID_IDLE 1
#define PTRN_ID_INIT 2

//blink patterns
//blink cnt, blink on time (1 = 100ms), delay between pattern repeats (1 = 100ms)
//delay between individual blinks is 100ms
static uint8_t blink_patterns[3][3] = {
    {5, 1, 5},  //PTRN_ID_ERR
    {1, 1, 50}, //PTRN_ID_IDLE
    {3, 1, 1}, //PTRN_ID_INIT
};

void status_task(void *pvParameters) {
    //configure pins

    //setup default values
    uint32_t id = PTRN_ID_ERR;

    while (true) {
        for (int i = 0; i < blink_patterns[id][0]; i++) {
            gpio_set_level(STATUS_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(blink_patterns[id][1] * 100));
            gpio_set_level(STATUS_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xTaskNotifyWait(0, 0, &id, pdMS_TO_TICKS(blink_patterns[id][2]* 100));
    }
}


extern "C" void app_main() {
    static const char* TAG = "main";

    gpio_reset_pin(STATUS_LED);
    gpio_set_direction(STATUS_LED, GPIO_MODE_INPUT_OUTPUT);

    //create all tasks
    xTaskCreate(status_task, "status_task", 344, nullptr, 20, &status_task_handle);
    xTaskNotify(status_task_handle, PTRN_ID_INIT, eSetValueWithOverwrite);


    ESP_LOGD(TAG, "App start");
    ESP_LOGD(TAG, "Reset all pins");


    //configure default pin states
    gpio_reset_pin(CS1);
    gpio_reset_pin(RST1);
    gpio_reset_pin(DIO01);
    gpio_reset_pin(CS4);
    gpio_reset_pin(RST4);
    gpio_reset_pin(DIO04);


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
    lora_434.registerMicros(micros);
    lora_434.registerDelay(delay);
    lora_434.registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_INPUT_OUTPUT);
    lora_434.registerPinWrite(pinWrite);
    lora_434.registerPinRead(pinRead);
    lora_434.registerSPIStartTransaction(SPIBeginTransaction);
    lora_434.registerSPIEndTransaction(SPIEndTransaction);
    lora_434.registerSpiTransfer(SPITransfer);

    lora_868.registerMicros(micros);
    lora_868.registerDelay(delay);
    lora_868.registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_INPUT_OUTPUT);
    lora_868.registerPinWrite(pinWrite);
    lora_868.registerPinRead(pinRead);
    lora_868.registerSPIStartTransaction(SPIBeginTransaction);
    lora_868.registerSPIEndTransaction(SPIEndTransaction);
    lora_868.registerSpiTransfer(SPITransfer);


    //start lora434 and reconfigure it
    uint8_t rc = lora_434.begin(434.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    if (rc) {
        xTaskNotify(status_task_handle, PTRN_ID_ERR, eSetValueWithOverwrite);
        printf("434 BEGIN ERROR: %d\n", rc);
        return;
    }
    rc = lora_434.getVersion();
    printf("434 Chip version: 0x%02X\n", rc);


    //start lora868 and reconfigure it
    rc = lora_868.begin(868.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    if (rc) {
        xTaskNotify(status_task_handle, PTRN_ID_ERR, eSetValueWithOverwrite);
        printf("868 BEGIN ERROR: %d\n", rc);
        return;
    }
    rc = lora_868.getVersion();
    printf("868 Chip version: 0x%02X\n", rc);


    //enable gpio interupts
    gpio_set_intr_type(DIO01, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(DIO04, GPIO_INTR_POSEDGE);
    gpio_intr_enable(DIO01);
    gpio_intr_enable(DIO04);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DIO01, gpio_isr_handler, (void*) DIO01);
    gpio_isr_handler_add(DIO04, gpio_isr_handler, (void*) DIO04);


    //init done -> change pattern to idle
    xTaskNotify(status_task_handle, PTRN_ID_IDLE, eSetValueWithOverwrite);


    lora_868.setMode(SX127X_OP_MODE_SLEEP);
    lora_434.receiveContinuous();

    //uint32_t start = micros();
    uint32_t bytes_total = 0;


    while(true) {
        if (irq1) {
            irq1 = false;
            uint8_t status = lora_434.checkPayloadIntegrity();
            if (status) {
                switch (status) {
                    case ERR_RX_TIMEOUT:   printf("ERR_RX_TIMEOUT\n"); break;
                    case ERR_CRC_MISMATCH: printf("ERR_CRC_MISSMATCH\n"); break;
                    default: printf("UNKNOWN\n"); break;
                }
                lora_434.clearIrqFlags();
                continue;
            }
            uint8_t payload[256] = {0};
            lora_434.readData(payload);
            bytes_total += lora_434.getPayloadLength();
            lora_434.clearIrqFlags();
            printf("Received payload: %s\n", payload);
            printf("Total bytes received: %lu\n", bytes_total);
            ESP_LOGI(TAG, "Status stack: %u", uxTaskGetStackHighWaterMark(status_task_handle));

        }
        if (irq2) {
            irq2 = false;

        }
        if (irq3) {
            irq3 = false;

        }
        if (irq4) {
            irq4 = false;

        }
    }
}
