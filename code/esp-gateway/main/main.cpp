#include <string>
#include <vector>
#include <set>
#include <map>
#include <array>
#include <queue>
#include <deque>
#include <algorithm>
#include <set>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "mqtt_client.h"
#include "mqtt5_client.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#include "libs/sx127x.hpp"
#include "libs/containers/simpleQueue.hpp"
#include "libs/tinymesh/TinyMeshPacket.hpp"
#include "libs/tinymesh/TinyMeshPacketID.hpp"
#include "libs/tinymesh/TinyMeshManager.hpp"
#include "libs/interfaces/sx127xInterfaceWrapper.hpp"



/*----(Pin defines)----*/
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


#define PAYLOAD_OUT "PING"
#define PAYLOAD_IN  "PONG"
bool answer = false;
unsigned long timer = 0;


using namespace std;


/*----(Instances)----*/
//radios
SX127X lora_434(CS1, RST1, DIO01);
SX127X lora_868(CS4, RST4, DIO04);

//spi
SemaphoreHandle_t spi_mux = xSemaphoreCreateMutex();
spi_device_handle_t dev_handl;


//interfaces
sx127xInterfaceWrapper lora434_it(&lora_434);
sx127xInterfaceWrapper lora868_it(&lora_868);
TinyMeshManager<10, 20> tmm(0, TM_NODE_TYPE_NORMAL, &lora434_it);

//TinyMeshManager tmm(1, TM_NODE_TYPE_NORMAL, &if_manager);



/*----(SX127X callbacks)----*/
void pinMode(uint8_t pin, uint8_t mode) {
    gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
}
void digitalWrite(uint8_t pin, uint8_t lvl) {
    gpio_set_level((gpio_num_t)pin, lvl);
}
int digitalRead(uint8_t pin) {
    return gpio_get_level((gpio_num_t)pin);
}
void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
unsigned long micros() {
    return esp_timer_get_time();
}
void SPIBeginTransfer() {
    xSemaphoreTake(spi_mux, portMAX_DELAY);
    auto ret = spi_device_acquire_bus(dev_handl, portMAX_DELAY);
    if (ret)
        printf("%d\n", ret);
    ESP_ERROR_CHECK(ret);
}
void SPIEndTransfer() {
    spi_device_release_bus(dev_handl);
    xSemaphoreGive(spi_mux);
}
void SPITransfer(uint8_t addr, uint8_t *buffer, size_t length) {
    static const char *TAG = "SPITransfer";

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &addr;
    
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev_handl, &t));

    memset(&t, 0, sizeof(t));
    t.length = length * 8;

    //write
    //if (addr & SX127X_WRITE_MASK)
        t.tx_buffer = buffer;
    //read
    //else
        t.rx_buffer = buffer;

    auto ret = spi_device_polling_transmit(dev_handl, &t);
    if (ret)
        ESP_LOGE(TAG, "Error: %d", ret);
    return;
}

unsigned long millis() {
    return esp_timer_get_time() / 1000;
}


/*----(HELPER FUNCTIONS)----*/
void printBinary(uint32_t num, uint8_t len) {
    for (int i = 1; i < len + 1; i++) {
        printf("%ld", (num >> (len - i)) & 1);
    }
}

void printPacket(TMPacket *packet) {
    printf("Version:             %d\n", packet->getVersion());
    printf("Source address:      %d\n", packet->getSource());
    printf("Destination address: %d\n", packet->getDestination());
    printf("Node type:           %d\n", packet->getNodeType());
    printf("Repeat count:        %d\n", packet->getRepeatCount());
    printf("Sequence:            %d\n", packet->getSequence());
    printf("Message type:        %d\n", packet->getMessageType());
    printf("Data length:         %d\n", packet->getDataLength());
    printf("Data: ");
    for (int i = 0; i < packet->getDataLength(); i++) {
        auto data = packet->getData();
        printBinary(data[i], 8);
        printf(" ");
    }
    printf("\n");
}


/*----(INITIALIZATION)----*/
void gpioInit() {
    //configure default pin states
    gpio_reset_pin(SCK);
    gpio_reset_pin(MISO);
    gpio_reset_pin(MOSI);

    gpio_reset_pin(CS1);
    gpio_reset_pin(RST1);
    gpio_reset_pin(DIO01);
    gpio_reset_pin(CS2);
    gpio_reset_pin(RST2);
    gpio_reset_pin(DIO02);
    gpio_reset_pin(CS3);
    gpio_reset_pin(RST3);
    gpio_reset_pin(DIO03);
    gpio_reset_pin(CS4);
    gpio_reset_pin(RST4);
    gpio_reset_pin(DIO04);

    gpio_reset_pin(SDA);
    gpio_reset_pin(SCL);
    
    gpio_reset_pin(STATUS_LED);

    gpio_reset_pin(NRF_IRQ);
    gpio_reset_pin(NRF_CE);
    gpio_reset_pin(NRF_CS);

    gpio_reset_pin(TX1);
    gpio_reset_pin(RX1);
    gpio_reset_pin(IRG_01);

    gpio_reset_pin(RF_IN);
    gpio_reset_pin(RF_OUT);
    
    gpio_reset_pin(SD_SCK);
    gpio_reset_pin(SD_CS);
    gpio_reset_pin(SD_MOSI);
    gpio_reset_pin(SD_MISO);

    gpio_set_direction(STATUS_LED, GPIO_MODE_INPUT_OUTPUT);
}


void spiInit() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    auto ret = spi_bus_initialize(SPI3_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 20000000,
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL
    };
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &dev_handl);
    ESP_ERROR_CHECK(ret);
}


void configureLora(SX127X *lora, float freq, uint8_t sync_word, uint16_t preamble_len, uint8_t bw, uint8_t sf, uint8_t cr) {
    static const char *TAG = "LoRa config";

    //register callbacks
    lora->registerMicros(micros);
    lora->registerDelay(delay);
    lora->registerPinMode(pinMode, GPIO_MODE_INPUT_OUTPUT, GPIO_MODE_INPUT_OUTPUT);
    lora->registerDigitalWrite(digitalWrite);
    lora->registerDigitalRead(digitalRead);
    lora->registerSPIBeginTransfer(SPIBeginTransfer);
    lora->registerSPIEndTransfer(SPIEndTransfer);
    lora->registerSPITransfer(SPITransfer);

    lora->reset();
    uint8_t rc = lora->begin(freq, sync_word, preamble_len, bw, sf, cr);
    if (rc) {
        ESP_LOGE(TAG, "BEGIN ERROR: %d", rc);
        ESP_LOGE(TAG, "Version: 0x%02X", lora->getVersion());
        return;
    }
    ESP_LOGI(TAG, "Version: 0x%02X", lora->getVersion());
    //mqttLog("LoRa module configured (" + std::to_string(freq) + "MHz)", MQTT_SEVERITY_SUCC);
}



uint8_t requestHandler(TMPacket *request, bool fwd) {
    printf("Got request:\n");
    printPacket(request);
    printf("--------------------------------------\n");
    if (fwd)
        printf("Is to be forwarded\n");

    uint8_t ret = 1;
    if (request->getMessageType() == TM_MSG_CUSTOM && request->getDataLength() == 5) {
        char *data = (char *)(request->getData());
        printf("Got data: %s\n", data);
        if (!strcmp(data, PAYLOAD_IN)) {
            ret = tmm.queuePacket(request->getSource(), TM_MSG_OK);
            printf("Send ret: %d\n", ret);
            answer = true;
            timer = millis();
            return 0;
        }
    }

    
    ret = tmm.queuePacket(request->getSource(), TM_MSG_ERR, &ret, 1);
    printf("Send ret: %d\n", ret);
    return 0;
}

uint8_t responseHandler(TMPacket *request, TMPacket *response) {
    printf("Got response:\n");
    printPacket(response);
    printf("to request:\n");
    printPacket(request);
    printf("--------------------------------------\n");
    return 0;
}


extern "C" void app_main() {
    static const char* TAG = "main";

    gpioInit(); //configure all gpio pins

    spiInit();  //configure and initialize spi
    
    //configure radios
    configureLora(&lora_434, 434.0, 0x12, 8, LORA_BANDWIDTH_250kHz, LORA_SPREADING_FACTOR_7, LORA_CODING_RATE_4_5);
    configureLora(&lora_868, 868.0, 0x24, 8, LORA_BANDWIDTH_250kHz, LORA_SPREADING_FACTOR_7, LORA_CODING_RATE_4_5);
    
    tmm.registerMillis(millis);
    tmm.registerRequestHandler(requestHandler);
    tmm.registerResponseHandler(responseHandler);


    /*----(MAIN LOOP)----*/
    uint8_t last_ret = 256;
    uint8_t ret = 0;
    answer = true;
    while(true) {
        if (answer && millis() - timer > 2000) {
            answer = false;
            timer = millis();
            uint8_t data[] = PAYLOAD_OUT;
            ret = tmm.queuePacket(1, TM_MSG_CUSTOM, data, 5);
            printf("Queued 1 packet: %d\n", ret);
        }

        ret = tmm.loop();
        if (ret != last_ret) {
            last_ret = ret;
            printf("Loop: %d\n", ret);
            printf("Queue size: %d\n", tmm.queueSize());
        }
    }
}
