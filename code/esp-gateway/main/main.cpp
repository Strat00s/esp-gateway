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
#include "libs/interfaces/InterfaceManager.hpp"
#include "libs/interfaces/sx127xInterfaceWrapper.hpp"
#include "creds.h"

/*----(CONFIGURATION)----*/
//comment out creds.h and configure your wifi and mqtt here
#ifndef HIDDEN_CREDS
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWD"
#define MQTT_URI  "YOUR_MQTTT_URI"
#endif

/*----(FUNCTION MACORS)----*/
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define CONCATENATE(a, b) a b

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


#define NODE_NAME "Gateway"
#define NODE_LOCATION "Main hall"
#define NODE_ADDRESS 1


/*----(WiFi config)----*/
//wifi config
//#define WIFI_SSID TEST_WIFI_SSID
//#define WIFI_PASS TEST_WIFI_PASS
#define WIFI_MAXIMUM_RETRY  5
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1


/*----(MQTT config)----*/
/*
/nodes
    /<name>
        /address
        /location
        /type
        /last_action
        /status
        /alive
/sensors
    /<location>
        /<name>
            /<type>
                value
*/
//main node topics and variables
#define MQTT_NODES_PATH   "nodes"
#define MQTT_GATEWAY_PATH CONCATENATE(CONCATENATE(MQTT_NODES_PATH, "/"), TOSTRING(NODE_ADDRESS))
#define MQTT_INTERFACE_IN  "/interface/in"
#define MQTT_INTERFACE_OUT "/interface/out"
#define MQTT_SENSOR_PATH   "sensors" //sensors/<location>/<name>/<type>
#define MQTT_ALIVE         "/alive"
#define MQTT_STATUS        "/status"
#define MQTT_LOG           "/log"
#define MQTT_TYPE          "/type"
#define MQTT_LAST_PACKET   "/last_packet"
#define MQTT_LOCATION      "/location"

//mqtt status severity
#define MQTT_SEVERITY_SUCC    0
#define MQTT_SEVERITY_INFO    1
#define MQTT_SEVERITY_WARNING 2
#define MQTT_SEVERITY_ERROR   3


//Networking
#define INTERFACE_COUNT 3 //number of implemented and usable interfaces
#define TIME_TO_STALE   TM_CLEAR_TIME //time (ms) for a record to become stale (when forwarding or when waiting for a response)


//blink pattern ids
#define PTRN_ID_ERR  0
#define PTRN_ID_IDLE 1
#define PTRN_ID_INIT 2


using namespace std;


/*----(Instances)----*/
//radios
SX127X lora_434(CS1, RST1, DIO01);
SX127X lora_868(CS4, RST4, DIO04);

//spi
SemaphoreHandle_t spi_mux = xSemaphoreCreateMutex();
spi_device_handle_t dev_handl;

//mqtt
esp_mqtt_client_handle_t mqtt_client;
esp_netif_ip_info_t ip_info;

//interfaces
//sx127xInterfaceWrapper lora434_it(&lora_434);
//sx127xInterfaceWrapper lora868_it(&lora_868);
sx127xInterfaceWrapper lora434_it(&lora_434);
sx127xInterfaceWrapper lora868_it(&lora_868);
InterfaceManager if_manager;

TinyMeshManager tmm(1, TM_NODE_TYPE_NORMAL, &if_manager);


/*----(STRUCTURES)----*/
typedef struct {
    std::string topic;
    std::string message;
    bool publish;
    uint8_t qos;
    bool retain;
} mqtt_queue_data_t;

//Node struct
typedef struct {
    std::string name = "";
    std::string location = "";
    std::deque<std::string> sensor_types;
    uint8_t address;
    uint8_t node_type;
    InterfaceWrapper *interfaces[INTERFACE_COUNT];
    uint8_t interface_cnt;
} node_info_t;
node_info_t this_node;
std::map<uint8_t, node_info_t> node_map;

//simple thread safe queues
SimpleQueue<mqtt_queue_data_t> mqtt_data_queue; //for mqtt task
//SimpleQueue<packet_t> default_response_q; //default response handler queue
//std::map<uint8_t, SimpleQueue<packet_t>*> request_queue; //for any task

//task handles
static TaskHandle_t led_task_handle;              //blinking
static TaskHandle_t mqtt_task_handle;             //handles mqtt listening and publishing
static TaskHandle_t ping_task_handle;             //ping service task
static TaskHandle_t default_response_task_handle; //default response (does pretty much nothing)


/*----(Rest of variables)----*/
//wifi
static EventGroupHandle_t wifi_event_group;
static int s_retry_num = 0;
std::string ip_addr;


/*----(FUNCTION DEFINITIONS)----*/
//void mqttPublishQueue(std::string topic, std::string data, uint32_t timeout = 10);
//void mqttLog(std::string msg, uint8_t severity = MQTT_SEVERITY_INFO, uint32_t timeout = portMAX_DELAY);
//void mqttLogMessage(packet_t packet);
//void handleCommands(std::string command);

//uint8_t sendDataOnInterface(InterfaceWrapper *it, uint8_t *data, uint8_t len);
//void sendPacket(packet_t packet, SimpleQueue<packet_t> *queue = &default_response_q);
//void handleResponse(packet_t packet, InterfaceWrapper *interface);
//void handleRequest(packet_t packet, InterfaceWrapper *interface, bool repeat = false);
void aliveTask(void *args);
void mqttTask(void *args);
void pingTask(void *args);
void defaultResponseTask(void *args);



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
    if (ret != 0)
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

/** @brief Get current time as a string
 * 
 * @return std::string - DD.MM.YYYY HH:MM:SS
 */
std::string getTimeStr() {
    time_t now;
    time(&now);
    struct tm timeinfo = {0};
    localtime_r(&now, &timeinfo);
    char buf[28] = {0};
    sprintf(buf, "%02d.%d.%04d %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon, (1900 + timeinfo.tm_year), timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return std::string(buf);
}

//mqtt magic from espressif
static void log_error_if_nonzero(const char *message, int error_code) {
    static const char *TAG = "log_error";
    if (error_code != 0)
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
}


/** @brief  Convert packet into a json string*/
std::string packet2json(TMPacket *packet) {
    std::string result = "{";
    result += "\"version\": "      + std::to_string(packet->getVersion())     + ",";
    result += "\"source\": "       + std::to_string(packet->getSource())      + ",";
    result += "\"destination\": "  + std::to_string(packet->getDestination()) + ",";
    result += "\"repeate cnt\": "  + std::to_string(packet->getRepeatCount()) + ",";
    result += "\"node type\": ";
    if (packet->getNodeType())
        result += "\"low power\",";
    else
        result += "\"normal\",";
    result += "\"message type\": ";
    switch (packet->getMessageType()) {
        case TM_MSG_OK:       result += "\"ok\",";       break;
        case TM_MSG_ERR:      result += "\"error\",";    break;
        case TM_MSG_PING:     result += "\"ping\",";     break;
        case TM_MSG_CUSTOM:   result += "\"custom\",";   break;
        default:              result += "\"other (\"," + to_string(packet->getMessageType()) + "\")\""; break;
    }
    result += "\"data length\": " + std::to_string(packet->getDataLength()) + ",";
    result += "\"data\": ";
    if (!packet->getDataLength()){
        result += "\"NULL\"}";
        return result;
    }
    result += "\"";

    for (size_t i = 0; i < packet->getDataLength(); i++) {
        auto data = packet->getData();
        for (int ii = 0; ii < 8; ii++)
            result += data[i] & ((uint8_t)1 << (7 - ii)) ? '1' : '0';
        if (i < packet->getDataLength() - 1)
            result += ' ';
    }

    result += "\"}";
    return result;
}

/** @brief Convert string to specified integer type using stoi.
 * 
 * @tparam T Result type
 * @param result Result of conversion
 * @param str String to convert
 * @return uint8_t 0 on success, 1 if string contains non-numerical character
 */
template<typename T>
uint8_t str2int(T *result, std::string str) {
    for (char const &c : str) {
        if (!std::isdigit(c) && c != '-') {
            //mqttLog("Failed to convert string '"+ str +"' to number.", MQTT_SEVERITY_ERROR);
            return 1;
        }
    }
    *result = std::stoi(str);
    return 0;
}


/*----(MQTT HELPERS)----*/
/** @brief Add messages to be published to mqtt data queue
 * 
 * @param topic Path to which to publish the data
 * @param data Data to be published
 * @param timeout Timeout in ticks how long to wait if queue is full
 */
void mqttPublishQueue(std::string topic, std::string data, uint32_t timeout = 10) {
    mqtt_data_queue.write({.topic = topic, .message = data, .publish = true, .qos = 0, .retain = true}, timeout);
}

/** @brief Simple logging to mqtt 
 * 
 * @param msg Message to write to status topic
 * @param severity severity prefix
 * @param timeout Timeout in ticks how long to wait if queue is full
 */
void mqttLog(std::string msg, uint8_t severity, uint32_t timeout = 10) {
    std::string data = getTimeStr();

    switch (severity) {
        case MQTT_SEVERITY_SUCC:    data += " [ OK ] " + msg; break;
        case MQTT_SEVERITY_INFO:    data += " [INFO] " + msg; break;
        case MQTT_SEVERITY_WARNING: data += " [WARN] " + msg; break;
        default:                    data += " [FAIL] " + msg; break;
    }
    mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH,MQTT_LOG), data, timeout);
}

/** @brief Log packet to node*/
void mqttLogMessage(TMPacket *packet) {
    if (packet->getSource() == TM_DEFAULT_ADDRESS)
        return;

    std::string node_path = MQTT_NODES_PATH "/" + std::to_string(packet->getSource());

    mqttPublishQueue(node_path + MQTT_ALIVE, getTimeStr()); //node is alive
    mqttPublishQueue(node_path + MQTT_LAST_PACKET, packet2json(packet));
}

/** @brief MQTT command handling
 * 
 * @param command Command string
 */
void handleCommands(std::string command) {
    static const char *TAG = "cmd_handle";

    ESP_LOGI(TAG, "Command: %s", command.c_str());

    //split input by spaces
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream stream(command);
    while (std::getline(stream, token, ' '))
        tokens.push_back(token);

    if (tokens.size() == 0) {
        ESP_LOGE(TAG, "No tokens");
        mqttLog("No tokens", MQTT_SEVERITY_ERROR);
        return;
    }

    //if (tokens[0] == "PURGE" && tokens.size() == 1) {
    //    //TODO
    //}

    if (tokens[0] == "SET" && tokens.size() == 4) {
        uint8_t address;

        if (str2int(&address, tokens[2]))
            return;

        auto search = node_map.find(address);
        if (search == node_map.end()) {
            mqttLog("Node '"+ tokens[2] +"' not found.", MQTT_SEVERITY_WARNING);
            return;
        }

        auto node = search->second;
        if (tokens[1] == "LOCATION") {
            node.location = tokens[3];
            mqttPublishQueue(CONCATENATE(MQTT_NODES_PATH, "/") + tokens[2] + MQTT_LOCATION, tokens[3]);
            mqttLog("Location of '"+ tokens[2] +"' changed to '"+ tokens[3] +"'.", MQTT_SEVERITY_SUCC);
        }
        if (tokens[1] == "NAME") {
            node.name = tokens[3];
            mqttPublishQueue(CONCATENATE(MQTT_NODES_PATH, "/") + tokens[2], tokens[3]);
            mqttLog("Name of '"+ tokens[2] +"' changed to '"+ tokens[3] +"'.", MQTT_SEVERITY_SUCC);
        }

        node_map[address] = node; //update the node
    }

    //SEND MSG_TYPE DESTINATION DATA
    if (tokens[0] == "SEND" && tokens.size() >= 3) {

        uint8_t destination;
        if (str2int(&destination, tokens[2]))
            return;

        if (tokens[1] == "PING" && tokens.size() == 3)
            xTaskCreate(pingTask, "ping", 4096, (void *)&destination, 10, &ping_task_handle);
    }
}


/*----(MAIN EVENT HANDLER)----*/
static void eventLoopHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static const char *TAG = "Event Handler";

    //WiFi event
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < WIFI_MAXIMUM_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "retry to connect to the AP");
                }
                else
                    xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG,"Failed to connect to AP");
                break;

            default: ESP_LOGW(TAG, "Unhandled ID %ld for WIFI_EVENT", event_id); break;
        }
        return;
    }

    //IP event
    if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ip_info = event->ip_info;   //save ip info
                char buf[17] = {0};
                sprintf(buf, IPSTR, IP2STR(&ip_info.ip));
                ip_addr = buf;
                ESP_LOGI(TAG, "Got ip: %s", ip_addr.c_str());
                s_retry_num = 0;
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            }
        
            default: ESP_LOGW(TAG, "Unhandled ID %ld for IP_EVENT", event_id); break;
        }
        return;
    }

    //MQTT event
    if (event_base == (esp_event_base_t)"MQTT_EVENTS") {
        esp_mqtt_event_handle_t event = (esp_mqtt_event_t *)event_data;
        esp_mqtt_client_handle_t client = event->client;

        switch ((esp_mqtt_event_id_t)event_id) {
        //subscribe to everything on connect
            case MQTT_EVENT_CONNECTED: {
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

                mqttPublishQueue(MQTT_GATEWAY_PATH, NODE_NAME);
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, "/IP"),              ip_addr);
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LOCATION),      this_node.location);
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_TYPE),          std::to_string(this_node.node_type));
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, "/CMD"),             " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_IN),  " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_OUT), " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LAST_PACKET),   " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_STATUS),        " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LOG),           " ");
                mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_ALIVE),         " ");
                
                //subscribe to everything
                esp_mqtt_client_subscribe(mqtt_client, CONCATENATE(MQTT_GATEWAY_PATH, "/CMD"),            0);
                esp_mqtt_client_subscribe(mqtt_client, CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_IN), 0);
                break;
            }

            case MQTT_EVENT_DISCONNECTED: ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED"); break;
            case MQTT_EVENT_SUBSCRIBED:   ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");   break;
            case MQTT_EVENT_UNSUBSCRIBED: ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED"); break;
            case MQTT_EVENT_PUBLISHED:    break; //ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);    break;

            //got data on subscribed topic
            case MQTT_EVENT_DATA: {
                mqtt_queue_data_t data = {
                    .topic = std::string(event->topic, event->topic_len),
                    .message = std::string(event->data, event->data_len),
                    .publish = false,
                    .qos = 0,
                    .retain = true
                };
                mqtt_data_queue.write(data);
                break;
            }

            case MQTT_EVENT_BEFORE_CONNECT: ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT"); break;
            case MQTT_EVENT_DELETED:        ESP_LOGI(TAG, "MQTT_EVENT_DELETED");        break;
            case MQTT_USER_EVENT:           ESP_LOGI(TAG, "MQTT_USER_EVENT");           break;

            case MQTT_EVENT_ERROR:
                ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
                ESP_LOGE(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
                if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                    log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                    log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                    log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                    ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
                }
                break;

            default:
                ESP_LOGW(TAG, "Other event id:%d", event->event_id);
                break;
        }
        return;
    }

    ESP_LOGW(TAG, "Unhandled event:\n  Base: `%s`\n  ID: %ld", event_base, event_id);
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

void wifiInit() {
    static const char *TAG = "WiFi Init";

    ESP_LOGI(TAG, "Starting WiFi");

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_LOGI(TAG, "WiFi initialized");

    //register default event loop handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventLoopHandler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventLoopHandler, NULL, NULL));

    ESP_LOGI(TAG, "WiFi event loop handlers registered");

    //mostly default wifi settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    //start wifi in station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "WiFi started in station mode");

    //wait for wifi to connect or fail
    wifi_event_group = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
        ESP_LOGI(TAG, "Connected to AP SSID: %s pswd: %s", WIFI_SSID, WIFI_PASS);
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s pswd: %s", WIFI_SSID, WIFI_PASS);
        while(true);
    }
    else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        while(true);
    }

    mqttLog("Connected to '" + std::string(WIFI_SSID) + "'. IP: " + ip_addr, MQTT_SEVERITY_SUCC);
}

void mqttInit() {
    static const char *TAG = "MQTT Init";

    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .message_expiry_interval = 10,
        .payload_format_indicator = true,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_URI,
            },
        },
        .session = {
            .last_will = {
                .topic = "topic/will",
                .msg = "I will leave",
                .msg_len = 12,
                .qos = 1,
                .retain = true,
            },
            .protocol_ver = MQTT_PROTOCOL_V_5,
        },
        .network = {
            .disable_auto_reconnect = true,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, eventLoopHandler, NULL));
    esp_mqtt_client_start(mqtt_client);

    mqttLog("MQTT configured", MQTT_SEVERITY_SUCC);
}

void sntpInit() {
    static const char *TAG = "sntp_init";

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "cz.pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_set_sync_interval(3600000);
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };

    ESP_LOGI(TAG, "Syncing time");
    while(sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
        vTaskDelay(pdMS_TO_TICKS(500));

    time(&now);

    //set timezone
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    std::string curr_time = getTimeStr();
    ESP_LOGI(TAG, "Current time: %s", curr_time.c_str());

    mqttLog("NTP configured. Current time: " + curr_time, MQTT_SEVERITY_SUCC);
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
        xTaskNotify(led_task_handle, PTRN_ID_ERR, eSetValueWithOverwrite);
        ESP_LOGE(TAG, "BEGIN ERROR: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "LoRa configured: 0x%02X", lora->getVersion());
    //mqttLog("LoRa module configured (" + std::to_string(freq) + "MHz)", MQTT_SEVERITY_SUCC);
}


/*----(TASKS)----*/
/** @brief Simple alive blink task that blinks every 15s and published alive message*/
void aliveTask(void *args) {
    TickType_t last_wakeup = xTaskGetTickCount();
    while (true) {
        gpio_set_level(STATUS_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(STATUS_LED, 0);
        mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_ALIVE), getTimeStr());
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(15000))
    }
}

/** @brief Task responsible for mqtt reading and writing */
//void mqttTask(void *args) {
//    static const char* TAG = "MQTT Task";
//
//    //ESP_LOGI(TAG, "MQTT task started");
//    //ESP_LOGI(TAG, "MQTT queue: %d", mqtt_data_queue.size());
//    //mqttLog("MQTT task started");
//
//    mqtt_queue_data_t data;
//
//    while(true) {
//        //wait for any queued message
//        uint8_t ret = mqtt_data_queue.read(&data);
//        if (ret)
//            continue;
//
//        //publish message
//        if (data.publish) {
//            //ESP_LOGI(TAG, "Queued publish message:\n\t%s: %s", data.topic.c_str(), data.message.c_str());
//            esp_mqtt_client_publish(mqtt_client, data.topic.c_str(), data.message.c_str(), 0, 1, 1);
//            continue;
//        }
//
//        //receive message
//        //ESP_LOGI(TAG, "Queued received message:\n\t%s: %s", data.topic.c_str(), data.message.c_str());
//
//        if (data.topic.contains("CMD")) {
//            handleCommands(data.message);
//        }
//        if (data.topic.contains("interface/in")) {
//            mqtt_if.addData(data.message);
//        }
//    }
//}


uint8_t requestHandler(TMPacket *request, bool fwd) {
    printf("Got request:\n");
    printPacket(request);
    printf("--------------------------------------\n");
    if (fwd)
        printf("Is to be forwarded\n");

    uint8_t data = 1;
    if (tmm.queuePacket(request->getSource(), TM_MSG_ERR, &data, 1))
        printf("Failed to send response\n");
    else
        printf("Sent response\n");

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

    //create default alive task
    xTaskCreate(aliveTask, "alive", 1024, nullptr, 1, &led_task_handle);

    //initialize nvs partition
    auto ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    //initialize tcp/ip stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "TCP/IP stack initialized");

    //create default even loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    spiInit();  //configure and initialize spi
    //wifiInit(); //configure and connect to wifi
    //sntpInit(); //initialize sntp and start hourly sync
    //mqttInit(); //configure and subscribe to mqtt server
    //xTaskCreate(mqttTask, "MQTTTask", 8192, nullptr, 5, &mqtt_task_handle);


    //create default response task for handling incoming packets
    //xTaskCreate(defaultResponseTask, "response", 4096, nullptr, 10, &default_response_task_handle);


    //configure radios
    configureLora(&lora_434, 434.0, 0x12, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    configureLora(&lora_868, 868.0, 0x24, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    if_manager.addInterface(&lora434_it);
    //if_manager.addInterface(&lora868_it);

    for (uint8_t i = 0; i < if_manager.getInterfaceCount(); i++) {
        printf("Starting reception on interface %d: ", if_manager.getInterface(i)->getType());
        if (!if_manager.getInterface(i)->startReception())
            printf("ok\n");
        else {
            printf("err\n");
        }
    }

    tmm.registerMillis(millis);
    tmm.registerRequestHandler(requestHandler);
    tmm.registerResponseHandler(responseHandler);

    /*----(MAIN LOOP)----*/
    auto timer = millis();
    uint8_t last_ret = 0;
    uint8_t last_stat = 0;
    while(true) {
        if (millis() - timer > 5000) {
            timer = millis();
            uint8_t data = 0;
            uint8_t ret = tmm.queuePacket(2, TM_MSG_CUSTOM, &data, 0);
            printf("Queue: %d\n", ret);
        }

        auto ret = tmm.loop();
        if (ret != last_ret) {
            last_ret = ret;
            printf("loop: %d %d\n", ret, tmm.getStatus());
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
