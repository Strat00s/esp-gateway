#pragma once
#include <string>
#include <map>
#include "mqtt_client.h"
#include "mqtt5_client.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include "containers/simpleQueue.hpp"



#define MQTT_SEV_INFO 0
#define MQTT_SEV_OK   1
#define MQTT_SEV_WARN 2
#define MQTT_SEV_ERR  3


typedef std::map<std::string, void (*)(const std::string&)> topic_func_map;

typedef struct {
    std::string topic;
    std::string message;
    bool publish;
    uint8_t qos;
    bool retain;
} mqtt_queue_data_t;

typedef struct {
    SimpleQueue<mqtt_queue_data_t> *data_queue;
    topic_func_map *callback_map;
    esp_mqtt_client_handle_t *client;
} task_args;


class SimpleMQTT {
private:
    std::string log_path;
    esp_mqtt_client_handle_t client;
    SimpleQueue<mqtt_queue_data_t> data_queue;
    topic_func_map callback_map;
    TaskHandle_t mqtt_task_handle;

    static void eventhandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static void log_error_if_nonzero(const char *message, int error_code) {}

    static void task(void *args) {
        task_args *arguments = (task_args*)args;

        mqtt_queue_data_t data;
        while(true) {
            //wait for any queued message
            if (arguments->data_queue->read(&data))
                continue;

            //publish message
            if (data.publish) {
                esp_mqtt_client_publish(*(arguments->client), data.topic.c_str(), data.message.c_str(), 0, data.qos, data.retain);
                continue;
            }
            if (arguments->callback_map->find(data.topic) == arguments->callback_map->end())
                continue;

            arguments->callback_map->at(data.topic)(data.message);
        }
    }

public:
    SimpleMQTT(/* args */);
    ~SimpleMQTT();

    /** @brief Set topic for logs
     * 
     * @param path 
     */
    void setLogPath(std::string path);

    /** @brief Initialize mqtt
     * @param uri MQTT broker uri (with login information if needed)
     */
    void init(const char *uri);

    /** @brief Subscribe to a topic with a callback with message as string argument
     * 
     * @param callback
     * @param topic 
     * @param qos 
     */
    void subscribe(void (*)(const std::string&), std::string topic, uint8_t qos);

    /** @brief Publis message to specified topic with specified settings
     * 
     * @param topic Topic to which to publish
     * @param message Message to publish
     * @param qos Quality of service
     * @param retain If messages should be retained by server
     * @param timeout Timeout in ms when accessing underlying data structure
     */
    void publish(std::string topic, std::string message, uint8_t qos = 0, bool retain = true, uint32_t timeout = 10);

    /** @brief 
     * 
     * @param msg 
     * @param severity 
     * @param timeout 
     */
    void log(std::string msg, uint8_t severity = MQTT_SEV_INFO, uint32_t timeout = 10);
};
