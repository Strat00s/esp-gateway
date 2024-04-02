#include "simpleMQTT.hpp"
#include <esp_log.h>



SimpleMQTT::SimpleMQTT(/* args */) {
}

SimpleMQTT::~SimpleMQTT() {
}


void SimpleMQTT::setLogPath(std::string path) {
    this->log_path = path;
}

void SimpleMQTT::init(const char *uri) {
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
                .uri = uri,
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

    this->client = esp_mqtt_client_init(&mqtt5_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, eventhandler, (void*)&(this->data_queue)));
    esp_mqtt_client_start(this->client);

    log("MQTT configured", MQTT_SEV_OK); 
    task_args args = {
        .data_queue = &this->data_queue,
        .callback_map = &this->callback_map,
        .client = &this->client
    };

    xTaskCreate(task, "MQTTTask", 8192, (void*)&args, 5, &mqtt_task_handle);
}

void SimpleMQTT::subscribe(void (*fnc)(const std::string&), std::string topic, uint8_t qos) {
    auto search = callback_map.find(topic);
    if (search == callback_map.end())
        callback_map.insert({topic, fnc});
    else
        callback_map[topic] = fnc;
}

void SimpleMQTT::publish(std::string topic, std::string message, uint8_t qos, bool retain, uint32_t timeout) {
    mqtt_queue_data_t data = {
        .topic = topic,
        .message = message,
        .publish = true,
        .qos = qos,
        .retain = retain
    };

    data_queue.write(data, timeout);
}

void SimpleMQTT::log(std::string msg, uint8_t severity, uint32_t timeout) {
    std::string type;
    switch (severity) {
        case MQTT_SEV_OK:   type = "[ OK ]"; break;
        case MQTT_SEV_WARN: type = "[WARN]"; break;
        case MQTT_SEV_ERR:  type = "[FAIL]"; break;
        default:            type = "[INFO]"; break;
    }

    msg = type + msg;

    publish(log_path + "/" + type, msg, 0, true, timeout);
    publish(log_path, msg, 0, true, timeout);
}



void SimpleMQTT::eventhandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static const char* TAG = "MQTT EVENT HANDLER";
    if (event_base == (esp_event_base_t)"MQTT_EVENTS")
        return;

    SimpleQueue<mqtt_queue_data_t> *data_queue = (SimpleQueue<mqtt_queue_data_t>*)arg;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_t *)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    //subscribe to everything on connect
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            //mqttPublishQueue(MQTT_GATEWAY_PATH, NODE_NAME);
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, "/IP"),              ip_addr);
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LOCATION),      this_node.location);
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_TYPE),          std::to_string(this_node.node_type));
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, "/CMD"),             " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_IN),  " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_OUT), " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LAST_PACKET),   " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_STATUS),        " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_LOG),           " ");
            //mqttPublishQueue(CONCATENATE(MQTT_GATEWAY_PATH, MQTT_ALIVE),         " ");
            
            //subscribe to everything
            //esp_mqtt_client_subscribe(mqtt_client, CONCATENATE(MQTT_GATEWAY_PATH, "/CMD"),            0);
            //esp_mqtt_client_subscribe(mqtt_client, CONCATENATE(MQTT_GATEWAY_PATH, MQTT_INTERFACE_IN), 0);
            break;
        }

        case MQTT_EVENT_DISCONNECTED: ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");                           break;
        case MQTT_EVENT_SUBSCRIBED:   ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);   break;
        case MQTT_EVENT_UNSUBSCRIBED: ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id); break;
        case MQTT_EVENT_PUBLISHED:    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);    break;

        case MQTT_EVENT_DATA: {
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
            mqtt_queue_data_t data = {
                .topic = std::string(event->topic, event->topic_len),
                .message = std::string(event->data, event->data_len),
                .publish = false,
                .qos = 0,
                .retain = true
            };
            data_queue->write(data);
            break;
        }

        case MQTT_EVENT_BEFORE_CONNECT: ESP_LOGW(TAG, "MQTT_EVENT_BEFORE_CONNECT"); break;
        case MQTT_EVENT_DELETED:        ESP_LOGW(TAG, "MQTT_EVENT_DELETED");        break;
        case MQTT_USER_EVENT:           ESP_LOGW(TAG, "MQTT_USER_EVENT");           break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;

        default:
            ESP_LOGW(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return;
}
