/*
 * MQTT客户端 - 实现文件（可复用模块）
 * 
 * 功能：MQTT连接、发布、订阅
 * 
 * 特性：
 * - 通过SN参数初始化，支持多设备复用
 * - 自动生成client_id：device_{sn}
 * - 自动订阅控制主题：device/{sn}/control
 */

#include "mqtt_wrapper.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"  // ESP-IDF的MQTT客户端头文件（通过mqtt组件提供）
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

/* 日志标签 */
static const char* TAG = "MQTT";

/* 全局变量 */
static mqtt_message_callback_t s_msg_callback = NULL;
static mqtt_status_callback_t s_status_callback = NULL;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static mqtt_config_t s_config = {0};
static bool s_connected = false;
static bool s_configured = false;
static char s_sn[64] = {0};  // 保存设备序列号
static char s_lwt_topic[128] = {0};  // LWT主题
static char s_lwt_message[128] = {0};  // LWT消息

/**
 * @brief 获取客户端ID（包含SN作为唯一标识）
 */
static void get_client_id(char *client_id, size_t len)
{
    if (s_config.client_id[0] != '\0') {
        // 如果配置中提供了client_id，直接使用
        strncpy(client_id, s_config.client_id, len - 1);
        client_id[len - 1] = '\0';
        return;
    }
    
    // 自动生成包含SN的client_id格式：device_{SN}
    // 使用SN作为唯一标识，格式：device_M701001
    if (s_sn[0] != '\0') {
        snprintf(client_id, len, "device_%s", s_sn);
    } else {
        // 如果SN未设置，使用默认值
        strncpy(client_id, "device_unknown", len - 1);
        client_id[len - 1] = '\0';
    }
}

/**
 * @brief MQTT事件处理函数
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        if (s_status_callback) {
            s_status_callback(true);
        }
        
        // 自动订阅控制主题（格式：device/{sn}/control）
        char topic_buf[128];
        if (s_sn[0] != '\0') {
            // 订阅控制主题
            snprintf(topic_buf, sizeof(topic_buf), "device/%s/control", s_sn);
            int sub_msg_id = esp_mqtt_client_subscribe(client, topic_buf, 1);
            ESP_LOGI(TAG, "Subscribed to topic: %s (msg_id=%d)", topic_buf, sub_msg_id);
            
            // 发布在线状态（格式：device/{sn}/status）
            snprintf(topic_buf, sizeof(topic_buf), "device/%s/status", s_sn);
            char status_json[128];
            snprintf(status_json, sizeof(status_json), "{\"sn\":\"%s\",\"status\":\"online\"}", s_sn);
            int pub_msg_id = esp_mqtt_client_publish(client, topic_buf, status_json, strlen(status_json), 1, 0);
            ESP_LOGI(TAG, "Published to topic: %s, payload: %s (msg_id=%d)", topic_buf, status_json, pub_msg_id);
        }
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        s_connected = false;
        if (s_status_callback) {
            s_status_callback(false);
        }
        break;
        
    case MQTT_EVENT_SUBSCRIBED:
        {
            char topic[128] = {0};
            if (event->topic_len > 0 && event->topic_len < sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
                ESP_LOGI(TAG, "Subscribed to topic: %s (msg_id=%d)", topic, event->msg_id);
            } else {
                ESP_LOGI(TAG, "MQTT subscribed (msg_id=%d)", event->msg_id);
            }
        }
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        {
            char topic[128] = {0};
            if (event->topic_len > 0 && event->topic_len < sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
                ESP_LOGI(TAG, "Unsubscribed from topic: %s (msg_id=%d)", topic, event->msg_id);
            } else {
                ESP_LOGI(TAG, "MQTT unsubscribed (msg_id=%d)", event->msg_id);
            }
        }
        break;
        
    case MQTT_EVENT_PUBLISHED:
        {
            char topic[128] = {0};
            if (event->topic_len > 0 && event->topic_len < sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
                char payload[256] = {0};
                int payload_len = (event->data_len < sizeof(payload) - 1) ? event->data_len : sizeof(payload) - 1;
                if (payload_len > 0) {
                    memcpy(payload, event->data, payload_len);
                    payload[payload_len] = '\0';
                    ESP_LOGI(TAG, "Published to topic: %s, payload: %.*s (msg_id=%d)", 
                            topic, payload_len, payload, event->msg_id);
                } else {
                    ESP_LOGI(TAG, "Published to topic: %s (msg_id=%d)", topic, event->msg_id);
                }
            } else {
                ESP_LOGI(TAG, "MQTT published (msg_id=%d)", event->msg_id);
            }
        }
        break;
        
    case MQTT_EVENT_DATA:
        {
            char topic[128] = {0};
            if (event->topic_len < sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
            }
            char payload[256] = {0};
            int payload_len = (event->data_len < sizeof(payload) - 1) ? event->data_len : sizeof(payload) - 1;
            if (payload_len > 0) {
                memcpy(payload, event->data, payload_len);
                payload[payload_len] = '\0';
                ESP_LOGI(TAG, "Received data from topic: %s, payload: %.*s (len=%d)", 
                        topic, payload_len, payload, event->data_len);
            } else {
                ESP_LOGI(TAG, "Received data from topic: %s (len=%d)", topic, event->data_len);
            }
            
            if (s_msg_callback) {
                s_msg_callback(topic, (uint8_t*)event->data, event->data_len);
            }
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error: %s", esp_err_to_name(event->error_handle->error_type));
        break;
        
    default:
        break;
    }
}

/**
 * @brief 初始化MQTT客户端
 */
esp_err_t mqtt_client_init(const char *sn, mqtt_message_callback_t callback, mqtt_status_callback_t status_callback)
{
    if (!sn || strlen(sn) == 0) {
        ESP_LOGE(TAG, "Invalid SN parameter");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing MQTT client with SN: %s", sn);
    
    // 保存SN
    strncpy(s_sn, sn, sizeof(s_sn) - 1);
    s_sn[sizeof(s_sn) - 1] = '\0';
    
    s_msg_callback = callback;
    s_status_callback = status_callback;
    s_connected = false;
    s_configured = false;
    
    return ESP_OK;
}

/**
 * @brief 配置MQTT连接参数
 */
esp_err_t mqtt_client_set_config(const mqtt_config_t *config)
{
    if (!config || strlen(config->broker) == 0) {
        ESP_LOGE(TAG, "Invalid MQTT config");
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(&s_config, config, sizeof(mqtt_config_t));
    
    // 设置默认值
    if (s_config.port == 0) {
        s_config.port = 8083;
    }
    
    s_configured = true;
    ESP_LOGI(TAG, "MQTT config set");
    
    return ESP_OK;
}

/**
 * @brief 连接到MQTT服务器
 */
esp_err_t mqtt_client_connect(void)
{
    if (!s_configured) {
        ESP_LOGE(TAG, "MQTT not configured");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_mqtt_client) {
        ESP_LOGI(TAG, "MQTT client already exists, disconnecting first");
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }
    
    // 构建MQTT URI
    char client_id[64];
    get_client_id(client_id, sizeof(client_id));
    
    // 增加缓冲区大小以避免截断警告
    char uri[512];
    int uri_len;
    if (strlen(s_config.username) > 0) {
        uri_len = snprintf(uri, sizeof(uri), "mqtt://%s:%s@%s:%d",
                          s_config.username, s_config.password,
                          s_config.broker, s_config.port);
    } else {
        uri_len = snprintf(uri, sizeof(uri), "mqtt://%s:%d",
                          s_config.broker, s_config.port);
    }
    
    if (uri_len < 0 || uri_len >= sizeof(uri)) {
        ESP_LOGE(TAG, "URI too long");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Connecting to MQTT broker: %s (client_id=%s)", uri, client_id);
    
    // 配置MQTT客户端
    esp_mqtt_client_config_t mqtt_cfg = {0};
    
    // 配置基本连接参数
    mqtt_cfg.broker.address.uri = uri;
    mqtt_cfg.credentials.client_id = client_id;
    // 缩短keepalive时间，让LWT更快触发（30秒，服务器会在约45秒后触发LWT）
    mqtt_cfg.session.keepalive = 30;
    mqtt_cfg.session.disable_clean_session = false;
    
    // 配置LWT遗嘱（Last Will and Testament）
    // 当客户端异常断开时，MQTT服务器会自动发布此消息
    // 注意：LWT需要等待keepalive超时（约1.5倍keepalive时间）才会触发
    if (s_sn[0] != '\0') {
        snprintf(s_lwt_topic, sizeof(s_lwt_topic), "device/%s/status", s_sn);
        snprintf(s_lwt_message, sizeof(s_lwt_message), "{\"sn\":\"%s\",\"status\":\"offline\"}", s_sn);
        
        mqtt_cfg.session.last_will.topic = s_lwt_topic;
        mqtt_cfg.session.last_will.msg = s_lwt_message;
        mqtt_cfg.session.last_will.msg_len = strlen(s_lwt_message);
        mqtt_cfg.session.last_will.qos = 1;
        mqtt_cfg.session.last_will.retain = 0;  // 不保留消息
        
        ESP_LOGI(TAG, "LWT configured: topic=%s, message=%s, keepalive=%d (LWT will trigger after ~%d seconds)", 
                s_lwt_topic, s_lwt_message, mqtt_cfg.session.keepalive, 
                (int)(mqtt_cfg.session.keepalive * 1.5));
    }
    
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return ESP_FAIL;
    }
    
    // 注册事件处理器
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // 启动连接
    esp_err_t ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief 断开MQTT连接
 */
esp_err_t mqtt_client_disconnect(void)
{
    if (s_mqtt_client) {
        // 发布离线状态（格式：device/{sn}/status）
        // 在断开前主动发送离线消息，确保服务器收到
        if (s_connected && s_sn[0] != '\0') {
            char topic[128];
            snprintf(topic, sizeof(topic), "device/%s/status", s_sn);
            char status_json[128];
            snprintf(status_json, sizeof(status_json), "{\"sn\":\"%s\",\"status\":\"offline\"}", s_sn);
            
            // 使用QoS 1发布，确保消息至少被服务器接收一次
            int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, status_json, strlen(status_json), 1, 0);
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Published offline status to topic: %s, payload: %s (msg_id=%d)", 
                        topic, status_json, msg_id);
                
                // 等待消息发送完成（QoS 1需要等待PUBACK）
                // 给足够的时间让消息发送和确认
                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                ESP_LOGW(TAG, "Failed to publish offline status to topic: %s", topic);
            }
        }
        
        ESP_LOGI(TAG, "Disconnecting MQTT client");
        // 先停止客户端（会发送DISCONNECT包）
        esp_mqtt_client_stop(s_mqtt_client);
        // 等待断开完成
        vTaskDelay(pdMS_TO_TICKS(100));
        // 销毁客户端
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        s_connected = false;
    }
    
    return ESP_OK;
}

/**
 * @brief 发布消息
 */
esp_err_t mqtt_client_publish(const char *topic, const uint8_t *data, int len, int qos)
{
    if (!s_mqtt_client || !s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    char full_topic[128];
    if (s_sn[0] != '\0') {
        snprintf(full_topic, sizeof(full_topic), "device/%s/%s", s_sn, topic);
    } else {
        snprintf(full_topic, sizeof(full_topic), "device/unknown/%s", topic);
    }
    
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, full_topic, (const char*)data, len, qos, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to topic: %s", full_topic);
        return ESP_FAIL;
    }
    
    // 记录发布日志（限制payload长度以避免日志过长）
    char payload_preview[128] = {0};
    int preview_len = (len < sizeof(payload_preview) - 1) ? len : sizeof(payload_preview) - 1;
    if (preview_len > 0) {
        memcpy(payload_preview, data, preview_len);
        payload_preview[preview_len] = '\0';
        ESP_LOGI(TAG, "Publishing to topic: %s, payload: %.*s (len=%d, qos=%d, msg_id=%d)", 
                full_topic, preview_len, payload_preview, len, qos, msg_id);
    } else {
        ESP_LOGI(TAG, "Publishing to topic: %s (len=%d, qos=%d, msg_id=%d)", 
                full_topic, len, qos, msg_id);
    }
    
    return ESP_OK;
}

/**
 * @brief 订阅主题
 */
esp_err_t mqtt_client_subscribe(const char *topic, int qos)
{
    if (!s_mqtt_client || !s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    char full_topic[128];
    if (s_sn[0] != '\0') {
        snprintf(full_topic, sizeof(full_topic), "device/%s/%s", s_sn, topic);
    } else {
        snprintf(full_topic, sizeof(full_topic), "device/unknown/%s", topic);
    }
    
    int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, full_topic, qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to topic: %s", full_topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Subscribing to topic: %s (qos=%d, msg_id=%d)", full_topic, qos, msg_id);
    
    return ESP_OK;
}

/**
 * @brief 获取MQTT连接状态
 */
bool mqtt_client_is_connected(void)
{
    return s_connected;
}

/**
 * @brief 获取完整主题路径
 */
const char* mqtt_client_get_full_topic(const char *relative_topic, char *full_topic, size_t buf_size)
{
    if (!relative_topic || !full_topic) {
        return NULL;
    }
    
    if (s_sn[0] != '\0') {
        snprintf(full_topic, buf_size, "device/%s/%s", s_sn, relative_topic);
    } else {
        snprintf(full_topic, buf_size, "device/unknown/%s", relative_topic);
    }
    return full_topic;
}

/**
 * @brief 获取当前MQTT配置信息（JSON格式）
 */
int mqtt_client_get_config_json(char *json_buf, size_t buf_size)
{
    if (!json_buf || buf_size == 0) {
        return 0;
    }
    
    if (!s_configured || strlen(s_config.broker) == 0) {
        // 未配置，返回状态信息
        return snprintf(json_buf, buf_size, "{\"status\":\"not_configured\"}");
    }
    
    if (s_connected) {
        // 已连接，返回broker、port和状态
        return snprintf(json_buf, buf_size, 
                       "{\"broker\":\"%s\",\"port\":%d,\"status\":\"connected\"}",
                       s_config.broker, s_config.port);
    } else {
        // 未连接，返回broker、port和状态
        return snprintf(json_buf, buf_size, 
                       "{\"broker\":\"%s\",\"port\":%d,\"status\":\"disconnected\"}",
                       s_config.broker, s_config.port);
    }
}

