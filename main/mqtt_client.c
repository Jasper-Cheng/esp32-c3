/*
 * MQTT客户端 - 实现文件
 * 
 * 功能：MQTT连接、发布、订阅
 */

#include "mqtt_client.h"  // 我们的头文件
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"  // ESP-IDF的MQTT客户端头文件（通过mqtt组件提供）
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

/**
 * @brief 获取设备MAC地址作为客户端ID
 */
static void get_client_id(char *client_id, size_t len)
{
    if (s_config.client_id[0] != '\0') {
        strncpy(client_id, s_config.client_id, len - 1);
        client_id[len - 1] = '\0';
        return;
    }
    
    // 使用MAC地址作为默认客户端ID
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(client_id, len, "jasper_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
        
        // 自动订阅控制主题
        char topic_buf[128];
        snprintf(topic_buf, sizeof(topic_buf), "%s/control/+", s_config.prefix);
        esp_mqtt_client_subscribe(client, topic_buf, 1);
        ESP_LOGI(TAG, "Subscribed to: %s", topic_buf);
        
        // 订阅配置主题
        snprintf(topic_buf, sizeof(topic_buf), "%s/config", s_config.prefix);
        esp_mqtt_client_subscribe(client, topic_buf, 1);
        ESP_LOGI(TAG, "Subscribed to: %s", topic_buf);
        
        // 发布在线状态
        snprintf(topic_buf, sizeof(topic_buf), "%s/status", s_config.prefix);
        esp_mqtt_client_publish(client, topic_buf, "online", 0, 1, 0);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        s_connected = false;
        if (s_status_callback) {
            s_status_callback(false);
        }
        break;
        
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT published, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT data received, topic=%.*s, data=%.*s",
                 event->topic_len, event->topic,
                 event->data_len, event->data);
        
        if (s_msg_callback) {
            // 创建临时字符串
            char topic[128] = {0};
            if (event->topic_len < sizeof(topic)) {
                memcpy(topic, event->topic, event->topic_len);
                topic[event->topic_len] = '\0';
            }
            s_msg_callback(topic, (uint8_t*)event->data, event->data_len);
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
esp_err_t mqtt_client_init(mqtt_message_callback_t callback, mqtt_status_callback_t status_callback)
{
    ESP_LOGI(TAG, "Initializing MQTT client");
    
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
        s_config.port = 1883;
    }
    if (strlen(s_config.prefix) == 0) {
        strcpy(s_config.prefix, "jasper-c3");
    }
    
    s_configured = true;
    ESP_LOGI(TAG, "MQTT config: broker=%s:%d, prefix=%s", 
             s_config.broker, s_config.port, s_config.prefix);
    
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
    
    char uri[256];
    if (strlen(s_config.username) > 0) {
        snprintf(uri, sizeof(uri), "mqtt://%s:%s@%s:%d",
                 s_config.username, s_config.password,
                 s_config.broker, s_config.port);
    } else {
        snprintf(uri, sizeof(uri), "mqtt://%s:%d",
                 s_config.broker, s_config.port);
    }
    
    ESP_LOGI(TAG, "Connecting to MQTT: %s (client_id=%s)", uri, client_id);
    
    // 配置MQTT客户端
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .credentials.client_id = client_id,
        .session.keepalive = 60,
        .session.disable_clean_session = false,
    };
    
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
        // 发布离线状态
        if (s_connected) {
            char topic[128];
            snprintf(topic, sizeof(topic), "%s/status", s_config.prefix);
            esp_mqtt_client_publish(s_mqtt_client, topic, "offline", 0, 1, 0);
        }
        
        esp_mqtt_client_stop(s_mqtt_client);
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
    snprintf(full_topic, sizeof(full_topic), "%s/%s", s_config.prefix, topic);
    
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, full_topic, (const char*)data, len, qos, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish to %s", full_topic);
        return ESP_FAIL;
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
    snprintf(full_topic, sizeof(full_topic), "%s/%s", s_config.prefix, topic);
    
    int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, full_topic, qos);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to %s", full_topic);
        return ESP_FAIL;
    }
    
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
    
    snprintf(full_topic, buf_size, "%s/%s", s_config.prefix, relative_topic);
    return full_topic;
}

