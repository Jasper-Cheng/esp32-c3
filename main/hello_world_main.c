/*
 * ESP32-C3 BLE LED控制器 - 主程序
 * 
 * 功能：
 * 1. 提供蓝牙BLE服务（可连接，支持读写和通知）
 * 2. 接收60字节数据，每字节控制一个LED（0=灭，1-7为不同颜色）
 * 3. 通过GPIO1输出WS2812 LED控制信号
 * 4. 通过BLE控制TD-8120MG舵机角度
 * 5. 通过M701SC传感器监测空气质量
 * 
 * 架构：
 * - BLE服务层：ble_service.c/h - 处理蓝牙通信
 * - LED驱动层：ws2812_driver.c/h - 控制WS2812 LED
 * - 舵机驱动层：servo_driver.c/h - 控制TD-8120MG舵机
 * - 传感器驱动层：m701_sensor.c/h - 读取M701SC空气质量数据
 * - 应用层：hello_world_main.c - 协调各模块工作
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ble_service.h"
#include "ws2812_driver.h"
#include "servo_driver.h"
#include "m701_sensor.h"
#include "wifi_manager.h"
#include "mqtt_wrapper.h"
#include "cJSON.h"

/* 日志标签 */
static const char* TAG = "MAIN";

/* MQTT配置队列 */
#define MQTT_CONFIG_QUEUE_SIZE 5
static QueueHandle_t s_mqtt_config_queue = NULL;
static bool s_config_message_sent = false;  // 标志：配置消息是否已发送

/* WiFi配置队列（BLE 回调中只投递到队列，避免在 BTC_TASK 中执行 NVS/WiFi 导致栈溢出） */
#define WIFI_CONFIG_QUEUE_SIZE 3
#define WIFI_SSID_MAX 64
#define WIFI_PASSWORD_MAX 64
typedef struct {
    char ssid[WIFI_SSID_MAX];
    char password[WIFI_PASSWORD_MAX];
} wifi_config_msg_t;
static QueueHandle_t s_wifi_config_queue = NULL;

/* NVS命名空间 */
#define NVS_NAMESPACE "mqtt_config"
#define NVS_KEY_MQTT_CONFIG "mqtt_cfg"
#define NVS_NAMESPACE_WIFI "wifi_config"
#define NVS_KEY_WIFI_CONFIG "wifi_cfg"

/* 前向声明 */
static esp_err_t load_mqtt_config_from_nvs(char *config_json, size_t max_len);
static void process_mqtt_config(const char *config_json);
static esp_err_t save_wifi_config_to_nvs(const char *ssid, const char *password);
static esp_err_t load_wifi_config_from_nvs(char *ssid, size_t ssid_size, char *password, size_t password_size);

/* 设备SN定义（唯一标识） */
// SN定义（需要在mqtt_wrapper.c中extern引用）
const char SENSOR_SN[] = "M701001-220403-000001";        // M701传感器序列号

/**
 * @brief LED数据接收回调函数
 * 
 * 当通过蓝牙接收到LED控制数据时调用
 * 
 * @param led_data 60字节LED控制数组
 */
static void on_led_data_received(uint8_t *led_data)
{
    // 更新LED状态
    // ws2812_update_leds(led_data);
}

/**
 * @brief 舵机角度接收回调函数
 * 
 * 当通过蓝牙接收到舵机角度数据时调用
 * 
 * @param angle 目标角度 (0.0 ~ 270.0度)
 */
static void on_servo_angle_received(float angle)
{
    // 设置舵机角度
    // servo_set_angle(angle);
}

/**
 * @brief 传感器数据回调函数
 * 
 * 当M701传感器读取到新数据时调用
 * 
 * @param data 传感器数据指针
 */
static void on_sensor_data_received(const m701_sensor_data_t *data)
{
    // 将数据转换为JSON
    char json_buf[128];
    int len = m701_sensor_to_json(data, json_buf, sizeof(json_buf));
    
    if (len > 0) {
        // 通过BLE发送
        ble_service_notify_sensor_data(json_buf, len);
        
        // 通过MQTT发布（格式：device/{sn}/data，数据中包含SN）
        if (mqtt_client_is_connected()) {
            // 在JSON中添加SN字段
            char json_with_sn[160];
            // 解析现有JSON并添加sn字段
            // json_buf格式: {"co2":609,"hcho":20,...}
            // 需要跳过开头的{，排除结尾的}，然后添加sn字段
            int sn_len = snprintf(json_with_sn, sizeof(json_with_sn),
                "{\"sn\":\"%s\",%.*s}", SENSOR_SN, len - 2, json_buf + 1);
            if (sn_len > 0 && sn_len < sizeof(json_with_sn)) {
                esp_err_t ret = mqtt_client_publish("data", (uint8_t*)json_with_sn, sn_len, 1);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Sensor data published to MQTT: %.*s", sn_len, json_with_sn);
                } else {
                    ESP_LOGW(TAG, "Failed to publish sensor data to MQTT");
                }
            }
        } else {
            ESP_LOGW(TAG, "MQTT not connected, sensor data not published");
        }
    }
}

/**
 * @brief WiFi连接状态回调
 */
static void on_wifi_status(bool connected, const char *ip_addr)
{
    if (connected) {
        ESP_LOGI(TAG, "WiFi connected, IP: %s", ip_addr);
        
        // WiFi连接成功后，从系统配置读取并保存到我们的NVS（确保配置持久化）
        // 这样即使是通过系统自动连接的，也能保存到我们的NVS中
        wifi_config_t wifi_config = {0};
        esp_err_t ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
        if (ret == ESP_OK && strlen((char*)wifi_config.sta.ssid) > 0) {
            // 检查是否已经保存到我们的NVS中
            char saved_ssid[64] = {0};
            char saved_password[64] = {0};
            if (load_wifi_config_from_nvs(saved_ssid, sizeof(saved_ssid), 
                                          saved_password, sizeof(saved_password)) != ESP_OK ||
                strcmp(saved_ssid, (char*)wifi_config.sta.ssid) != 0) {
                // 配置不存在或不匹配，保存当前配置
                save_wifi_config_to_nvs((char*)wifi_config.sta.ssid, 
                                        (char*)wifi_config.sta.password);
            }
        }
        
        // WiFi连接成功后，如果MQTT已配置则自动连接
        // 通过队列发送，避免在回调中执行耗时操作
        if (s_mqtt_config_queue) {
            // 尝试从NVS加载MQTT配置
            char config_json[512] = {0};
            ret = load_mqtt_config_from_nvs(config_json, sizeof(config_json));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Auto-connecting MQTT with saved config");
                // 分配内存并发送到队列
                char *config_copy = malloc(strlen(config_json) + 1);
                if (config_copy) {
                    strcpy(config_copy, config_json);
                    if (xQueueSend(s_mqtt_config_queue, &config_copy, pdMS_TO_TICKS(1000)) != pdTRUE) {
                        ESP_LOGE(TAG, "Failed to send MQTT config to queue");
                        free(config_copy);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for MQTT config");
                }
            } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(TAG, "No saved MQTT config found in NVS, waiting for BLE configuration");
            } else {
                ESP_LOGW(TAG, "Failed to load MQTT config from NVS: %s", esp_err_to_name(ret));
            }
        }
    } else {
        ESP_LOGI(TAG, "WiFi disconnected");
    }
}

/**
 * @brief 发送MQTT配置消息
 * 
 * 在MQTT连接成功后，发送设备配置信息到device/{sn}/config主题
 * 注意：配置消息只发送一次，避免重复发送
 */
static void send_mqtt_config_message(void)
{
    if (!mqtt_client_is_connected()) {
        ESP_LOGW(TAG, "MQTT not connected, cannot send config message");
        return;
    }
    
    // 如果已经发送过配置消息，则跳过
    if (s_config_message_sent) {
        ESP_LOGI(TAG, "MQTT config message already sent, skipping");
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(TAG, "Failed to create JSON object for config message");
        return;
    }
    
    // sn: 设备序列号（必填）
    cJSON_AddStringToObject(json, "sn", SENSOR_SN);
    
    // ip: 设备IP地址（可选）
    char ip_buf[16] = {0};
    const char *ip = wifi_manager_get_ip(ip_buf, sizeof(ip_buf));
    if (ip && strlen(ip) > 0) {
        cJSON_AddStringToObject(json, "ip", ip);
    }
    
    // mac: 设备MAC地址（可选）
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON_AddStringToObject(json, "mac", mac_str);
    }
    
    // ble_name: 蓝牙名称（可选）
    cJSON_AddStringToObject(json, "ble_name", BLE_DEVICE_NAME);
    
    // wifi_name: WiFi SSID（可选）
    char wifi_ssid[64] = {0};
    char wifi_password[64] = {0};
    if (load_wifi_config_from_nvs(wifi_ssid, sizeof(wifi_ssid), 
                                   wifi_password, sizeof(wifi_password)) == ESP_OK) {
        if (strlen(wifi_ssid) > 0) {
            cJSON_AddStringToObject(json, "wifi_name", wifi_ssid);
        }
    }
    
    // wifi_password: WiFi密码（可选）
    if (strlen(wifi_password) > 0) {
        cJSON_AddStringToObject(json, "wifi_password", wifi_password);
    }
    
    // mqtt_broker: MQTT服务器地址（可选）
    // mqtt_port: MQTT端口（可选）
    char mqtt_config_json[512] = {0};
    if (load_mqtt_config_from_nvs(mqtt_config_json, sizeof(mqtt_config_json)) == ESP_OK) {
        cJSON *mqtt_json = cJSON_Parse(mqtt_config_json);
        if (mqtt_json) {
            cJSON *broker = cJSON_GetObjectItem(mqtt_json, "broker");
            cJSON *port = cJSON_GetObjectItem(mqtt_json, "port");
            if (broker && cJSON_IsString(broker)) {
                cJSON_AddStringToObject(json, "mqtt_broker", broker->valuestring);
            }
            if (port && cJSON_IsNumber(port)) {
                cJSON_AddNumberToObject(json, "mqtt_port", port->valueint);
            } else {
                cJSON_AddNumberToObject(json, "mqtt_port", 443);  // 默认端口
            }
            cJSON_Delete(mqtt_json);
        }
    }
    
    // firmware: 固件版本（可选）
    char firmware_version[32];
    snprintf(firmware_version, sizeof(firmware_version), "v%d.%d.%d", 
             ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
    cJSON_AddStringToObject(json, "firmware", firmware_version);
    
    // 转换为JSON字符串
    char *json_str = cJSON_Print(json);
    if (json_str) {
        // 发布到device/{sn}/config主题
        char topic[128];
        snprintf(topic, sizeof(topic), "device/%s/config", SENSOR_SN);
        
        esp_err_t ret = mqtt_client_publish("config", (uint8_t*)json_str, strlen(json_str), 1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "MQTT config message sent to topic: device/%s/config", SENSOR_SN);
            ESP_LOGI(TAG, "Config message: %s", json_str);
            s_config_message_sent = true;  // 标记已发送
        } else {
            ESP_LOGE(TAG, "Failed to publish MQTT config message");
        }
        
        free(json_str);
    } else {
        ESP_LOGE(TAG, "Failed to convert config JSON to string");
    }
    
    cJSON_Delete(json);
}

/**
 * @brief MQTT连接状态回调
 */
static void on_mqtt_status(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "MQTT connected");
        // MQTT连接成功后，延迟一下确保连接稳定，然后发送配置消息
        // 注意：配置消息只发送一次，通过s_config_message_sent标志控制
        vTaskDelay(pdMS_TO_TICKS(1000));
        send_mqtt_config_message();
    } else {
        ESP_LOGI(TAG, "MQTT disconnected");
        // 断开连接时不清除标志，因为重连后也不需要再次发送配置消息
    }
}

/**
 * @brief MQTT消息接收回调
 */
static void on_mqtt_message(const char *topic, const uint8_t *data, int len)
{
    char msg_buf[256] = {0};
    int copy_len = (len < sizeof(msg_buf) - 1) ? len : sizeof(msg_buf) - 1;
    memcpy(msg_buf, data, copy_len);
    msg_buf[copy_len] = '\0';
    
    ESP_LOGI(TAG, "MQTT message received");
    
    // 解析主题（格式：device/{sn}/control
    char sensor_query_pattern[128];
    char sensor_control_pattern[128];
    
    snprintf(sensor_query_pattern, sizeof(sensor_query_pattern), "/%s/query", SENSOR_SN);
    snprintf(sensor_control_pattern, sizeof(sensor_control_pattern), "device/%s/control", SENSOR_SN);
    
    if (strstr(topic, sensor_control_pattern) != NULL) {
        // M701传感器控制（预留，当前传感器为只读）
        // 可以用于设置采样间隔、校准等（如果传感器支持）
        ESP_LOGI(TAG, "Sensor control command received");
    } else if (strstr(topic, sensor_query_pattern) != NULL) {
        // 查询传感器数据 - 匹配传感器SN
        m701_sensor_data_t sensor_data;
        if (m701_sensor_get_data(&sensor_data) == ESP_OK) {
            char json_buf[128];
            int json_len = m701_sensor_to_json(&sensor_data, json_buf, sizeof(json_buf));
            if (json_len > 0) {
                // 在JSON中添加SN字段（格式：device/{sn}/data，数据中包含SN）
                char json_with_sn[160];
                // json_buf格式: {"co2":609,"hcho":20,...}
                // 需要跳过开头的{，排除结尾的}，然后添加sn字段
                int sn_len = snprintf(json_with_sn, sizeof(json_with_sn),
                    "{\"sn\":\"%s\",%.*s}", SENSOR_SN, json_len - 2, json_buf + 1);
                if (sn_len > 0 && sn_len < sizeof(json_with_sn)) {
                    mqtt_client_publish("data", (uint8_t*)json_with_sn, sn_len, 1);
                }
            }
        }
    }
}

/**
 * @brief 处理 WiFi 配置（在独立任务中执行，避免在 BLE 回调中执行导致 BTC_TASK 栈溢出）
 */
static void process_wifi_config(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "WiFi config: SSID=%s", ssid);
    save_wifi_config_to_nvs(ssid, password);
    wifi_manager_connect(ssid, password);
}

/**
 * @brief WiFi 配置回调（从 BLE 回调中调用，只投递到队列，不执行耗时操作）
 */
static void on_wifi_config(const char *ssid, const char *password)
{
    if (!s_wifi_config_queue) {
        return;
    }
    wifi_config_msg_t *msg = malloc(sizeof(wifi_config_msg_t));
    if (!msg) {
        ESP_LOGE(TAG, "Failed to allocate WiFi config message");
        return;
    }
    strncpy(msg->ssid, ssid, WIFI_SSID_MAX - 1);
    msg->ssid[WIFI_SSID_MAX - 1] = '\0';
    strncpy(msg->password, password ? password : "", WIFI_PASSWORD_MAX - 1);
    msg->password[WIFI_PASSWORD_MAX - 1] = '\0';
    if (xQueueSend(s_wifi_config_queue, &msg, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send WiFi config to queue");
        free(msg);
    }
}

/**
 * @brief MQTT配置回调（从BLE回调中调用，只发送到队列）
 */
static void on_mqtt_config(const char *config_json)
{
    // 只记录日志，不执行耗时操作
    ESP_LOGI(TAG, "MQTT config received: %s", config_json);
    
    // 将配置发送到队列，由主任务处理
    if (s_mqtt_config_queue) {
        // 分配内存存储配置字符串
        char *config_copy = malloc(strlen(config_json) + 1);
        if (config_copy) {
            strcpy(config_copy, config_json);
            if (xQueueSend(s_mqtt_config_queue, &config_copy, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send MQTT config to queue");
                free(config_copy);
            }
        } else {
            ESP_LOGE(TAG, "Failed to allocate memory for MQTT config");
        }
    }
}

/**
 * @brief 保存MQTT配置到NVS
 */
static esp_err_t save_mqtt_config_to_nvs(const char *config_json)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_str(nvs_handle, NVS_KEY_MQTT_CONFIG, config_json);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save MQTT config to NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MQTT config saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief 从NVS加载MQTT配置
 */
static esp_err_t load_mqtt_config_from_nvs(char *config_json, size_t max_len)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to open NVS namespace '%s' (may not exist): %s", NVS_NAMESPACE, esp_err_to_name(ret));
        return ret;
    }
    
    size_t required_size = max_len;
    ret = nvs_get_str(nvs_handle, NVS_KEY_MQTT_CONFIG, config_json, &required_size);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MQTT config loaded from NVS: %s", config_json);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No MQTT config found in NVS (key: %s)", NVS_KEY_MQTT_CONFIG);
    } else {
        ESP_LOGE(TAG, "Failed to load MQTT config from NVS: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief 保存WiFi配置到NVS
 */
static esp_err_t save_wifi_config_to_nvs(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_WIFI, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 保存SSID
    ret = nvs_set_str(nvs_handle, "ssid", ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi SSID to NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // 保存密码（如果提供）
    if (password && strlen(password) > 0) {
        ret = nvs_set_str(nvs_handle, "password", password);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save WiFi password to NVS: %s", esp_err_to_name(ret));
            nvs_close(nvs_handle);
            return ret;
        }
    } else {
        // 如果没有密码，删除之前保存的密码
        nvs_erase_key(nvs_handle, "password");
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi config saved to NVS (SSID: %s)", ssid);
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief 从NVS加载WiFi配置
 */
static esp_err_t load_wifi_config_from_nvs(char *ssid, size_t ssid_size, char *password, size_t password_size)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE_WIFI, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to open NVS namespace '%s' (may not exist): %s", NVS_NAMESPACE_WIFI, esp_err_to_name(ret));
        return ret;
    }
    
    // 加载SSID
    size_t required_size = ssid_size;
    ret = nvs_get_str(nvs_handle, "ssid", ssid, &required_size);
    if (ret != ESP_OK) {
        nvs_close(nvs_handle);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No WiFi SSID found in NVS");
        } else {
            ESP_LOGE(TAG, "Failed to load WiFi SSID from NVS: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    // 加载密码（可选）
    if (password && password_size > 0) {
        required_size = password_size;
        ret = nvs_get_str(nvs_handle, "password", password, &required_size);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            // 密码不存在（可能是开放网络），清空密码
            password[0] = '\0';
            ESP_LOGI(TAG, "No WiFi password found in NVS (open network)");
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load WiFi password from NVS: %s", esp_err_to_name(ret));
            password[0] = '\0';
        }
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "WiFi config loaded from NVS (SSID: %s)", ssid);
    
    return ESP_OK;
}

/**
 * @brief 处理MQTT配置（在主任务中执行）
 */
static void process_mqtt_config(const char *config_json)
{
    ESP_LOGI(TAG, "Processing MQTT config: %s", config_json);
    
    // 解析JSON配置
    cJSON *json = cJSON_Parse(config_json);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON config");
        return;
    }
    
    mqtt_config_t mqtt_cfg = {0};
    
    // 只解析broker和port，其他字段使用默认值
    cJSON *item = cJSON_GetObjectItem(json, "broker");
    if (item && cJSON_IsString(item) && strlen(item->valuestring) > 0) {
        strncpy(mqtt_cfg.broker, item->valuestring, sizeof(mqtt_cfg.broker) - 1);
    } else {
        strncpy(mqtt_cfg.broker, "212.129.243.155", sizeof(mqtt_cfg.broker) - 1);  // 默认 host
    }
    
    item = cJSON_GetObjectItem(json, "port");
    if (item && cJSON_IsNumber(item)) {
        mqtt_cfg.port = item->valueint;
    } else {
        mqtt_cfg.port = 443;  // 默认端口 (wss)
    }
    
    // username、password、client_id 不需要配置
    // - username/password: 留空表示不使用认证
    // - client_id: 自动生成 device_{SN} 格式
    
    cJSON_Delete(json);
    
    // 配置并连接MQTT
    if (strlen(mqtt_cfg.broker) > 0) {
        // 保存配置到NVS
        save_mqtt_config_to_nvs(config_json);
        
        // 设置MQTT配置
        mqtt_client_set_config(&mqtt_cfg);
        
        // 重置配置消息发送标志，确保新配置连接成功后会发送配置消息
        s_config_message_sent = false;
        
        // 如果WiFi已连接，立即连接MQTT
        if (wifi_manager_is_connected()) {
            mqtt_client_connect();
        } else {
            ESP_LOGW(TAG, "WiFi not connected, MQTT will connect after WiFi ready");
        }
    }
}

/**
 * @brief WiFi 配置处理任务
 */
static void wifi_config_task(void *pvParameters)
{
    wifi_config_msg_t *msg = NULL;
    
    while (1) {
        if (xQueueReceive(s_wifi_config_queue, &msg, portMAX_DELAY) == pdTRUE && msg) {
            process_wifi_config(msg->ssid, msg->password[0] ? msg->password : NULL);
            free(msg);
        }
    }
}

/**
 * @brief MQTT配置处理任务
 */
static void mqtt_config_task(void *pvParameters)
{
    char *config_json = NULL;
    
    while (1) {
        // 从队列接收配置
        if (xQueueReceive(s_mqtt_config_queue, &config_json, portMAX_DELAY) == pdTRUE) {
            if (config_json) {
                process_mqtt_config(config_json);
                free(config_json);
                config_json = NULL;
            }
        }
    }
}

/**
 * @brief 初始化NVS闪存
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS initialized");
    } else {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief 应用程序主函数
 */
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "ESP32-C3 BLE LED Controller Starting...");

    // 1. 初始化NVS（蓝牙需要）
    ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed");
        return;
    }

    // 2. 初始化WS2812 LED驱动
    // ret = ws2812_init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "WS2812 init failed");
    //     return;
    // }

    // 3. 初始化舵机驱动
    // ret = servo_init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Servo init failed");
    //     return;
    // }

    // 4. 初始化WiFi管理器
    ret = wifi_manager_init(on_wifi_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi manager init failed");
        return;
    }

    // 5. 初始化MQTT客户端（传入传感器SN）
    ret = mqtt_client_init(SENSOR_SN, on_mqtt_message, on_mqtt_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MQTT client init failed");
        return;
    }

    // 6. 初始化M701传感器
    ret = m701_sensor_init(on_sensor_data_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "M701 sensor init failed");
        return;
    }

    // 7. 创建 WiFi 配置队列与处理任务（必须在 BLE 注册回调前创建）
    s_wifi_config_queue = xQueueCreate(WIFI_CONFIG_QUEUE_SIZE, sizeof(wifi_config_msg_t *));
    if (!s_wifi_config_queue) {
        ESP_LOGE(TAG, "Failed to create WiFi config queue");
        return;
    }
    xTaskCreate(wifi_config_task, "wifi_config_task", 4096, NULL, 5, NULL);

    // 8. 创建MQTT配置队列
    s_mqtt_config_queue = xQueueCreate(MQTT_CONFIG_QUEUE_SIZE, sizeof(char*));
    if (!s_mqtt_config_queue) {
        ESP_LOGE(TAG, "Failed to create MQTT config queue");
        return;
    }

    // 9. 创建MQTT配置处理任务
    xTaskCreate(mqtt_config_task, "mqtt_config_task", 4096, NULL, 5, NULL);

    // 10. 初始化BLE服务
    ret = ble_service_init(on_led_data_received, on_servo_angle_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    // 11. 注册WiFi和MQTT配置回调
    ble_service_set_wifi_config_callback(on_wifi_config);
    ble_service_set_mqtt_config_callback(on_mqtt_config);

    // 12. 如果WiFi未连接，尝试从NVS加载配置并自动连接
    if (!wifi_manager_is_connected()) {
        ESP_LOGI(TAG, "WiFi not connected, checking for saved WiFi config...");
        char saved_ssid[64] = {0};
        char saved_password[64] = {0};
        if (load_wifi_config_from_nvs(saved_ssid, sizeof(saved_ssid), 
                                      saved_password, sizeof(saved_password)) == ESP_OK) {
            ESP_LOGI(TAG, "Auto-connecting WiFi with saved config (SSID: %s)", saved_ssid);
            // 延迟一下确保系统稳定
            vTaskDelay(pdMS_TO_TICKS(2000));
            // 自动连接WiFi
            wifi_manager_connect(saved_ssid, saved_password[0] ? saved_password : NULL);
        } else {
            ESP_LOGI(TAG, "No saved WiFi config found, waiting for BLE configuration");
        }
    }

    // 13. 如果WiFi已连接，尝试自动加载并连接MQTT（通过队列）
    // 注意：如果MQTT已经连接（可能在on_wifi_status回调中已连接），则跳过
    if (wifi_manager_is_connected() && !mqtt_client_is_connected()) {
        ESP_LOGI(TAG, "WiFi already connected, checking for saved MQTT config...");
        char config_json[512] = {0};
        if (load_mqtt_config_from_nvs(config_json, sizeof(config_json)) == ESP_OK) {
            ESP_LOGI(TAG, "Auto-connecting MQTT with saved config");
            // 延迟一下确保系统稳定
            vTaskDelay(pdMS_TO_TICKS(2000));
            // 通过队列发送，由mqtt_config_task处理
            char *config_copy = malloc(strlen(config_json) + 1);
            if (config_copy) {
                strcpy(config_copy, config_json);
                if (xQueueSend(s_mqtt_config_queue, &config_copy, pdMS_TO_TICKS(1000)) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send MQTT config to queue");
                    free(config_copy);
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate memory for MQTT config");
            }
        } else {
            ESP_LOGI(TAG, "No saved MQTT config found, waiting for BLE configuration");
        }
    } else if (wifi_manager_is_connected() && mqtt_client_is_connected()) {
        ESP_LOGI(TAG, "WiFi and MQTT already connected, skipping auto-connect");
    }

    // 14. 输出系统就绪信息
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "System ready!");
    
    // 输出BLE名称
    ESP_LOGI(TAG, "BLE Configuration:");
    ESP_LOGI(TAG, "  Device Name: %s", BLE_DEVICE_NAME);
    
    // 输出WiFi配置
    ESP_LOGI(TAG, "WiFi Configuration:");
    char saved_ssid[64] = {0};
    char saved_password[64] = {0};
    if (load_wifi_config_from_nvs(saved_ssid, sizeof(saved_ssid), 
                                  saved_password, sizeof(saved_password)) == ESP_OK) {
        if (saved_password[0] != '\0') {
            ESP_LOGI(TAG, "  SSID: %s", saved_ssid);
            ESP_LOGI(TAG, "  Password: %s", saved_password);
        } else {
            ESP_LOGI(TAG, "  SSID: %s", saved_ssid);
            ESP_LOGI(TAG, "  Password: (open network)");
        }
    } else {
        ESP_LOGI(TAG, "  Status: Not configured");
    }
    
    // 输出MQTT配置
    ESP_LOGI(TAG, "MQTT Configuration:");
    char mqtt_config_json[512] = {0};
    if (load_mqtt_config_from_nvs(mqtt_config_json, sizeof(mqtt_config_json)) == ESP_OK) {
        // 解析JSON以提取broker和port
        cJSON *json = cJSON_Parse(mqtt_config_json);
        if (json) {
            cJSON *broker = cJSON_GetObjectItem(json, "broker");
            cJSON *port = cJSON_GetObjectItem(json, "port");
            if (broker && cJSON_IsString(broker)) {
                ESP_LOGI(TAG, "  Broker: %s", broker->valuestring);
            }
            if (port && cJSON_IsNumber(port)) {
                ESP_LOGI(TAG, "  Port: %d", port->valueint);
            } else {
                ESP_LOGI(TAG, "  Port: 443 (default)");
            }
            cJSON_Delete(json);
        } else {
            ESP_LOGI(TAG, "  Config: %s", mqtt_config_json);
        }
    } else {
        ESP_LOGI(TAG, "  Status: Not configured");
    }
    
    ESP_LOGI(TAG, "Device SN:");
    ESP_LOGI(TAG, "  Sensor SN: %s", SENSOR_SN);
    ESP_LOGI(TAG, "Hardware:");
    ESP_LOGI(TAG, "  Servo: GPIO2");
    ESP_LOGI(TAG, "  M701:  GPIO4");
    ESP_LOGI(TAG, "Use BLE to configure WiFi and MQTT");
    ESP_LOGI(TAG, "WiFi and MQTT configs will be auto-loaded on restart");
    ESP_LOGI(TAG, "========================================");
}
