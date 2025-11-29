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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>

#include "ble_service.h"
#include "ws2812_driver.h"
#include "servo_driver.h"
#include "m701_sensor.h"
#include "wifi_manager.h"
#include "mqtt_wrapper.h"
#include "cJSON.h"

/* 日志标签 */
static const char* TAG = "MAIN";

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
        
        // 通过MQTT发布
        if (mqtt_client_is_connected()) {
            mqtt_client_publish("sensor/data", (uint8_t*)json_buf, len, 1);
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
        // WiFi连接成功后，如果MQTT已配置则自动连接
        // MQTT连接在配置回调中处理
    } else {
        ESP_LOGI(TAG, "WiFi disconnected");
    }
}

/**
 * @brief MQTT连接状态回调
 */
static void on_mqtt_status(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "MQTT connected");
    } else {
        ESP_LOGI(TAG, "MQTT disconnected");
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
    
    ESP_LOGI(TAG, "MQTT message: topic=%s, data=%.*s", topic, len, msg_buf);
    
    // 解析主题，判断是LED控制还是舵机控制
    if (strstr(topic, "/control/led") != NULL) {
        // LED控制
        uint8_t led_data[60] = {0};
        int count = 0;
        for (int i = 0; i < len && count < 60; i++) {
            char c = msg_buf[i];
            if (c >= '0' && c <= '7') {
                led_data[count++] = (uint8_t)(c - '0');
            }
        }
        if (count > 0) {
            on_led_data_received(led_data);
        }
    } else if (strstr(topic, "/control/servo") != NULL) {
        // 舵机控制
        float angle = 0.0f;
        if (sscanf(msg_buf, "%f", &angle) == 1 && angle >= 0.0f && angle <= 270.0f) {
            on_servo_angle_received(angle);
        }
    }
}

/**
 * @brief WiFi配置回调
 */
static void on_wifi_config(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "WiFi config: SSID=%s", ssid);
    wifi_manager_connect(ssid, password);
}

/**
 * @brief MQTT配置回调
 */
static void on_mqtt_config(const char *config_json)
{
    ESP_LOGI(TAG, "MQTT config: %s", config_json);
    
    // 解析JSON配置
    cJSON *json = cJSON_Parse(config_json);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON config");
        return;
    }
    
    mqtt_config_t mqtt_cfg = {0};
    
    cJSON *item = cJSON_GetObjectItem(json, "broker");
    if (item && cJSON_IsString(item)) {
        strncpy(mqtt_cfg.broker, item->valuestring, sizeof(mqtt_cfg.broker) - 1);
    }
    
    item = cJSON_GetObjectItem(json, "port");
    if (item && cJSON_IsNumber(item)) {
        mqtt_cfg.port = item->valueint;
    } else {
        mqtt_cfg.port = 1883;
    }
    
    item = cJSON_GetObjectItem(json, "username");
    if (item && cJSON_IsString(item)) {
        strncpy(mqtt_cfg.username, item->valuestring, sizeof(mqtt_cfg.username) - 1);
    }
    
    item = cJSON_GetObjectItem(json, "password");
    if (item && cJSON_IsString(item)) {
        strncpy(mqtt_cfg.password, item->valuestring, sizeof(mqtt_cfg.password) - 1);
    }
    
    item = cJSON_GetObjectItem(json, "prefix");
    if (item && cJSON_IsString(item)) {
        strncpy(mqtt_cfg.prefix, item->valuestring, sizeof(mqtt_cfg.prefix) - 1);
    } else {
        strcpy(mqtt_cfg.prefix, "jasper-c3");
    }
    
    cJSON_Delete(json);
    
    // 配置并连接MQTT
    if (strlen(mqtt_cfg.broker) > 0) {
        mqtt_client_set_config(&mqtt_cfg);
        if (wifi_manager_is_connected()) {
            mqtt_client_connect();
        } else {
            ESP_LOGW(TAG, "WiFi not connected, MQTT will connect after WiFi ready");
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

    // 5. 初始化MQTT客户端
    ret = mqtt_client_init(on_mqtt_message, on_mqtt_status);
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

    // 7. 初始化BLE服务
    ret = ble_service_init(on_led_data_received, on_servo_angle_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    // 8. 注册WiFi和MQTT配置回调
    ble_service_set_wifi_config_callback(on_wifi_config);
    ble_service_set_mqtt_config_callback(on_mqtt_config);

    ESP_LOGI(TAG, "System ready! Servo:GPIO2, M701:GPIO3");
    ESP_LOGI(TAG, "Use BLE to configure WiFi and MQTT");
}
