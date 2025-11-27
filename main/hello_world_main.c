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

#include "ble_service.h"
#include "ws2812_driver.h"
#include "servo_driver.h"
#include "m701_sensor.h"

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
    // 将数据转换为JSON并通过BLE发送
    char json_buf[128];
    int len = m701_sensor_to_json(data, json_buf, sizeof(json_buf));
    
    if (len > 0) {
        ble_service_notify_sensor_data(json_buf, len);
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

    // 4. 初始化M701传感器
    ret = m701_sensor_init(on_sensor_data_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "M701 sensor init failed");
        return;
    }

    // 5. 初始化BLE服务
    ret = ble_service_init(on_led_data_received, on_servo_angle_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    ESP_LOGI(TAG, "System ready! Servo:GPIO2, M701:GPIO3");
}
