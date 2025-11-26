/*
 * ESP32-C3 BLE LED控制器 - 主程序
 * 
 * 功能：
 * 1. 提供蓝牙BLE服务（可连接，支持读写和通知）
 * 2. 接收60字节数据，每字节控制一个LED（0=灭，1-7为不同颜色）
 * 3. 通过GPIO1输出WS2812 LED控制信号
 * 
 * 架构：
 * - BLE服务层：ble_service.c/h - 处理蓝牙通信
 * - LED驱动层：ws2812_driver.c/h - 控制WS2812 LED
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
    ws2812_update_leds(led_data);
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
    ret = ws2812_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 init failed");
        return;
    }

    // 3. 初始化BLE服务
    ret = ble_service_init(on_led_data_received);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    ESP_LOGI(TAG, "System ready!");
}
