/*
 * BLE服务模块 - 头文件
 * 
 * 功能：提供蓝牙BLE GATT服务，用于接收LED控制数据和舵机角度控制
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>
#include "esp_err.h"

/* BLE配置参数 */
#define BLE_DEVICE_NAME         "Jasper-C3"
#define BLE_SERVICE_UUID        0x00FF
#define BLE_CHAR_UUID           0xFF01      // LED控制特征值
#define BLE_SERVO_CHAR_UUID     0xFF02      // 舵机控制特征值
#define BLE_SENSOR_CHAR_UUID    0xFF03      // 传感器数据特征值
#define BLE_WIFI_CONFIG_UUID    0xFF04      // WiFi配置特征值
#define BLE_MQTT_CONFIG_UUID    0xFF05      // MQTT配置特征值
 
/**
 * @brief LED数据接收回调函数类型
 * 
 * @param led_data 解析后的60个LED颜色索引
 */
typedef void (*ble_led_data_callback_t)(uint8_t *led_data);

/**
 * @brief 舵机角度接收回调函数类型
 * 
 * @param angle 舵机目标角度 (0.0 ~ 270.0度)
 */
typedef void (*ble_servo_callback_t)(float angle);

/**
 * @brief 初始化BLE服务
 * 
 * @param led_callback LED数据接收回调函数
 * @param servo_callback 舵机角度接收回调函数
 * @return 
 *     - ESP_OK: 成功
 *     - 其他: 失败
 */
esp_err_t ble_service_init(ble_led_data_callback_t led_callback, ble_servo_callback_t servo_callback);

/**
 * @brief 获取当前LED数据
 * 
 * @return 当前的LED颜色索引数组
 */
uint8_t* ble_service_get_led_data(void);

/**
 * @brief 发送传感器数据通知
 * 
 * 通过BLE向连接的客户端发送传感器数据
 * 
 * @param data JSON格式的传感器数据字符串
 * @param len 数据长度
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败（未连接等）
 */
esp_err_t ble_service_notify_sensor_data(const char *data, uint16_t len);

/**
 * @brief WiFi配置回调函数类型
 * 
 * @param ssid WiFi SSID
 * @param password WiFi密码
 */
typedef void (*ble_wifi_config_callback_t)(const char *ssid, const char *password);

/**
 * @brief MQTT配置回调函数类型
 * 
 * @param config_json MQTT配置JSON字符串
 */
typedef void (*ble_mqtt_config_callback_t)(const char *config_json);

/**
 * @brief 设置WiFi配置回调
 * 
 * @param callback WiFi配置回调函数
 */
void ble_service_set_wifi_config_callback(ble_wifi_config_callback_t callback);

/**
 * @brief 设置MQTT配置回调
 * 
 * @param callback MQTT配置回调函数
 */
void ble_service_set_mqtt_config_callback(ble_mqtt_config_callback_t callback);

#endif // BLE_SERVICE_H
