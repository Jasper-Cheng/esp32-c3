/*
 * BLE服务模块 - 头文件
 * 
 * 功能：提供蓝牙BLE GATT服务，用于接收LED控制数据
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>
#include "esp_err.h"

/* BLE配置参数 */
#define BLE_DEVICE_NAME         "ESP-LED"
#define BLE_SERVICE_UUID        0x00FF
#define BLE_CHAR_UUID           0xFF01
 
/**
 * @brief LED数据接收回调函数类型
 * 
 * @param led_data 解析后的60个LED颜色索引
 */
typedef void (*ble_led_data_callback_t)(uint8_t *led_data);

/**
 * @brief 初始化BLE服务
 * 
 * @param callback LED数据接收回调函数
 * @return 
 *     - ESP_OK: 成功
 *     - 其他: 失败
 */
esp_err_t ble_service_init(ble_led_data_callback_t callback);

/**
 * @brief 获取当前LED数据
 * 
 * @return 当前的LED颜色索引数组
 */
uint8_t* ble_service_get_led_data(void);

#endif // BLE_SERVICE_H
