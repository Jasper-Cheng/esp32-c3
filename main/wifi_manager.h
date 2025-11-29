/*
 * WiFi管理器 - 头文件
 * 
 * 功能：管理WiFi连接，支持通过BLE配置WiFi参数
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief WiFi连接状态回调函数类型
 * 
 * @param connected true=已连接, false=断开
 * @param ip_addr IP地址字符串（连接成功时有效）
 */
typedef void (*wifi_status_callback_t)(bool connected, const char *ip_addr);

/**
 * @brief 初始化WiFi管理器
 * 
 * @param callback 连接状态回调函数
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t wifi_manager_init(wifi_status_callback_t callback);

/**
 * @brief 连接WiFi
 * 
 * @param ssid WiFi SSID
 * @param password WiFi密码
 * @return 
 *     - ESP_OK: 开始连接
 *     - ESP_ERR_INVALID_ARG: 参数无效
 */
esp_err_t wifi_manager_connect(const char *ssid, const char *password);

/**
 * @brief 断开WiFi连接
 * 
 * @return 
 *     - ESP_OK: 成功
 */
esp_err_t wifi_manager_disconnect(void);

/**
 * @brief 获取WiFi连接状态
 * 
 * @return true=已连接, false=未连接
 */
bool wifi_manager_is_connected(void);

/**
 * @brief 获取当前IP地址
 * 
 * @param ip_buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return IP地址字符串，未连接时返回NULL
 */
const char* wifi_manager_get_ip(char *ip_buf, size_t buf_size);

#endif // WIFI_MANAGER_H

