/*
 * MQTT客户端 - 头文件
 * 
 * 功能：MQTT连接、发布、订阅
 */

#ifndef MQTT_WRAPPER_H
#define MQTT_WRAPPER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief MQTT消息接收回调函数类型
 * 
 * @param topic 主题
 * @param data 消息数据
 * @param len 数据长度
 */
typedef void (*mqtt_message_callback_t)(const char *topic, const uint8_t *data, int len);

/**
 * @brief MQTT连接状态回调函数类型
 * 
 * @param connected true=已连接, false=断开
 */
typedef void (*mqtt_status_callback_t)(bool connected);

/**
 * @brief MQTT配置结构
 */
typedef struct {
    char broker[128];      // MQTT服务器地址
    uint16_t port;         // 端口（默认1883）
    char username[64];     // 用户名（可选）
    char password[64];     // 密码（可选）
    char client_id[64];    // 客户端ID（可选，默认使用MAC地址）
    char prefix[64];       // 主题前缀（如 "jasper-c3"）
} mqtt_config_t;

/**
 * @brief 初始化MQTT客户端
 * 
 * @param callback 消息接收回调函数
 * @param status_callback 连接状态回调函数
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t mqtt_client_init(mqtt_message_callback_t callback, mqtt_status_callback_t status_callback);

/**
 * @brief 配置MQTT连接参数
 * 
 * @param config MQTT配置结构
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数无效
 */
esp_err_t mqtt_client_set_config(const mqtt_config_t *config);

/**
 * @brief 连接到MQTT服务器
 * 
 * @return 
 *     - ESP_OK: 开始连接
 *     - ESP_ERR_INVALID_STATE: 未配置或WiFi未连接
 */
esp_err_t mqtt_client_connect(void);

/**
 * @brief 断开MQTT连接
 * 
 * @return 
 *     - ESP_OK: 成功
 */
esp_err_t mqtt_client_disconnect(void);

/**
 * @brief 发布消息
 * 
 * @param topic 主题（相对路径，会自动添加前缀）
 * @param data 消息数据
 * @param len 数据长度
 * @param qos QoS等级（0或1）
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t mqtt_client_publish(const char *topic, const uint8_t *data, int len, int qos);

/**
 * @brief 订阅主题
 * 
 * @param topic 主题（相对路径，会自动添加前缀）
 * @param qos QoS等级（0或1）
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t mqtt_client_subscribe(const char *topic, int qos);

/**
 * @brief 获取MQTT连接状态
 * 
 * @return true=已连接, false=未连接
 */
bool mqtt_client_is_connected(void);

/**
 * @brief 获取完整主题路径（前缀+相对路径）
 * 
 * @param relative_topic 相对主题
 * @param full_topic 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 完整主题路径
 */
const char* mqtt_client_get_full_topic(const char *relative_topic, char *full_topic, size_t buf_size);

#endif // MQTT_WRAPPER_H

