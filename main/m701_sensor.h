/*
 * M701SC 7合一空气质量传感器驱动 - 头文件
 * 
 * 功能：通过UART读取M701SC传感器数据
 * 
 * 测量项：CO2, HCHO, TVOC, PM2.5, PM10, 温度, 湿度
 * 通信协议：UART 9600 bps, 8N1
 * 数据帧：17字节，帧头0x3C
 */

#ifndef M701_SENSOR_H
#define M701_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* 配置参数 */
#define M701_UART_NUM           UART_NUM_1      // 使用UART1
#define M701_UART_RX_PIN        GPIO_NUM_3      // RX引脚
#define M701_UART_BAUD_RATE     9600            // 波特率
#define M701_FRAME_SIZE         17              // 数据帧长度
#define M701_FRAME_HEADER       0x3C            // 帧头

/**
 * @brief M701传感器数据结构
 */
typedef struct {
    uint16_t co2;           // CO2浓度 (ppm)
    uint16_t hcho;          // 甲醛浓度 (µg/m³)
    uint16_t tvoc;          // TVOC浓度 (µg/m³)
    uint16_t pm25;          // PM2.5浓度 (µg/m³)
    uint16_t pm10;          // PM10浓度 (µg/m³)
    float temperature;      // 温度 (°C)
    float humidity;         // 湿度 (%RH)
    bool valid;             // 数据是否有效
} m701_sensor_data_t;

/**
 * @brief 传感器数据回调函数类型
 * 
 * @param data 传感器数据指针
 */
typedef void (*m701_data_callback_t)(const m701_sensor_data_t *data);

/**
 * @brief 初始化M701传感器
 * 
 * 配置UART并启动数据读取任务
 * 
 * @param callback 数据回调函数（收到有效数据时调用）
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t m701_sensor_init(m701_data_callback_t callback);

/**
 * @brief 获取最新的传感器数据
 * 
 * @param data 输出数据结构指针
 * @return 
 *     - ESP_OK: 成功获取有效数据
 *     - ESP_ERR_INVALID_STATE: 数据无效或未初始化
 */
esp_err_t m701_sensor_get_data(m701_sensor_data_t *data);

/**
 * @brief 将传感器数据格式化为JSON字符串
 * 
 * @param data 传感器数据指针
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @return 写入的字符数
 */
int m701_sensor_to_json(const m701_sensor_data_t *data, char *buf, size_t buf_size);

#endif // M701_SENSOR_H

