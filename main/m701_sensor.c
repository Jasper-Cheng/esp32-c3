/*
 * M701SC 7合一空气质量传感器驱动 - 实现文件
 * 
 * 数据帧格式 (17字节):
 * B1:     0x3C (帧头)
 * B2:     0x02
 * B3-B4:  CO2 高/低字节
 * B5-B6:  HCHO 高/低字节
 * B7-B8:  TVOC 高/低字节
 * B9-B10: PM2.5 高/低字节
 * B11-B12: PM10 高/低字节
 * B13:    温度整数 (bit7=1表示负数)
 * B14:    温度小数
 * B15:    湿度整数
 * B16:    湿度小数
 * B17:    校验和 (B1~B16之和的低8位)
 */

#include "m701_sensor.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>

/* 日志标签 */
static const char* TAG = "M701";

/* UART缓冲区大小 */
#define UART_BUF_SIZE   256

/* 全局变量 */
static m701_sensor_data_t s_sensor_data = {0};
static m701_data_callback_t s_data_callback = NULL;
static bool s_initialized = false;
static QueueHandle_t s_uart_queue = NULL;

/**
 * @brief 验证校验和
 */
static bool verify_checksum(const uint8_t *frame)
{
    uint8_t sum = 0;
    for (int i = 0; i < M701_FRAME_SIZE - 1; i++) {
        sum += frame[i];
    }
    return (sum == frame[M701_FRAME_SIZE - 1]);
}

/**
 * @brief 解析数据帧
 */
static bool parse_frame(const uint8_t *frame, m701_sensor_data_t *data)
{
    // 验证帧头
    if (frame[0] != M701_FRAME_HEADER || frame[1] != 0x02) {
        ESP_LOGW(TAG, "Invalid frame header: 0x%02X 0x%02X", frame[0], frame[1]);
        return false;
    }
    
    // 验证校验和
    if (!verify_checksum(frame)) {
        ESP_LOGW(TAG, "Checksum verification failed");
        return false;
    }
    
    // 解析CO2 (B3-B4)
    data->co2 = (frame[2] << 8) | frame[3];
    
    // 解析HCHO (B5-B6)
    data->hcho = (frame[4] << 8) | frame[5];
    
    // 解析TVOC (B7-B8)
    data->tvoc = (frame[6] << 8) | frame[7];
    
    // 解析PM2.5 (B9-B10)
    data->pm25 = (frame[8] << 8) | frame[9];
    
    // 解析PM10 (B11-B12)
    data->pm10 = (frame[10] << 8) | frame[11];
    
    // 解析温度 (B13-B14)
    // B13 bit7=1表示负数
    int8_t temp_int = frame[12];
    uint8_t temp_dec = frame[13];
    if (temp_int & 0x80) {
        // 负数
        data->temperature = -((temp_int & 0x7F) + temp_dec / 100.0f);
    } else {
        data->temperature = temp_int + temp_dec / 100.0f;
    }
    
    // 解析湿度 (B15-B16)
    data->humidity = frame[14] + frame[15] / 100.0f;
    
    data->valid = true;
    
    return true;
}

/**
 * @brief UART读取任务
 */
static void m701_uart_task(void *arg)
{
    uint8_t rx_buf[UART_BUF_SIZE];
    uint8_t frame_buf[M701_FRAME_SIZE];
    size_t frame_idx = 0;
    bool in_frame = false;
    TickType_t last_byte_tick = 0;
    const TickType_t frame_timeout = pdMS_TO_TICKS(500);
    uart_event_t event;
    
    ESP_LOGI(TAG, "UART read task started");
    
    while (1) {
        if (s_uart_queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (xQueueReceive(s_uart_queue, &event, frame_timeout) != pdTRUE) {
            // 超时无事件，检查是否需要重置帧
            if (in_frame && (xTaskGetTickCount() - last_byte_tick) > frame_timeout) {
                in_frame = false;
                frame_idx = 0;
            }
            continue;
        }

        switch (event.type) {
        case UART_DATA: {
            size_t bytes_to_read = event.size;
            while (bytes_to_read > 0) {
                int chunk = uart_read_bytes(M701_UART_NUM, rx_buf,
                                            bytes_to_read > UART_BUF_SIZE ? UART_BUF_SIZE : bytes_to_read,
                                            pdMS_TO_TICKS(20));
                if (chunk <= 0) {
                    break;
                }
                bytes_to_read -= chunk;

                int offset = 0;
                while (offset < chunk) {
                    if (!in_frame) {
                        uint8_t *header = (uint8_t *)memchr(rx_buf + offset, M701_FRAME_HEADER, chunk - offset);
                        if (!header) {
                            break;
                        }
                        offset = header - rx_buf;
                        in_frame = true;
                        frame_idx = 0;
                    }

                    size_t bytes_needed = M701_FRAME_SIZE - frame_idx;
                    size_t remaining = (size_t)(chunk - offset);
                    size_t copy_len = bytes_needed < remaining ? bytes_needed : remaining;
                    memcpy(frame_buf + frame_idx, rx_buf + offset, copy_len);
                    frame_idx += copy_len;
                    offset += copy_len;
                    last_byte_tick = xTaskGetTickCount();

                    if (frame_idx == M701_FRAME_SIZE) {
                        m701_sensor_data_t temp_data;
                        if (parse_frame(frame_buf, &temp_data)) {
                            memcpy(&s_sensor_data, &temp_data, sizeof(m701_sensor_data_t));
                            ESP_LOGI(TAG, "CO2:%d HCHO:%d TVOC:%d PM2.5:%d PM10:%d T:%.1f H:%.1f",
                                     temp_data.co2, temp_data.hcho, temp_data.tvoc,
                                     temp_data.pm25, temp_data.pm10,
                                     temp_data.temperature, temp_data.humidity);
                            if (s_data_callback) {
                                s_data_callback(&temp_data);
                            }
                        }
                        in_frame = false;
                        frame_idx = 0;
                    }
                }
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "UART FIFO overflow or buffer full");
            uart_flush_input(M701_UART_NUM);
            xQueueReset(s_uart_queue);
            in_frame = false;
            frame_idx = 0;
            break;
        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "UART parity error");
            break;
        case UART_FRAME_ERR:
            ESP_LOGW(TAG, "UART frame error");
            break;
        default:
            break;
        }
    }
}

/**
 * @brief 初始化M701传感器
 */
esp_err_t m701_sensor_init(m701_data_callback_t callback)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing M701 sensor on GPIO%d", M701_UART_RX_PIN);
    
    // 保存回调
    s_data_callback = callback;
    
    // 配置UART
    uart_config_t uart_config = {
        .baud_rate = M701_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(M701_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 只设置RX引脚，TX不使用
    ret = uart_set_pin(M701_UART_NUM, UART_PIN_NO_CHANGE, M701_UART_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 安装UART驱动
    ret = uart_driver_install(M701_UART_NUM, UART_BUF_SIZE * 2, 0, 20, &s_uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 创建读取任务
    BaseType_t xReturned = xTaskCreate(m701_uart_task, "m701_task", 4096, NULL, 5, NULL);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        return ESP_FAIL;
    }
    
    s_initialized = true;
    ESP_LOGI(TAG, "M701 sensor initialized, waiting for data (2 min warmup)...");
    
    return ESP_OK;
}

/**
 * @brief 获取最新的传感器数据
 */
esp_err_t m701_sensor_get_data(m701_sensor_data_t *data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!s_sensor_data.valid) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(data, &s_sensor_data, sizeof(m701_sensor_data_t));
    return ESP_OK;
}

/**
 * @brief 将传感器数据格式化为JSON字符串
 */
int m701_sensor_to_json(const m701_sensor_data_t *data, char *buf, size_t buf_size)
{
    if (!data || !buf || buf_size == 0) {
        return 0;
    }
    
    return snprintf(buf, buf_size,
        "{\"co2\":%d,\"hcho\":%d,\"tvoc\":%d,\"pm25\":%d,\"pm10\":%d,\"temp\":%.1f,\"humi\":%.1f}",
        data->co2, data->hcho, data->tvoc, data->pm25, data->pm10,
        data->temperature, data->humidity);
}

