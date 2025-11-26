/*
 * WS2812 LED驱动模块 - 头文件
 * 
 * 功能：提供WS2812智能LED的初始化和控制接口
 * 支持：最多60个独立LED的控制
 */

#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* 配置参数 */
#define WS2812_LED_COUNT    60          // LED数量
#define WS2812_GPIO_PIN     GPIO_NUM_1  // GPIO引脚

/* RGB颜色定义 (GRB格式) */
#define WS2812_COLOR_OFF    0x000000
#define WS2812_COLOR_ON     0x101010    // 默认白色，低亮度

/**
 * @brief 初始化WS2812驱动
 * 
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t ws2812_init(void);

/**
 * @brief 更新LED状态
 * 
 * 根据60字节数据更新LED状态，每字节对应一个LED
 * 0:灭, 1:红, 2:橙, 3:黄, 4:绿, 5:青, 6:蓝, 7:紫
 * 
 * @param led_data 60字节LED颜色索引数组
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t ws2812_update_leds(uint8_t *led_data);

/**
 * @brief 清除所有LED（全部熄灭）
 * 
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t ws2812_clear_all(void);

#endif // WS2812_DRIVER_H

