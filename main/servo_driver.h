/*
 * 舵机驱动模块 - 头文件
 * 
 * 功能：使用LEDC外设产生PWM信号控制TD-8120MG数字舵机
 * 
 * TD-8120MG舵机参数：
 * - 脉冲宽度范围：500~2500μs
 * - 中立位置：1500μs
 * - 运行角度：270°
 * - PWM频率：50Hz（周期20ms）
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* 舵机配置参数 */
#define SERVO_GPIO_PIN          GPIO_NUM_2      // 舵机信号引脚
#define SERVO_PWM_FREQ_HZ       50              // PWM频率50Hz
#define SERVO_MIN_PULSE_US      500             // 最小脉冲宽度(μs)
#define SERVO_MAX_PULSE_US      2500            // 最大脉冲宽度(μs)
#define SERVO_CENTER_PULSE_US   1500            // 中立位置脉冲宽度(μs)
#define SERVO_MAX_ANGLE         270.0f          // 最大角度(度)

/**
 * @brief 初始化舵机驱动
 * 
 * 配置LEDC外设产生50Hz PWM信号
 * 
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_FAIL: 失败
 */
esp_err_t servo_init(void);

/**
 * @brief 设置舵机角度
 * 
 * @param angle 目标角度 (0.0 ~ 270.0度)
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 角度超出范围
 */
esp_err_t servo_set_angle(float angle);

/**
 * @brief 设置舵机脉冲宽度
 * 
 * @param pulse_us 脉冲宽度 (500 ~ 2500μs)
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 脉冲宽度超出范围
 */
esp_err_t servo_set_pulse(uint32_t pulse_us);

/**
 * @brief 将舵机移动到中立位置
 * 
 * @return 
 *     - ESP_OK: 成功
 */
esp_err_t servo_center(void);

/**
 * @brief 获取当前舵机角度
 * 
 * @return 当前角度值 (0.0 ~ 270.0度)
 */
float servo_get_angle(void);

#endif // SERVO_DRIVER_H
