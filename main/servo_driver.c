/*
 * 舵机驱动模块 - 实现文件
 * 
 * 使用ESP32-C3的LEDC外设产生PWM信号控制TD-8120MG数字舵机
 * 
 * PWM计算说明：
 * - PWM频率：50Hz，周期20ms = 20000μs
 * - LEDC分辨率：14位 (16384级)
 * - 占空比计算：duty = (pulse_us / 20000) * 16384
 */

#include "servo_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"

/* 日志标签 */
static const char* TAG = "SERVO";

/* LEDC配置 */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT   // 14位分辨率 (0-16383)
#define LEDC_DUTY_MAX           16384               // 2^14

/* PWM周期(μs) */
#define PWM_PERIOD_US           (1000000 / SERVO_PWM_FREQ_HZ)  // 20000μs

/* 当前角度 */
static float s_current_angle = 0.0f;

/**
 * @brief 将脉冲宽度转换为LEDC占空比
 */
static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    // duty = (pulse_us / period_us) * max_duty
    return (uint32_t)((pulse_us * LEDC_DUTY_MAX) / PWM_PERIOD_US);
}

/**
 * @brief 将角度转换为脉冲宽度
 */
static uint32_t angle_to_pulse(float angle)
{
    // pulse = min_pulse + (angle / max_angle) * (max_pulse - min_pulse)
    uint32_t pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    return SERVO_MIN_PULSE_US + (uint32_t)((angle / SERVO_MAX_ANGLE) * pulse_range);
}

/**
 * @brief 初始化舵机驱动
 */
esp_err_t servo_init(void)
{
    ESP_LOGI(TAG, "Initializing servo driver on GPIO%d", SERVO_GPIO_PIN);
    
    // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = SERVO_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置LEDC通道
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_GPIO_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化到中立位置
    servo_center();
    
    ESP_LOGI(TAG, "Servo initialized, PWM freq=%dHz, resolution=%d bits", 
             SERVO_PWM_FREQ_HZ, LEDC_DUTY_RES);
    
    return ESP_OK;
}

/**
 * @brief 设置舵机角度
 */
esp_err_t servo_set_angle(float angle)
{
    // 角度范围检查
    if (angle < 0.0f || angle > SERVO_MAX_ANGLE) {
        ESP_LOGW(TAG, "Angle %.1f out of range [0, %.1f]", angle, SERVO_MAX_ANGLE);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算脉冲宽度和占空比
    uint32_t pulse_us = angle_to_pulse(angle);
    uint32_t duty = pulse_to_duty(pulse_us);
    
    // 设置LEDC占空比
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set duty failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Update duty failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    s_current_angle = angle;
    ESP_LOGI(TAG, "Servo angle set to %.1f deg (pulse=%lu us, duty=%lu)", 
             angle, pulse_us, duty);
    
    return ESP_OK;
}

/**
 * @brief 设置舵机脉冲宽度
 */
esp_err_t servo_set_pulse(uint32_t pulse_us)
{
    // 脉冲宽度范围检查
    if (pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US) {
        ESP_LOGW(TAG, "Pulse %lu us out of range [%d, %d]", 
                 pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算占空比
    uint32_t duty = pulse_to_duty(pulse_us);
    
    // 设置LEDC占空比
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 更新当前角度
    uint32_t pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    s_current_angle = ((float)(pulse_us - SERVO_MIN_PULSE_US) / pulse_range) * SERVO_MAX_ANGLE;
    
    ESP_LOGI(TAG, "Servo pulse set to %lu us (angle=%.1f deg)", pulse_us, s_current_angle);
    
    return ESP_OK;
}

/**
 * @brief 将舵机移动到中立位置
 */
esp_err_t servo_center(void)
{
    ESP_LOGI(TAG, "Moving servo to center position");
    return servo_set_pulse(SERVO_CENTER_PULSE_US);
}

/**
 * @brief 获取当前舵机角度
 */
float servo_get_angle(void)
{
    return s_current_angle;
}
