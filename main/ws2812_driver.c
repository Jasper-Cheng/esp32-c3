/*
 * WS2812 LED驱动模块 - 实现文件
 * 
 * 基于您的参考代码重写，确保时序和逻辑完全一致
 */

#include "ws2812_driver.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>

/* 日志标签 */
static const char* TAG = "WS2812";

/* RMT配置 */
#define RESOLUTION_HZ 10000000

/* 全局变量 */
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

/* 内部结构体定义 */
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

/* 编码器实现 */
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size,
                                    rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;

    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to initial state
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    
    led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    // 完全照搬参考代码的时序配置
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.3 * RESOLUTION_HZ / 1000000, // T0H=0.3us
            .level1 = 0,
            .duration1 = 0.9 * RESOLUTION_HZ / 1000000, // T0L=0.9us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.9 * RESOLUTION_HZ / 1000000, // T1H=0.9us
            .level1 = 0,
            .duration1 = 0.3 * RESOLUTION_HZ / 1000000, // T1L=0.3us
        },
        .flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
    };
    
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG, "create copy encoder failed");
    
    // 完全照搬参考代码的Reset配置
    // 100us total reset time
    uint32_t reset_ticks = RESOLUTION_HZ / 1000000 * 100 / 2; 
    
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    
    *ret_encoder = &led_encoder->base;
    return ESP_OK;

err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

/**
 * @brief 初始化WS2812驱动
 */
esp_err_t ws2812_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing WS2812 driver with REFERENCE implementation");
    
    // 创建RMT TX通道
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = WS2812_GPIO_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = RESOLUTION_HZ,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    
    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &led_chan), err, TAG, "create tx channel failed");
    
    // 创建LED编码器
    ESP_GOTO_ON_ERROR(rmt_new_led_strip_encoder(&led_encoder), err, TAG, "create encoder failed");
    
    // 使能RMT通道
    ESP_GOTO_ON_ERROR(rmt_enable(led_chan), err, TAG, "enable rmt failed");
    
    ESP_LOGI(TAG, "WS2812 initialized on GPIO%d", WS2812_GPIO_PIN);
    
    // 初始化后清除所有LED
    ws2812_clear_all();
    return ESP_OK;

err:
    if (led_chan) {
        rmt_del_channel(led_chan);
    }
    if (led_encoder) {
        rmt_del_encoder(led_encoder);
    }
    return ret;
}

/**
 * @brief 更新LED状态
 */
esp_err_t ws2812_update_leds(uint8_t *led_data_in)
{
    // 颜色调色板 (GRB格式: 0xGGRRBB)
    static const uint32_t color_palette[] = {
        0x000000, // 0: 灭
        0x001000, // 1: 红 (G=0, R=16, B=0)
        0x0A1000, // 2: 橙 (G=10, R=16, B=0)
        0x101000, // 3: 黄 (G=16, R=16, B=0)
        0x100000, // 4: 绿 (G=16, R=0, B=0)
        0x100010, // 5: 青 (G=16, R=0, B=16)
        0x000010, // 6: 蓝 (G=0, R=0, B=16)
        0x000808, // 7: 紫 (G=0, R=8, B=8)
    };

    // 准备LED数据（GRB格式，每个LED 3字节）
    uint8_t led_data_out[WS2812_LED_COUNT * 3];
    int data_idx = 0;
    
    // 遍历32个LED
    for (int led = 0; led < WS2812_LED_COUNT; led++) {
        uint8_t color_idx = led_data_in[led];
        uint32_t color;
        
        if (color_idx < sizeof(color_palette)/sizeof(color_palette[0])) {
            color = color_palette[color_idx];
        } else {
            color = color_palette[1]; // 默认红色
        }
        
        // GRB顺序 (与参考代码一致: g, r, b)
        led_data_out[data_idx++] = (color >> 16) & 0xFF;  // Green
        led_data_out[data_idx++] = (color >> 8) & 0xFF;   // Red
        led_data_out[data_idx++] = color & 0xFF;          // Blue
    }
    
    // 配置传输
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    
    // 发送数据
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_data_out, sizeof(led_data_out), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, 100));
    
    ESP_LOGI(TAG, "LED updated success");
    return ESP_OK;
}

/**
 * @brief 清除所有LED
 */
esp_err_t ws2812_clear_all(void)
{
    uint8_t all_off[WS2812_LED_COUNT] = {0};
    return ws2812_update_leds(all_off);
}
