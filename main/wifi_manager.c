/*
 * WiFi管理器 - 实现文件
 * 
 * 功能：管理WiFi连接
 */

#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

/* 日志标签 */
static const char* TAG = "WIFI_MGR";

/* WiFi事件组 */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* 全局变量 */
static wifi_status_callback_t s_status_callback = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static const int WIFI_MAX_RETRY = 5;
static bool s_connected = false;
static char s_ip_addr[16] = {0};

/**
 * @brief WiFi事件处理函数
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi station started");
            esp_wifi_connect();
            break;
            
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected to AP");
            s_retry_num = 0;
            break;
            
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "WiFi disconnected");
            s_connected = false;
            if (s_status_callback) {
                s_status_callback(false, NULL);
            }
            
            if (s_retry_num < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retry to connect to AP (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "Failed to connect after %d retries", WIFI_MAX_RETRY);
            }
            break;
            
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            
            snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
            s_connected = true;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            
            if (s_status_callback) {
                s_status_callback(true, s_ip_addr);
            }
        }
    }
}

/**
 * @brief 初始化WiFi管理器
 */
esp_err_t wifi_manager_init(wifi_status_callback_t callback)
{
    ESP_LOGI(TAG, "Initializing WiFi manager");
    
    s_status_callback = callback;
    s_connected = false;
    s_retry_num = 0;
    
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    // 创建事件组
    s_wifi_event_group = xEventGroupCreate();
    
    // WiFi配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    // 设置WiFi模式为STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi manager initialized");
    return ESP_OK;
}

/**
 * @brief 连接WiFi
 */
esp_err_t wifi_manager_connect(const char *ssid, const char *password)
{
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
    
    // 清除事件组
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry_num = 0;
    
    // 配置WiFi
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    // 等待连接结果
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(30000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 断开WiFi连接
 */
esp_err_t wifi_manager_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting WiFi");
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    s_connected = false;
    if (s_status_callback) {
        s_status_callback(false, NULL);
    }
    return ESP_OK;
}

/**
 * @brief 获取WiFi连接状态
 */
bool wifi_manager_is_connected(void)
{
    return s_connected;
}

/**
 * @brief 获取当前IP地址
 */
const char* wifi_manager_get_ip(char *ip_buf, size_t buf_size)
{
    if (!s_connected || !ip_buf) {
        return NULL;
    }
    
    if (buf_size > 0) {
        strncpy(ip_buf, s_ip_addr, buf_size - 1);
        ip_buf[buf_size - 1] = '\0';
    }
    
    return s_ip_addr;
}

