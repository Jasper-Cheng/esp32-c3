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
#include <stdio.h>

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
            // 不自动连接，等待应用层调用 wifi_manager_connect() 或使用系统保存的配置
            // esp_wifi_connect();  // 移除自动连接，避免与应用层连接冲突
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
                // 增加重试延迟，给WiFi/BLE共存机制更多时间协调
                vTaskDelay(pdMS_TO_TICKS(2000));
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
    ESP_LOGI(TAG, "Using WiFi/BLE coexistence mechanism");
    
    // 如果已经连接，先断开当前连接以支持重新配网
    if (s_connected) {
        ESP_LOGI(TAG, "Already connected, disconnecting first to allow reconfiguration");
        esp_wifi_disconnect();
        // 等待断开完成
        vTaskDelay(pdMS_TO_TICKS(500));
        s_connected = false;
        if (s_status_callback) {
            s_status_callback(false, NULL);
        }
    }
    
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
    
    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 检查WiFi状态，如果正在连接则跳过
    ret = esp_wifi_connect();
    if (ret == ESP_ERR_WIFI_CONN) {
        // WiFi正在连接中，直接返回成功，连接在后台进行
        ESP_LOGI(TAG, "WiFi is already connecting, connection in progress...");
        return ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 连接已启动，立即返回，连接结果通过回调函数通知
    ESP_LOGI(TAG, "WiFi connection started, result will be notified via callback");
    return ESP_OK;
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

/**
 * @brief 获取当前WiFi配置信息（JSON格式）
 */
int wifi_manager_get_config_json(char *json_buf, size_t buf_size)
{
    if (!json_buf || buf_size == 0) {
        return 0;
    }
    
    wifi_config_t wifi_config = {0};
    esp_err_t ret = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    
    if (ret != ESP_OK) {
        // 获取配置失败，返回状态信息
        return snprintf(json_buf, buf_size, "{\"status\":\"not_configured\"}");
    }
    
    // 获取SSID
    char ssid[33] = {0};
    strncpy(ssid, (char*)wifi_config.sta.ssid, sizeof(ssid) - 1);
    
    if (s_connected) {
        // 已连接，返回SSID、状态和IP
        return snprintf(json_buf, buf_size, 
                       "{\"ssid\":\"%s\",\"status\":\"connected\",\"ip\":\"%s\"}",
                       ssid, s_ip_addr[0] ? s_ip_addr : "0.0.0.0");
    } else {
        // 未连接，返回SSID和状态
        return snprintf(json_buf, buf_size, 
                       "{\"ssid\":\"%s\",\"status\":\"disconnected\"}",
                       ssid);
    }
}

