/*
 * BLE服务模块 - 实现文件
 * 
 * 功能：实现BLE GATT服务器，处理蓝牙连接和数据通信
 */

#include "ble_service.h"
#include "ws2812_driver.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include <ctype.h>

/* 日志标签 */
static const char* TAG = "BLE_SERVICE";

/* GATT服务配置 */
#define GATTS_NUM_HANDLE    4
#define ADV_CONFIG_FLAG     BIT0
#define SCAN_RSP_CONFIG_FLAG BIT1

/* 全局变量 */
static uint8_t g_led_data[WS2812_LED_COUNT] = {0};      // 当前LED数据
static ble_led_data_callback_t g_led_callback = NULL;   // LED数据回调
static uint8_t adv_config_state = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
static esp_gatt_if_t ble_gatts_if = ESP_GATT_IF_NONE;
static uint16_t ble_service_handle = 0;
static uint16_t ble_char_handle = 0;
static uint16_t ble_conn_id = 0;

static bool parse_led_data_string(const uint8_t *data, uint16_t len, uint8_t *out_led_data)
{
    if (!data || !out_led_data || len == 0) {
        return false;
    }
    int count = 0;
    for (uint16_t i = 0; i < len && count < WS2812_LED_COUNT; i++) {
        char c = (char)data[i];
        if (c < '0' || c > '7') {
            return false;
        }
        out_led_data[count++] = (uint8_t)(c - '0');
    }
    if (count == 0) {
        return false;
    }
    for (int i = count; i < WS2812_LED_COUNT; i++) {
        out_led_data[i] = 0;
    }
    return true;
}

static uint8_t adv_payload[] = {
    0x02, 0x01, 0x06,
    0x08, 0x09, 'E', 'S', 'P', '-', 'L', 'E', 'D',
    0x03, 0x03, 0xFF, 0x00,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_gatt_srvc_id_t ble_service_id = {
    .is_primary = true,
    .id = {
        .inst_id = 0x00,
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = BLE_SERVICE_UUID}
        }
    }
};

static esp_bt_uuid_t ble_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = BLE_CHAR_UUID}
};

/**
 * @brief GAP事件处理函数
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_state &= ~ADV_CONFIG_FLAG;
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_state &= ~SCAN_RSP_CONFIG_FLAG;
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        }
        break;
    default:
        break;
    }

    if (adv_config_state == 0) {
        esp_ble_gap_start_advertising(&adv_params);
        adv_config_state = 0xFF; // prevent repeated start until reconfigured
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATT registration failed: %d", param->reg.status);
            return;
        }
        ble_gatts_if = gatts_if;
        adv_config_state = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data_raw(adv_payload, sizeof(adv_payload));
        esp_ble_gap_config_scan_rsp_data_raw(adv_payload, sizeof(adv_payload));
        esp_ble_gatts_create_service(gatts_if, &ble_service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ble_service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(ble_service_handle);
        esp_ble_gatts_add_char(ble_service_handle, &ble_char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ |
                               ESP_GATT_CHAR_PROP_BIT_WRITE |
                               ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ble_char_handle = param->add_char.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        ble_conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (parse_led_data_string(param->write.value, param->write.len, g_led_data)) {
            if (g_led_callback) {
                g_led_callback(g_led_data);
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, ble_char_handle,
                                        WS2812_LED_COUNT, g_led_data, false);
        } else {
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_INVALID_ATTR_LEN, NULL);
            }
        }
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = WS2812_LED_COUNT;
        memcpy(rsp.attr_value.value, g_led_data, WS2812_LED_COUNT);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief 初始化BLE服务
 */
esp_err_t ble_service_init(ble_led_data_callback_t callback)
{
    esp_err_t ret;

    // 保存回调函数
    g_led_callback = callback;

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化Bluedroid协议栈
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "init bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "enable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    esp_ble_gatt_set_local_mtu(128);

    ESP_LOGI(TAG, "BLE ready, waiting for connections");
    return ESP_OK;
}

/**
 * @brief 获取当前LED数据
 */
uint8_t* ble_service_get_led_data(void)
{
    return g_led_data;
}
