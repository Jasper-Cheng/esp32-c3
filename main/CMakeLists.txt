idf_component_register(SRCS "mqtt_wrapper.c" "wifi_manager.c" "servo_driver.c" "hello_world_main.c"
                            "ble_service.c"
                            "ws2812_driver.c"
                            "m701_sensor.c"
                    INCLUDE_DIRS ""
                    REQUIRES nvs_flash bt driver mqtt cjson mbedtls
                             esp_driver_gpio esp_driver_ledc esp_driver_rmt esp_driver_uart)
