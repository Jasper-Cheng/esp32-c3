idf_component_register(SRCS "hello_world_main.c"
                            "ble_service.c"
                            "ws2812_driver.c"
                    INCLUDE_DIRS ""
                    REQUIRES nvs_flash bt driver)
