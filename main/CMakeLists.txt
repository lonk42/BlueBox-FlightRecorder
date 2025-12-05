idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 mpu6500 flash_storage circular_buffer speaker neo6m nvs_flash esp_wifi esp_http_server esp_timer)
