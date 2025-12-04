idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bmp280 mpu6500 circular_buffer speaker nvs_flash esp_wifi esp_http_server esp_timer)
