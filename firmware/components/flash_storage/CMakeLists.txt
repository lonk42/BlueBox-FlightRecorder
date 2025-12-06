idf_component_register(SRCS "flash_storage.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_partition spi_flash)
