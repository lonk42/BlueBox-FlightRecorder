#include "flash_storage.h"
#include "esp_partition.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "FLASH_STORAGE";

static const esp_partition_t *flight_partition = NULL;
static size_t write_position = 0;
static uint8_t *write_buffer = NULL;
static size_t buffer_position = 0;

#define FLASH_SECTOR_SIZE 4096
#define WRITE_BUFFER_SIZE FLASH_SECTOR_SIZE

esp_err_t flash_storage_init(void) {
    ESP_LOGI(TAG, "Initializing flash storage");

    // List all partitions for debugging
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (it != NULL) {
        ESP_LOGI(TAG, "Available data partitions:");
        while (it != NULL) {
            const esp_partition_t *part = esp_partition_get(it);
            ESP_LOGI(TAG, "  - %s (type=0x%02x, subtype=0x%02x, size=%lu bytes)",
                     part->label, part->type, part->subtype, part->size);
            it = esp_partition_next(it);
        }
        esp_partition_iterator_release(it);
    }

    // Find the flight data partition by name
    // Using ESP_PARTITION_SUBTYPE_ANY since we're using custom subtype 0x40
    flight_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        "flightdata"
    );

    if (flight_partition == NULL) {
        ESP_LOGE(TAG, "Flight data partition 'flightdata' not found");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Found flight data partition: size=%lu bytes", flight_partition->size);

    // Allocate write buffer for sector-aligned writes
    write_buffer = (uint8_t *)malloc(WRITE_BUFFER_SIZE);
    if (write_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate write buffer");
        return ESP_ERR_NO_MEM;
    }

    // Erase the partition
    ESP_LOGI(TAG, "Erasing flight data partition...");
    esp_err_t err = esp_partition_erase_range(flight_partition, 0, flight_partition->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase partition: %s", esp_err_to_name(err));
        free(write_buffer);
        return err;
    }

    write_position = 0;
    buffer_position = 0;
    memset(write_buffer, 0xFF, WRITE_BUFFER_SIZE);

    ESP_LOGI(TAG, "Flash storage initialized successfully");
    return ESP_OK;
}

esp_err_t flash_storage_write(const void *data, size_t size) {
    if (flight_partition == NULL || write_buffer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (write_position + size > flight_partition->size) {
        ESP_LOGW(TAG, "Flash storage full, wrapping around");
        // Reset to beginning (circular behavior)
        write_position = 0;
        buffer_position = 0;
        memset(write_buffer, 0xFF, WRITE_BUFFER_SIZE);
    }

    const uint8_t *src = (const uint8_t *)data;
    size_t remaining = size;
    size_t offset = 0;

    while (remaining > 0) {
        size_t space_in_buffer = WRITE_BUFFER_SIZE - buffer_position;
        size_t to_copy = (remaining < space_in_buffer) ? remaining : space_in_buffer;

        memcpy(write_buffer + buffer_position, src + offset, to_copy);
        buffer_position += to_copy;
        offset += to_copy;
        remaining -= to_copy;

        // If buffer is full, flush to flash
        if (buffer_position >= WRITE_BUFFER_SIZE) {
            size_t aligned_addr = (write_position / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;

            esp_err_t err = esp_partition_write(flight_partition, aligned_addr,
                                                write_buffer, WRITE_BUFFER_SIZE);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Flash write failed: %s", esp_err_to_name(err));
                return err;
            }

            write_position += WRITE_BUFFER_SIZE;
            buffer_position = 0;
            memset(write_buffer, 0xFF, WRITE_BUFFER_SIZE);
        }
    }

    return ESP_OK;
}

esp_err_t flash_storage_flush(void) {
    if (flight_partition == NULL || write_buffer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Flush remaining buffer to flash if there's any data
    if (buffer_position > 0) {
        size_t aligned_addr = (write_position / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;

        esp_err_t err = esp_partition_write(flight_partition, aligned_addr,
                                            write_buffer, WRITE_BUFFER_SIZE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Flash flush failed: %s", esp_err_to_name(err));
            return err;
        }

        write_position += buffer_position;
        buffer_position = 0;
        memset(write_buffer, 0xFF, WRITE_BUFFER_SIZE);
    }

    return ESP_OK;
}

esp_err_t flash_storage_read(size_t offset, void *data, size_t size) {
    if (flight_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (offset + size > write_position) {
        ESP_LOGW(TAG, "Attempted to read beyond written data");
        size = write_position - offset;
    }

    return esp_partition_read(flight_partition, offset, data, size);
}

size_t flash_storage_get_bytes_written(void) {
    return write_position + buffer_position;
}

esp_err_t flash_storage_reset(void) {
    if (flight_partition == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting flash storage");

    esp_err_t err = esp_partition_erase_range(flight_partition, 0, flight_partition->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase partition: %s", esp_err_to_name(err));
        return err;
    }

    write_position = 0;
    buffer_position = 0;
    if (write_buffer != NULL) {
        memset(write_buffer, 0xFF, WRITE_BUFFER_SIZE);
    }

    return ESP_OK;
}

size_t flash_storage_get_capacity(void) {
    if (flight_partition == NULL) {
        return 0;
    }
    return flight_partition->size;
}
