#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize flash storage partition
 *
 * Finds and erases the flight data partition, preparing it for writes.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t flash_storage_init(void);

/**
 * @brief Write a sample to flash storage
 *
 * Appends a sample to the flash partition. Automatically handles
 * sector alignment and tracks write position.
 *
 * @param data Pointer to sample data
 * @param size Size of sample in bytes
 * @return esp_err_t ESP_OK on success
 */
esp_err_t flash_storage_write(const void *data, size_t size);

/**
 * @brief Flush buffered writes to flash
 *
 * Forces any buffered data to be written to flash immediately.
 * Called automatically when buffer fills, but can be called manually.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t flash_storage_flush(void);

/**
 * @brief Read samples from flash storage
 *
 * Reads samples from flash starting at the given offset.
 *
 * @param offset Byte offset from start of data
 * @param data Pointer to buffer to read into
 * @param size Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
esp_err_t flash_storage_read(size_t offset, void *data, size_t size);

/**
 * @brief Get the total number of bytes written to flash
 *
 * @return size_t Number of bytes written
 */
size_t flash_storage_get_bytes_written(void);

/**
 * @brief Reset flash storage
 *
 * Erases the entire flight data partition and resets write position.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t flash_storage_reset(void);

/**
 * @brief Get the total capacity of the flash storage partition
 *
 * @return size_t Total partition size in bytes
 */
size_t flash_storage_get_capacity(void);

#endif // FLASH_STORAGE_H
