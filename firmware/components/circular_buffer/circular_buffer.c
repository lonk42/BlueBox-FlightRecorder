#include "circular_buffer.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "CIRCULAR_BUFFER";

struct circular_buffer {
    uint8_t *buffer;
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
};

circular_buffer_t *circular_buffer_create(size_t size)
{
    circular_buffer_t *cb = malloc(sizeof(circular_buffer_t));
    if (cb == NULL) {
        ESP_LOGE(TAG, "Failed to allocate circular buffer structure");
        return NULL;
    }

    cb->buffer = malloc(size);
    if (cb->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer memory");
        free(cb);
        return NULL;
    }

    cb->size = size;
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;

    ESP_LOGI(TAG, "Circular buffer created: %zu bytes", size);
    return cb;
}

void circular_buffer_destroy(circular_buffer_t *cb)
{
    if (cb != NULL) {
        if (cb->buffer != NULL) {
            free(cb->buffer);
        }
        free(cb);
    }
}

size_t circular_buffer_write(circular_buffer_t *cb, const void *data, size_t len)
{
    if (cb == NULL || data == NULL || len == 0) {
        return 0;
    }

    // Check if buffer has space
    size_t space_available = cb->size - cb->count;
    if (len > space_available) {
        // Buffer full, overwrite oldest data
        // IMPORTANT: Remove whole samples to maintain alignment
        size_t overflow = len - space_available;
        // Calculate how many complete samples to remove (round up)
        size_t samples_to_remove = (overflow + len - 1) / len;
        size_t bytes_to_remove = samples_to_remove * len;

        // Ensure we don't remove more than available
        if (bytes_to_remove > cb->count) {
            bytes_to_remove = cb->count;
        }

        cb->tail = (cb->tail + bytes_to_remove) % cb->size;
        cb->count -= bytes_to_remove;
    }

    // Write data
    const uint8_t *src = (const uint8_t *)data;
    size_t bytes_written = 0;

    while (bytes_written < len) {
        size_t chunk = len - bytes_written;
        size_t space_to_end = cb->size - cb->head;

        if (chunk > space_to_end) {
            chunk = space_to_end;
        }

        memcpy(&cb->buffer[cb->head], &src[bytes_written], chunk);
        cb->head = (cb->head + chunk) % cb->size;
        cb->count += chunk;
        bytes_written += chunk;
    }

    return bytes_written;
}

size_t circular_buffer_read(circular_buffer_t *cb, void *data, size_t len)
{
    if (cb == NULL || data == NULL || len == 0) {
        return 0;
    }

    // Check available data
    if (len > cb->count) {
        len = cb->count;
    }

    if (len == 0) {
        return 0;
    }

    // Read data
    uint8_t *dst = (uint8_t *)data;
    size_t bytes_read = 0;

    while (bytes_read < len) {
        size_t chunk = len - bytes_read;
        size_t data_to_end = cb->size - cb->tail;

        if (chunk > data_to_end) {
            chunk = data_to_end;
        }

        memcpy(&dst[bytes_read], &cb->buffer[cb->tail], chunk);
        cb->tail = (cb->tail + chunk) % cb->size;
        cb->count -= chunk;
        bytes_read += chunk;
    }

    return bytes_read;
}

size_t circular_buffer_available(circular_buffer_t *cb)
{
    if (cb == NULL) {
        return 0;
    }
    return cb->count;
}

void circular_buffer_reset(circular_buffer_t *cb)
{
    if (cb != NULL) {
        cb->head = 0;
        cb->tail = 0;
        cb->count = 0;
    }
}
