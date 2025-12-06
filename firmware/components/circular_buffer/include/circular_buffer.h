#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stddef.h>

typedef struct circular_buffer circular_buffer_t;

/**
 * @brief Create a circular buffer
 *
 * @param size Size of the buffer in bytes
 * @return circular_buffer_t* Pointer to the created buffer, or NULL on failure
 */
circular_buffer_t *circular_buffer_create(size_t size);

/**
 * @brief Destroy a circular buffer
 *
 * @param cb Pointer to the circular buffer
 */
void circular_buffer_destroy(circular_buffer_t *cb);

/**
 * @brief Write data to the circular buffer
 *
 * @param cb Pointer to the circular buffer
 * @param data Pointer to data to write
 * @param len Length of data in bytes
 * @return size_t Number of bytes written
 */
size_t circular_buffer_write(circular_buffer_t *cb, const void *data, size_t len);

/**
 * @brief Read data from the circular buffer
 *
 * @param cb Pointer to the circular buffer
 * @param data Pointer to buffer to read into
 * @param len Maximum number of bytes to read
 * @return size_t Number of bytes read
 */
size_t circular_buffer_read(circular_buffer_t *cb, void *data, size_t len);

/**
 * @brief Get the number of bytes available to read
 *
 * @param cb Pointer to the circular buffer
 * @return size_t Number of bytes available
 */
size_t circular_buffer_available(circular_buffer_t *cb);

/**
 * @brief Reset the circular buffer
 *
 * @param cb Pointer to the circular buffer
 */
void circular_buffer_reset(circular_buffer_t *cb);

#endif // CIRCULAR_BUFFER_H
