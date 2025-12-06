#ifndef MPU6500_H
#define MPU6500_H

#include "esp_err.h"
#include <stdint.h>

// SPI Configuration
#define MPU6500_SPI_HOST    SPI2_HOST
#define MPU6500_PIN_MOSI    23
#define MPU6500_PIN_MISO    19
#define MPU6500_PIN_CLK     18
#define MPU6500_PIN_CS      5
#define MPU6500_SPI_FREQ_HZ (1000000)  // 1MHz, can increase to 10MHz for faster reads

/**
 * @brief Initialize MPU6500 sensor
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_init(void);

/**
 * @brief Read gyroscope data
 *
 * @param gyro_x Pointer to store X-axis gyro data (deg/s)
 * @param gyro_y Pointer to store Y-axis gyro data (deg/s)
 * @param gyro_z Pointer to store Z-axis gyro data (deg/s)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_gyro(float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief Read accelerometer data
 *
 * @param accel_x Pointer to store X-axis accel data (g)
 * @param accel_y Pointer to store Y-axis accel data (g)
 * @param accel_z Pointer to store Z-axis accel data (g)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_accel(float *accel_x, float *accel_y, float *accel_z);

/**
 * @brief Read all sensor data (gyro + accel)
 *
 * @param gyro_x Pointer to store X-axis gyro data (deg/s)
 * @param gyro_y Pointer to store Y-axis gyro data (deg/s)
 * @param gyro_z Pointer to store Z-axis gyro data (deg/s)
 * @param accel_x Pointer to store X-axis accel data (g)
 * @param accel_y Pointer to store Y-axis accel data (g)
 * @param accel_z Pointer to store Z-axis accel data (g)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_all(float *gyro_x, float *gyro_y, float *gyro_z,
                           float *accel_x, float *accel_y, float *accel_z);

#endif // MPU6500_H
