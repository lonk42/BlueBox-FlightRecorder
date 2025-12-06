#ifndef BMP280_H
#define BMP280_H

#include "esp_err.h"
#include <stdint.h>

// I2C Configuration
#define BMP280_I2C_PORT     I2C_NUM_0
#define BMP280_I2C_SDA      21
#define BMP280_I2C_SCL      22
#define BMP280_I2C_FREQ_HZ  400000
#define BMP280_I2C_ADDR     0x76  // or 0x77 depending on SDO pin

/**
 * @brief Initialize BMP280 sensor
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp280_init(void);

/**
 * @brief Read pressure from BMP280
 *
 * @param pressure Pointer to store pressure value in Pa
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp280_read_pressure(float *pressure);

/**
 * @brief Read temperature from BMP280
 *
 * @param temperature Pointer to store temperature value in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp280_read_temperature(float *temperature);

/**
 * @brief Read both pressure and temperature from BMP280
 *
 * @param pressure Pointer to store pressure value in Pa
 * @param temperature Pointer to store temperature value in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmp280_read_all(float *pressure, float *temperature);

#endif // BMP280_H
