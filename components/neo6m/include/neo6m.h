#ifndef NEO6M_H
#define NEO6M_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// UART Configuration for NEO-6m
#define NEO6M_UART_NUM      UART_NUM_2
#define NEO6M_TXD_PIN       17
#define NEO6M_RXD_PIN       16
#define NEO6M_BAUD_RATE     9600

// GPS data structure
typedef struct {
    bool valid;                 // GPS fix valid
    double latitude;            // Latitude in degrees (positive = North)
    double longitude;           // Longitude in degrees (positive = East)
    float altitude;             // Altitude in meters above sea level
    float speed_kmh;            // Ground speed in km/h
    float heading;              // Course over ground in degrees
    uint8_t satellites;         // Number of satellites in use
    float hdop;                 // Horizontal dilution of precision
    int64_t timestamp_us;       // ESP32 timestamp when data was captured
} neo6m_data_t;

/**
 * @brief Initialize NEO-6m GPS module
 *
 * Configures UART2 and sends UBX commands to:
 * - Set update rate to 5Hz
 * - Set dynamic platform model to airborne <1g
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t neo6m_init(void);

/**
 * @brief Start GPS data collection task
 *
 * Spawns a FreeRTOS task that continuously reads and parses NMEA sentences
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t neo6m_start_task(void);

/**
 * @brief Get the latest GPS data
 *
 * Returns the most recently parsed GPS data. Non-blocking.
 *
 * @param data Pointer to neo6m_data_t structure to fill
 * @return esp_err_t ESP_OK if data is available, ESP_ERR_NOT_FOUND if no data yet
 */
esp_err_t neo6m_get_data(neo6m_data_t *data);

/**
 * @brief Check if GPS has a valid fix
 *
 * @return true if GPS has a valid fix, false otherwise
 */
bool neo6m_has_fix(void);

#endif // NEO6M_H
