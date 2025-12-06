#ifndef SPEAKER_H
#define SPEAKER_H

#include "esp_err.h"

#define SPEAKER_GPIO 25
#define LED_GPIO 2

/**
 * @brief Initialize the speaker
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_init(void);

/**
 * @brief Play a tone for a specified duration
 *
 * @param frequency_hz Frequency in Hz
 * @param duration_ms Duration in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_tone(uint32_t frequency_hz, uint32_t duration_ms);

/**
 * @brief Play a number of beeps
 *
 * @param count Number of beeps
 * @param duration_ms Duration of each beep in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_beep(uint8_t count, uint32_t duration_ms);

/**
 * @brief Play SOS pattern
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_beep_sos(void);

/**
 * @brief Stop any playing tone
 */
void speaker_stop(void);

/**
 * @brief Play a two-tone pair (low then high)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_two_tone_pair(void);

/**
 * @brief Play a triple-tone pattern
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_triple_tone(void);

/**
 * @brief Start a continuous tone (doesn't stop automatically)
 *
 * @param frequency_hz Frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t speaker_start_continuous(uint32_t frequency_hz);

#endif // SPEAKER_H
