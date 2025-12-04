#include "speaker.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SPEAKER";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_DUTY               (4096)  // 50% duty cycle
#define LEDC_FREQUENCY          (1000)

esp_err_t speaker_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return err;
    }

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SPEAKER_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Speaker initialized on GPIO %d", SPEAKER_GPIO);
    return ESP_OK;
}

esp_err_t speaker_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    // Set frequency
    esp_err_t err = ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frequency: %s", esp_err_to_name(err));
        return err;
    }

    // Set duty cycle (50%)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Wait for duration
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        speaker_stop();
    }

    return ESP_OK;
}

esp_err_t speaker_beep(uint8_t count, uint32_t duration_ms)
{
    for (uint8_t i = 0; i < count; i++) {
        speaker_tone(1000, duration_ms);
        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));  // Pause between beeps
        }
    }
    return ESP_OK;
}

esp_err_t speaker_beep_sos(void)
{
    // S: three short beeps
    for (int i = 0; i < 3; i++) {
        speaker_tone(1000, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    // O: three long beeps
    for (int i = 0; i < 3; i++) {
        speaker_tone(1000, 300);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    // S: three short beeps
    for (int i = 0; i < 3; i++) {
        speaker_tone(1000, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}

void speaker_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

esp_err_t speaker_two_tone_pair(void)
{
    // Low tone
    speaker_tone(800, 150);
    vTaskDelay(pdMS_TO_TICKS(100));
    // High tone
    speaker_tone(1200, 150);
    return ESP_OK;
}

esp_err_t speaker_triple_tone(void)
{
    // Three ascending tones
    speaker_tone(800, 150);
    vTaskDelay(pdMS_TO_TICKS(100));
    speaker_tone(1000, 150);
    vTaskDelay(pdMS_TO_TICKS(100));
    speaker_tone(1200, 150);
    return ESP_OK;
}

esp_err_t speaker_start_continuous(uint32_t frequency_hz)
{
    // Set frequency
    esp_err_t err = ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency_hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frequency: %s", esp_err_to_name(err));
        return err;
    }

    // Set duty cycle (50%) and keep playing
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    return ESP_OK;
}
