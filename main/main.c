#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

#include "bmp280.h"
#include "mpu6500.h"
#include "circular_buffer.h"
#include "speaker.h"

static const char *TAG = "BLUEBOX";

// Flight recorder states
typedef enum {
    STATE_INIT,
    STATE_LAUNCH_MODE,
    STATE_FLIGHT_MODE,
    STATE_RECOVERY_MODE,
    STATE_ERROR
} bluebox_state_t;

static bluebox_state_t current_state = STATE_INIT;

// Launch detection parameters
#define LAUNCH_GYRO_THRESHOLD 150.0f      // deg/s - threshold for launch detection
#define LAUNCH_DETECTION_DURATION_MS 250  // 250ms of sustained high acceleration

// Landing detection parameters
#define LANDING_GYRO_THRESHOLD 10.0f      // deg/s - threshold for stable/landed
#define LANDING_STABLE_DURATION_MS 1000   // 1 second of stability to consider landed

// Buffer management for launch mode (1 second of data)
#define LAUNCH_MODE_BUFFER_DURATION_MS 1000

// Sensor data structure
typedef struct {
    int64_t timestamp_us;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float pressure;
    float temperature;
} sensor_sample_t;

// Global circular buffer handle
static circular_buffer_t *data_buffer = NULL;

// Task to read MPU6500 at high speed (Core 0)
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started on core %d", xPortGetCoreID());

    uint32_t sample_count = 0;
    int64_t launch_mode_start = esp_timer_get_time();
    int64_t last_buffer_reset = launch_mode_start;

    // Launch detection
    int64_t high_accel_start_time = 0;
    bool in_high_accel = false;

    // Landing detection
    int64_t stable_start_time = 0;
    bool is_stable = false;

    while (current_state != STATE_RECOVERY_MODE && current_state != STATE_ERROR) {
        int64_t now = esp_timer_get_time();

        sensor_sample_t sample;
        sample.timestamp_us = now;

        // Read MPU6500 (gyro + accel) - fast SPI read
        if (mpu6500_read_gyro(&sample.gyro_x, &sample.gyro_y, &sample.gyro_z) == ESP_OK &&
            mpu6500_read_accel(&sample.accel_x, &sample.accel_y, &sample.accel_z) == ESP_OK) {

            // Read BMP280 every 10th sample (100Hz if MPU is 1kHz)
            if (sample_count % 10 == 0) {
                bmp280_read_pressure(&sample.pressure);
                bmp280_read_temperature(&sample.temperature);
            }

            // Write to circular buffer
            circular_buffer_write(data_buffer, &sample, sizeof(sensor_sample_t));
            sample_count++;

            // Calculate gyro magnitude for launch/landing detection
            float gyro_magnitude = sqrtf(sample.gyro_x * sample.gyro_x +
                                         sample.gyro_y * sample.gyro_y +
                                         sample.gyro_z * sample.gyro_z);

            // LAUNCH MODE: Monitor for launch
            if (current_state == STATE_LAUNCH_MODE) {
                // Reset buffer every 1 second to maintain 1-second window
                if ((now - last_buffer_reset) > (LAUNCH_MODE_BUFFER_DURATION_MS * 1000)) {
                    circular_buffer_reset(data_buffer);
                    last_buffer_reset = now;
                    sample_count = 0;
                }

                // Detect launch by sustained high gyro activity
                if (gyro_magnitude > LAUNCH_GYRO_THRESHOLD) {
                    if (!in_high_accel) {
                        high_accel_start_time = now;
                        in_high_accel = true;
                    } else if ((now - high_accel_start_time) > (LAUNCH_DETECTION_DURATION_MS * 1000)) {
                        ESP_LOGI(TAG, "LAUNCH DETECTED! Gyro magnitude: %.1f deg/s", gyro_magnitude);
                        current_state = STATE_FLIGHT_MODE;
                        speaker_stop();  // Stop launch mode beeps
                        speaker_start_continuous(1500);  // Start flight mode continuous tone
                    }
                } else {
                    in_high_accel = false;
                }

                // Debug output every 100 samples
                if (sample_count % 100 == 0) {
                    ESP_LOGI(TAG, "[LAUNCH] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f",
                             sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z, gyro_magnitude);
                }
            }
            // FLIGHT MODE: Monitor for landing
            else if (current_state == STATE_FLIGHT_MODE) {
                // Check if gyro is stable (below threshold)
                if (gyro_magnitude < LANDING_GYRO_THRESHOLD) {
                    if (!is_stable) {
                        stable_start_time = now;
                        is_stable = true;
                    } else if ((now - stable_start_time) > (LANDING_STABLE_DURATION_MS * 1000)) {
                        ESP_LOGI(TAG, "LANDING DETECTED! Gyro stabilized at %.1f deg/s", gyro_magnitude);
                        size_t buffer_used = circular_buffer_available(data_buffer);
                        size_t samples_stored = buffer_used / sizeof(sensor_sample_t);
                        ESP_LOGI(TAG, "Flight complete! Buffer: %zu bytes, %zu samples", buffer_used, samples_stored);
                        current_state = STATE_RECOVERY_MODE;
                        speaker_stop();  // Stop flight mode tone
                        break;
                    }
                } else {
                    is_stable = false;
                }

                // Debug output every 100 samples
                if (sample_count % 100 == 0) {
                    ESP_LOGI(TAG, "[FLIGHT] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f Stable:%s",
                             sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z,
                             gyro_magnitude, is_stable ? "YES" : "NO");
                }
            }
        }

        // Delay for target sample rate (1ms = 1kHz, adjust as needed)
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "Sensor task ending. Total samples: %lu", sample_count);
    vTaskDelete(NULL);
}

// Task for periodic beeps in launch and recovery modes
void beep_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Beep task started on core %d", xPortGetCoreID());

    while (current_state != STATE_ERROR) {
        if (current_state == STATE_LAUNCH_MODE) {
            speaker_two_tone_pair();
            vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3 seconds
        } else if (current_state == STATE_RECOVERY_MODE) {
            speaker_triple_tone();
            vTaskDelay(pdMS_TO_TICKS(6000));  // Wait 6 seconds
        } else if (current_state == STATE_FLIGHT_MODE) {
            // Flight mode uses continuous tone (started by sensor task)
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    vTaskDelete(NULL);
}

// Initialize WiFi Access Point
void wifi_init_ap(void)
{
    ESP_LOGI(TAG, "Starting WiFi Access Point");

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "BlueBox",
            .ssid_len = strlen("BlueBox"),
            .channel = 1,
            .password = "bluebox123",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID: BlueBox, Password: bluebox123");
}

// HTTP handler to serve recorded data
esp_err_t data_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving recorded data");

    // Send HTTP headers
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{\"samples\":[");

    // Read all samples from circular buffer and send as JSON
    size_t available = circular_buffer_available(data_buffer);
    size_t sample_count = available / sizeof(sensor_sample_t);

    sensor_sample_t sample;
    char json_buf[256];

    for (size_t i = 0; i < sample_count; i++) {
        if (circular_buffer_read(data_buffer, &sample, sizeof(sensor_sample_t)) > 0) {
            snprintf(json_buf, sizeof(json_buf),
                     "%s{\"t\":%lld,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"p\":%.2f,\"temp\":%.2f}",
                     (i > 0) ? "," : "",
                     sample.timestamp_us,
                     sample.gyro_x, sample.gyro_y, sample.gyro_z,
                     sample.accel_x, sample.accel_y, sample.accel_z,
                     sample.pressure, sample.temperature);
            httpd_resp_sendstr_chunk(req, json_buf);
        }
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

// Start HTTP server
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t data_uri = {
            .uri       = "/data",
            .method    = HTTP_GET,
            .handler   = data_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &data_uri);
        ESP_LOGI(TAG, "HTTP server started");
    }

    return server;
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize speaker
    speaker_init();
    speaker_beep(1, 200);  // Power on beep
    vTaskDelay(pdMS_TO_TICKS(300));

    current_state = STATE_INIT;
    ESP_LOGI(TAG, "BlueBox Flight Recorder starting...");

    // Initialize sensors
    ESP_LOGI(TAG, "Initializing sensors...");

    if (bmp280_init() != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization failed");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    if (mpu6500_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6500 initialization failed");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    speaker_beep(2, 200);  // Sensors initialized
    ESP_LOGI(TAG, "Sensors initialized successfully");

    // Test sensor readings
    float gx, gy, gz, ax, ay, az, pressure, temperature;
    ESP_LOGI(TAG, "Testing sensors...");
    if (mpu6500_read_gyro(&gx, &gy, &gz) == ESP_OK) {
        ESP_LOGI(TAG, "MPU6500 Gyro: X=%.1f Y=%.1f Z=%.1f deg/s", gx, gy, gz);
    }
    if (mpu6500_read_accel(&ax, &ay, &az) == ESP_OK) {
        ESP_LOGI(TAG, "MPU6500 Accel: X=%.2f Y=%.2f Z=%.2f g", ax, ay, az);
    }
    if (bmp280_read_pressure(&pressure) == ESP_OK) {
        ESP_LOGI(TAG, "BMP280 Pressure: %.1f Pa (%.2f kPa)", pressure, pressure / 1000.0f);
    }
    if (bmp280_read_temperature(&temperature) == ESP_OK) {
        ESP_LOGI(TAG, "BMP280 Temperature: %.2f C", temperature);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // Create circular buffer (limited by internal RAM, no PSRAM on this board)
    // At 1kHz: ~64KB / 48 bytes/sample = ~1365 samples = ~1.3 seconds of flight data
    // Circular buffer will overwrite old data, keeping most recent samples
    size_t buffer_size = 64 * 1024;  // 64KB buffer
    data_buffer = circular_buffer_create(buffer_size);

    if (data_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to create circular buffer");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    ESP_LOGI(TAG, "Circular buffer created: %zu bytes", buffer_size);

    // Enter LAUNCH MODE
    current_state = STATE_LAUNCH_MODE;
    ESP_LOGI(TAG, "Entering LAUNCH MODE - waiting for launch...");
    ESP_LOGI(TAG, "Launch threshold: %.1f deg/s", LAUNCH_GYRO_THRESHOLD);

    // Create sensor task on Core 0 for maximum performance
    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // High priority
        NULL,
        0  // Core 0
    );

    // Create beep task on Core 1
    xTaskCreatePinnedToCore(
        beep_task,
        "beep_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,  // Low priority
        NULL,
        1  // Core 1
    );

    // Main task monitors state transitions on Core 1
    while (current_state == STATE_LAUNCH_MODE || current_state == STATE_FLIGHT_MODE) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Transition to RECOVERY MODE
    if (current_state == STATE_RECOVERY_MODE) {
        ESP_LOGI(TAG, "Entering RECOVERY MODE");

        // Start WiFi AP mode
        wifi_init_ap();

        // Start HTTP server
        httpd_handle_t server = start_webserver();

        if (server != NULL) {
            ESP_LOGI(TAG, "System ready. Connect to 'BlueBox' WiFi and access http://192.168.4.1/data");

            // Keep running (beep task will continue triple-tone pattern)
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}
