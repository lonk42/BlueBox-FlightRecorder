#include <stdio.h>
#include <string.h>
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
    STATE_ARMED,
    STATE_RECORDING,
    STATE_COMPLETE,
    STATE_WIFI_AP,
    STATE_ERROR
} bluebox_state_t;

static bluebox_state_t current_state = STATE_INIT;

// Recording duration (milliseconds)
#define RECORDING_DURATION_MS 20000  // 20 seconds max

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

    int64_t start_time = esp_timer_get_time();
    uint32_t sample_count = 0;

    while (current_state == STATE_RECORDING) {
        int64_t now = esp_timer_get_time();

        // Check if recording duration exceeded
        if ((now - start_time) > (RECORDING_DURATION_MS * 1000)) {
            current_state = STATE_COMPLETE;
            break;
        }

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

            // Debug output every 100 samples (~100ms at 1kHz)
            if (sample_count % 100 == 0) {
                ESP_LOGI(TAG, "Sample %lu: Gyro[%.1f,%.1f,%.1f] Accel[%.2f,%.2f,%.2f] P:%.1f T:%.1f",
                         sample_count,
                         sample.gyro_x, sample.gyro_y, sample.gyro_z,
                         sample.accel_x, sample.accel_y, sample.accel_z,
                         sample.pressure, sample.temperature);
            }
        }

        // Delay for target sample rate (1ms = 1kHz, adjust as needed)
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "Recording complete. Samples: %lu", sample_count);
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

    // Start recording immediately
    current_state = STATE_RECORDING;
    ESP_LOGI(TAG, "Starting recording...");

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

    // Main task monitors state on Core 1
    while (current_state == STATE_RECORDING) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Recording complete
    if (current_state == STATE_COMPLETE) {
        size_t buffer_used = circular_buffer_available(data_buffer);
        size_t samples_stored = buffer_used / sizeof(sensor_sample_t);
        ESP_LOGI(TAG, "Recording complete!");
        ESP_LOGI(TAG, "Buffer: %zu bytes used, %zu samples stored", buffer_used, samples_stored);
        speaker_beep(3, 200);  // Three beeps
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Start WiFi AP mode
        current_state = STATE_WIFI_AP;
        wifi_init_ap();

        // Start HTTP server
        httpd_handle_t server = start_webserver();

        if (server != NULL) {
            speaker_tone(1000, 500);  // Continuous tone for WiFi ready
            ESP_LOGI(TAG, "System ready. Connect to 'BlueBox' WiFi and access http://192.168.4.1/data");

            // Keep running
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}
