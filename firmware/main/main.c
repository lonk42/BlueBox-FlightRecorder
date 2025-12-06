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
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

#include "bmp280.h"
#include "mpu6500.h"
#include "flash_storage.h"
#include "circular_buffer.h"
#include "speaker.h"
#include "neo6m.h"

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
    // GPS data (sampled at 5Hz)
    uint8_t gps_valid;          // 1 if GPS fix is valid, 0 otherwise
    double gps_latitude;         // Latitude in degrees
    double gps_longitude;        // Longitude in degrees
    float gps_altitude;          // Altitude in meters
    float gps_speed_kmh;         // Speed in km/h
    float gps_heading;           // Heading in degrees
    uint8_t gps_satellites;      // Number of satellites
} sensor_sample_t;

// Launch mode RAM buffer (holds 1 second of data before launch)
static circular_buffer_t *launch_buffer = NULL;

// Task to read MPU6500 at high speed (Core 0)
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started on core %d", xPortGetCoreID());

    uint32_t sample_count = 0;

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

        // Initialize GPS fields to defaults
        sample.gps_valid = 0;
        sample.gps_latitude = 0.0;
        sample.gps_longitude = 0.0;
        sample.gps_altitude = 0.0f;
        sample.gps_speed_kmh = 0.0f;
        sample.gps_heading = 0.0f;
        sample.gps_satellites = 0;

        // Read MPU6500 (gyro + accel) - fast SPI read
        if (mpu6500_read_gyro(&sample.gyro_x, &sample.gyro_y, &sample.gyro_z) == ESP_OK &&
            mpu6500_read_accel(&sample.accel_x, &sample.accel_y, &sample.accel_z) == ESP_OK) {

            // Read BMP280 every 10th sample (100Hz if MPU is 1kHz)
            if (sample_count % 10 == 0) {
                bmp280_read_pressure(&sample.pressure);
                bmp280_read_temperature(&sample.temperature);
            }

            // Read GPS every 200th sample (5Hz if MPU is 1kHz)
            if (sample_count % 200 == 0) {
                neo6m_data_t gps_data;
                if (neo6m_get_data(&gps_data) == ESP_OK && gps_data.valid) {
                    sample.gps_valid = 1;
                    sample.gps_latitude = gps_data.latitude;
                    sample.gps_longitude = gps_data.longitude;
                    sample.gps_altitude = gps_data.altitude;
                    sample.gps_speed_kmh = gps_data.speed_kmh;
                    sample.gps_heading = gps_data.heading;
                    sample.gps_satellites = gps_data.satellites;
                }
            }

            // Write to appropriate storage based on state
            if (current_state == STATE_LAUNCH_MODE) {
                // In Launch Mode: write to RAM buffer only
                circular_buffer_write(launch_buffer, &sample, sizeof(sensor_sample_t));
            } else if (current_state == STATE_FLIGHT_MODE) {
                // In Flight Mode: write directly to flash
                flash_storage_write(&sample, sizeof(sensor_sample_t));
            }
            sample_count++;

            // Calculate gyro magnitude for launch/landing detection
            float gyro_magnitude = sqrtf(sample.gyro_x * sample.gyro_x +
                                         sample.gyro_y * sample.gyro_y +
                                         sample.gyro_z * sample.gyro_z);

            // LAUNCH MODE: Monitor for launch
            if (current_state == STATE_LAUNCH_MODE) {
                // Detect launch by sustained high gyro activity
                if (gyro_magnitude > LAUNCH_GYRO_THRESHOLD) {
                    if (!in_high_accel) {
                        high_accel_start_time = now;
                        in_high_accel = true;
                    } else if ((now - high_accel_start_time) > (LAUNCH_DETECTION_DURATION_MS * 1000)) {
                        ESP_LOGI(TAG, "LAUNCH DETECTED! Gyro magnitude: %.1f deg/s", gyro_magnitude);

                        // Flush pre-launch RAM buffer to flash
                        size_t prelaunch_bytes = circular_buffer_available(launch_buffer);
                        size_t prelaunch_samples = prelaunch_bytes / sizeof(sensor_sample_t);
                        ESP_LOGI(TAG, "Flushing %zu pre-launch samples from RAM to flash...", prelaunch_samples);

                        sensor_sample_t temp_sample;
                        while (circular_buffer_read(launch_buffer, &temp_sample, sizeof(sensor_sample_t)) > 0) {
                            flash_storage_write(&temp_sample, sizeof(sensor_sample_t));
                        }
                        ESP_LOGI(TAG, "Pre-launch data written to flash");

                        current_state = STATE_FLIGHT_MODE;
                        speaker_stop();  // Stop launch mode beeps
                        speaker_start_continuous(1500);  // Start flight mode continuous tone
                    }
                } else {
                    in_high_accel = false;
                }

                // Debug output every 100 samples
                if (sample_count % 100 == 0) {
                    if (sample.gps_valid) {
                        ESP_LOGI(TAG, "[LAUNCH] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f P:%.1fkPa T:%.1fC GPS[%.6f,%.6f] Alt:%.1fm Sat:%d",
                                 sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z, gyro_magnitude,
                                 sample.pressure / 1000.0f, sample.temperature,
                                 sample.gps_latitude, sample.gps_longitude, sample.gps_altitude, sample.gps_satellites);
                    } else {
                        ESP_LOGI(TAG, "[LAUNCH] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f P:%.1fkPa T:%.1fC GPS:NO_FIX",
                                 sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z, gyro_magnitude,
                                 sample.pressure / 1000.0f, sample.temperature);
                    }
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

                        // Flush any buffered data to flash
                        flash_storage_flush();

                        size_t bytes_written = flash_storage_get_bytes_written();
                        size_t samples_stored = bytes_written / sizeof(sensor_sample_t);
                        ESP_LOGI(TAG, "Flight complete! Flash: %zu bytes, %zu samples", bytes_written, samples_stored);
                        current_state = STATE_RECOVERY_MODE;
                        speaker_stop();  // Stop flight mode tone
                        break;
                    }
                } else {
                    is_stable = false;
                }

                // Debug output every 100 samples
                if (sample_count % 100 == 0) {
                    if (sample.gps_valid) {
                        ESP_LOGI(TAG, "[FLIGHT] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f P:%.1fkPa T:%.1fC Stable:%s GPS[%.6f,%.6f] Spd:%.1fkm/h",
                                 sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z,
                                 gyro_magnitude, sample.pressure / 1000.0f, sample.temperature,
                                 is_stable ? "YES" : "NO",
                                 sample.gps_latitude, sample.gps_longitude, sample.gps_speed_kmh);
                    } else {
                        ESP_LOGI(TAG, "[FLIGHT] Sample %lu: Gyro[%.1f,%.1f,%.1f] Mag:%.1f P:%.1fkPa T:%.1fC Stable:%s GPS:NO_FIX",
                                 sample_count, sample.gyro_x, sample.gyro_y, sample.gyro_z,
                                 gyro_magnitude, sample.pressure / 1000.0f, sample.temperature,
                                 is_stable ? "YES" : "NO");
                    }
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
            // Check GPS fix status and beep accordingly
            if (neo6m_has_fix()) {
                // GPS has fix: double beep (ready for flight)
                speaker_beep(2, 150);
            } else {
                // GPS no fix: single beep (waiting for GPS)
                speaker_beep(1, 150);
            }
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

// WiFi event handler for Station Mode
static EventGroupHandle_t wifi_event_group = NULL;
static const int WIFI_CONNECTED_BIT = BIT0;
static int wifi_retry_count = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi Station started, attempting connection...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_retry_count++;
        ESP_LOGI(TAG, "Disconnected from AP, retry attempt %d...", wifi_retry_count);

        #ifdef CONFIG_BLUEBOX_AUDIO_FEEDBACK
        // Single low beep for disconnect
        speaker_tone(500, 200);
        #endif

        // Wait before retry based on config
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BLUEBOX_WIFI_RETRY_INTERVAL_SEC * 1000));
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        #ifdef CONFIG_BLUEBOX_AUDIO_FEEDBACK
        // Double beep for successful connection
        speaker_beep(2, 200);
        #endif
    }
}

// WiFi initialization task (runs on separate stack to avoid main task overflow)
static volatile bool wifi_init_complete = false;
static volatile bool wifi_connected = false;

void wifi_init_ap_task(void *pvParameters)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "WiFi AP Mode init task started on core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "WiFi init task stack: %u bytes", uxTaskGetStackHighWaterMark(NULL));

    ESP_LOGI(TAG, "Creating default WiFi AP netif...");
    esp_netif_create_default_wifi_ap();

    ESP_LOGI(TAG, "Initializing WiFi driver...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Configuring WiFi AP...");
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

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set config failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting WiFi...");
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WiFi AP started successfully. SSID: BlueBox, Password: bluebox123");
    ESP_LOGI(TAG, "WiFi init task final stack watermark: %u bytes", uxTaskGetStackHighWaterMark(NULL));

    wifi_init_complete = true;
    wifi_connected = true;  // AP mode is always "connected"
    vTaskDelete(NULL);
}

void wifi_init_station_task(void *pvParameters)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "WiFi Station Mode init task started on core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "WiFi init task stack: %u bytes", uxTaskGetStackHighWaterMark(NULL));

    // Create event group for connection status
    wifi_event_group = xEventGroupCreate();

    ESP_LOGI(TAG, "Creating default WiFi STA netif...");
    esp_netif_create_default_wifi_sta();

    ESP_LOGI(TAG, "Initializing WiFi driver...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_LOGI(TAG, "Configuring WiFi Station Mode...");
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_BLUEBOX_WIFI_SSID,
            .password = CONFIG_BLUEBOX_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_BLUEBOX_WIFI_SSID);

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set mode failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi set config failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting WiFi...");
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(ret));
        wifi_init_complete = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WiFi Station Mode started. Waiting for connection...");
    wifi_init_complete = true;

    // Wait for connection indefinitely
    ESP_LOGI(TAG, "Waiting for WiFi connection to %s...", CONFIG_BLUEBOX_WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully!");
        wifi_connected = true;
    }

    ESP_LOGI(TAG, "WiFi init task final stack watermark: %u bytes", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(NULL);
}

// HTTP write callback for chunked upload
typedef struct {
    size_t sample_count;
    size_t current_index;
    bool first_sample;
} upload_context_t;

static int http_upload_chunk_cb(esp_http_client_event_t *evt)
{
    return ESP_OK;
}

// HTTP POST to upload flight data to webapp
esp_err_t upload_flight_data_to_webapp(void)
{
    ESP_LOGI(TAG, "=== UPLOADING FLIGHT DATA TO WEBAPP ===");

    size_t bytes_written = flash_storage_get_bytes_written();
    size_t sample_count = bytes_written / sizeof(sensor_sample_t);

    ESP_LOGI(TAG, "Preparing to upload %zu samples", sample_count);
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    // Allocate buffer for JSON construction (8KB should be enough for chunks)
    char *json_buffer = malloc(8192);
    if (!json_buffer) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        return ESP_FAIL;
    }

    // Build JSON header with device ID
    int header_len = snprintf(json_buffer, 8192, "{\"device_id\":\"%s\",\"samples\":[",
    #ifdef CONFIG_BLUEBOX_DEVICE_ID
        CONFIG_BLUEBOX_DEVICE_ID
    #else
        "bluebox-001"
    #endif
    );

    ESP_LOGI(TAG, "JSON header size: %d bytes", header_len);

    // Build URL
    char url[256];
    snprintf(url, sizeof(url), "%s/api/flights", CONFIG_BLUEBOX_WEBAPP_URL);
    ESP_LOGI(TAG, "Uploading to: %s", url);

    // Check if URL is HTTPS
    bool is_https = (strncmp(url, "https://", 8) == 0);

    // Configure HTTP client for chunked upload
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 60000,  // 60 second timeout for large uploads
        .buffer_size = 2048,
        .buffer_size_tx = 2048,
        .is_async = false,
    };

    // Enable SSL/TLS for HTTPS URLs
    if (is_https) {
        config.transport_type = HTTP_TRANSPORT_OVER_SSL;
        config.skip_cert_common_name_check = true;
        config.use_global_ca_store = true;  // Set to true to satisfy verification requirement
        ESP_LOGI(TAG, "HTTPS detected - enabling SSL/TLS without cert verification");
    }

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(json_buffer);
        return ESP_FAIL;
    }

    // Set headers
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // Open connection
    esp_err_t err = esp_http_client_open(client, -1);  // -1 = unknown content length (chunked)
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(json_buffer);
        return ESP_FAIL;
    }

    // Write JSON header
    int written = esp_http_client_write(client, json_buffer, header_len);
    if (written < 0) {
        ESP_LOGE(TAG, "Failed to write JSON header");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(json_buffer);
        return ESP_FAIL;
    }

    // Stream samples one at a time
    sensor_sample_t sample;
    for (size_t i = 0; i < sample_count; i++) {
        if (i > 0 && i % 500 == 0) {
            ESP_LOGI(TAG, "Uploaded %zu / %zu samples", i, sample_count);
        }

        size_t offset = i * sizeof(sensor_sample_t);
        if (flash_storage_read(offset, &sample, sizeof(sensor_sample_t)) == ESP_OK) {
            // Build JSON for this sample
            int sample_len;
            if (sample.gps_valid) {
                sample_len = snprintf(json_buffer, 8192,
                    "%s{\"t\":%lld,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"p\":%.2f,\"temp\":%.2f,\"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.1f,\"hdg\":%.1f,\"sat\":%u}}",
                    (i > 0) ? "," : "",
                    sample.timestamp_us,
                    sample.gyro_x, sample.gyro_y, sample.gyro_z,
                    sample.accel_x, sample.accel_y, sample.accel_z,
                    sample.pressure, sample.temperature,
                    sample.gps_latitude, sample.gps_longitude, sample.gps_altitude,
                    sample.gps_speed_kmh, sample.gps_heading, sample.gps_satellites);
            } else {
                sample_len = snprintf(json_buffer, 8192,
                    "%s{\"t\":%lld,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"p\":%.2f,\"temp\":%.2f}",
                    (i > 0) ? "," : "",
                    sample.timestamp_us,
                    sample.gyro_x, sample.gyro_y, sample.gyro_z,
                    sample.accel_x, sample.accel_y, sample.accel_z,
                    sample.pressure, sample.temperature);
            }

            // Write this sample
            written = esp_http_client_write(client, json_buffer, sample_len);
            if (written < 0) {
                ESP_LOGE(TAG, "Failed to write sample %zu", i);
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                free(json_buffer);
                return ESP_FAIL;
            }
        }
    }

    // Write JSON footer
    const char *footer = "]}";
    written = esp_http_client_write(client, footer, strlen(footer));
    if (written < 0) {
        ESP_LOGE(TAG, "Failed to write JSON footer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(json_buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All data written, fetching response...");

    // Fetch response
    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d", status_code, content_length);

    // Close connection
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(json_buffer);

    if (status_code == 200 || status_code == 201) {
        ESP_LOGI(TAG, "✓ Flight data uploaded successfully!");

        #ifdef CONFIG_BLUEBOX_AUDIO_FEEDBACK
        // Success tone: three ascending beeps
        speaker_tone(800, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        speaker_tone(1000, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        speaker_tone(1200, 200);
        #endif

        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Upload failed with status code: %d", status_code);

        #ifdef CONFIG_BLUEBOX_AUDIO_FEEDBACK
        // Failure tone: descending beeps
        speaker_tone(1200, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        speaker_tone(800, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        speaker_tone(400, 200);
        #endif

        return ESP_FAIL;
    }
}

// Task to continuously attempt upload in Station Mode
void upload_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Upload task started. Waiting for WiFi connection...");

    // Wait for WiFi to be connected
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "WiFi connected. Starting upload attempts...");

    while (1) {
        esp_err_t result = upload_flight_data_to_webapp();

        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Upload successful! Task complete.");
            // Successfully uploaded, we can stop trying
            vTaskDelete(NULL);
            return;
        }

        // Wait before retry
        ESP_LOGI(TAG, "Upload failed. Retrying in %d seconds...", CONFIG_BLUEBOX_UPLOAD_RETRY_INTERVAL_SEC);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BLUEBOX_UPLOAD_RETRY_INTERVAL_SEC * 1000));
    }
}

// HTTP handler for root path
esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Root endpoint accessed");
    const char* resp_str = "<html><body><h1>BlueBox Flight Recorder</h1><p>Access flight data at <a href=\"/data\">/data</a></p></body></html>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP handler to serve recorded data
esp_err_t data_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "=== DATA ENDPOINT ACCESSED ===");
    ESP_LOGI(TAG, "Client connected, preparing to serve flight data");

    // Send HTTP headers
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"bluebox-flight-data.json\"");

    ESP_LOGI(TAG, "Sending JSON header...");
    esp_err_t ret = httpd_resp_sendstr_chunk(req, "{\"samples\":[");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send JSON header: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read all samples from flash and send as JSON
    size_t bytes_written = flash_storage_get_bytes_written();
    size_t sample_count = bytes_written / sizeof(sensor_sample_t);

    ESP_LOGI(TAG, "Flash contains %zu bytes, %zu samples", bytes_written, sample_count);

    sensor_sample_t sample;
    char json_buf[512];

    ESP_LOGI(TAG, "Starting to send %zu samples...", sample_count);

    for (size_t i = 0; i < sample_count; i++) {
        // Log progress every 1000 samples
        if (i > 0 && i % 1000 == 0) {
            ESP_LOGI(TAG, "Sent %zu / %zu samples", i, sample_count);
        }

        size_t offset = i * sizeof(sensor_sample_t);
        if (flash_storage_read(offset, &sample, sizeof(sensor_sample_t)) == ESP_OK) {
            if (sample.gps_valid) {
                snprintf(json_buf, sizeof(json_buf),
                         "%s{\"t\":%lld,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"p\":%.2f,\"temp\":%.2f,\"gps\":{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.1f,\"hdg\":%.1f,\"sat\":%u}}",
                         (i > 0) ? "," : "",
                         sample.timestamp_us,
                         sample.gyro_x, sample.gyro_y, sample.gyro_z,
                         sample.accel_x, sample.accel_y, sample.accel_z,
                         sample.pressure, sample.temperature,
                         sample.gps_latitude, sample.gps_longitude, sample.gps_altitude,
                         sample.gps_speed_kmh, sample.gps_heading, sample.gps_satellites);
            } else {
                snprintf(json_buf, sizeof(json_buf),
                         "%s{\"t\":%lld,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"p\":%.2f,\"temp\":%.2f}",
                         (i > 0) ? "," : "",
                         sample.timestamp_us,
                         sample.gyro_x, sample.gyro_y, sample.gyro_z,
                         sample.accel_x, sample.accel_y, sample.accel_z,
                         sample.pressure, sample.temperature);
            }

            ret = httpd_resp_sendstr_chunk(req, json_buf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send sample %zu: %s", i, esp_err_to_name(ret));
                return ret;
            }
        }
    }

    ESP_LOGI(TAG, "Sending JSON footer...");
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);

    ESP_LOGI(TAG, "Data transfer complete - sent %zu samples", sample_count);
    return ESP_OK;
}

// Start HTTP server
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 8192;  // Increase stack size for large data transfers
    config.task_priority = tskIDLE_PRIORITY + 3;  // Higher priority

    ESP_LOGI(TAG, "Starting HTTP server on port %d (stack: %d bytes)...",
             config.server_port, config.stack_size);

    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server started successfully");

        // Register root handler
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(server, &root_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Registered / endpoint");
        } else {
            ESP_LOGE(TAG, "Failed to register / endpoint: %s", esp_err_to_name(ret));
        }

        // Register data handler
        httpd_uri_t data_uri = {
            .uri       = "/data",
            .method    = HTTP_GET,
            .handler   = data_get_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(server, &data_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Registered /data endpoint");
        } else {
            ESP_LOGE(TAG, "Failed to register /data endpoint: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
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

    if (neo6m_init() != ESP_OK) {
        ESP_LOGE(TAG, "NEO-6m GPS initialization failed");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    // Start GPS task to begin acquiring fix
    if (neo6m_start_task() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start GPS task");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    speaker_beep(2, 200);  // Sensors initialized
    ESP_LOGI(TAG, "Sensors initialized successfully");
    ESP_LOGI(TAG, "GPS acquiring fix...");

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

    // Initialize flash storage for flight data
    // 2MB flash partition can hold ~23,250 samples = ~23 seconds at 1kHz
    // Sample size: 8+24+8+30(GPS) = ~86 bytes (timestamp, IMU, BMP280, GPS)
    if (flash_storage_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flash storage");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    size_t flash_capacity = flash_storage_get_capacity();
    size_t max_samples = flash_capacity / sizeof(sensor_sample_t);
    float max_seconds = (float)max_samples / 1000.0f;  // Assuming 1kHz sampling

    ESP_LOGI(TAG, "Flash storage initialized: %zu bytes capacity", flash_capacity);
    ESP_LOGI(TAG, "Can store ~%zu samples (~%.1f seconds of flight data)", max_samples, max_seconds);

    // Create RAM buffer for Launch Mode (1 second of data before launch)
    // At 1kHz: 1000 samples × 86 bytes = ~86KB
    size_t launch_buffer_size = 86 * 1024;  // 86KB buffer
    launch_buffer = circular_buffer_create(launch_buffer_size);

    if (launch_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to create launch mode RAM buffer");
        current_state = STATE_ERROR;
        speaker_beep_sos();
        return;
    }

    size_t launch_samples = launch_buffer_size / sizeof(sensor_sample_t);
    ESP_LOGI(TAG, "Launch mode RAM buffer created: %zu bytes (~%zu samples, ~%.1f seconds)",
             launch_buffer_size, launch_samples, (float)launch_samples / 1000.0f);

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

        // Give tasks time to fully terminate and free resources
        vTaskDelay(pdMS_TO_TICKS(500));

        // Free launch mode RAM buffer to reclaim memory for WiFi
        if (launch_buffer != NULL) {
            ESP_LOGI(TAG, "Freeing launch mode RAM buffer...");
            circular_buffer_destroy(launch_buffer);
            launch_buffer = NULL;
        }

        // Log memory status before WiFi init
        ESP_LOGI(TAG, "Free heap before WiFi: %lu bytes (largest block: %lu bytes)",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

        #ifdef CONFIG_BLUEBOX_RECOVERY_MODE_STATION
        // Station Mode: Connect to WiFi network and upload data
        ESP_LOGI(TAG, "Recovery Mode: STATION MODE");
        ESP_LOGI(TAG, "Will connect to WiFi and upload to: %s", CONFIG_BLUEBOX_WEBAPP_URL);

        wifi_init_complete = false;
        wifi_connected = false;

        // Start WiFi Station mode task
        xTaskCreatePinnedToCore(
            wifi_init_station_task,
            "wifi_station",
            10240,  // 10KB stack for WiFi init and PHY calibration
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL,
            1  // Core 1
        );

        // Wait for WiFi initialization to start
        while (!wifi_init_complete) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Free heap after WiFi init: %lu bytes", (unsigned long)esp_get_free_heap_size());

        // Start upload task (will wait for connection and retry indefinitely)
        xTaskCreatePinnedToCore(
            upload_task,
            "upload_task",
            8192,  // 8KB stack for HTTP client
            NULL,
            tskIDLE_PRIORITY + 1,
            NULL,
            1  // Core 1
        );

        ESP_LOGI(TAG, "Upload task started. Will automatically upload when WiFi connects.");

        // Keep running (upload task and beep task continue in background)
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        #else
        // AP Mode: Create access point for manual data retrieval
        ESP_LOGI(TAG, "Recovery Mode: AP MODE");
        ESP_LOGI(TAG, "Starting WiFi Access Point");

        wifi_init_complete = false;
        wifi_connected = false;

        xTaskCreatePinnedToCore(
            wifi_init_ap_task,
            "wifi_ap",
            10240,  // 10KB stack for WiFi init and PHY calibration
            NULL,
            tskIDLE_PRIORITY + 2,
            NULL,
            1  // Core 1
        );

        // Wait for WiFi initialization to complete
        while (!wifi_init_complete) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Free heap after WiFi: %lu bytes", (unsigned long)esp_get_free_heap_size());

        // Start HTTP server
        httpd_handle_t server = start_webserver();

        if (server != NULL) {
            ESP_LOGI(TAG, "System ready. Connect to 'BlueBox' WiFi and access http://192.168.4.1/data");

            // Keep running (beep task will continue triple-tone pattern)
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        #endif
    }
}
