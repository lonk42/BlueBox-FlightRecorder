#include "neo6m.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "NEO6M";

#define UART_BUF_SIZE 1024
#define NMEA_MAX_LENGTH 256
#define GPS_DEBUG_ENABLED 1  // Set to 0 to disable verbose GPS debug

static neo6m_data_t current_gps_data = {0};
static bool gps_data_ready = false;
static uint32_t sentences_received = 0;
static uint32_t sentences_valid = 0;

// UBX protocol helpers
static void ubx_checksum(uint8_t *msg, int len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (int i = 2; i < len - 2; i++) {
        *ck_a = *ck_a + msg[i];
        *ck_b = *ck_b + *ck_a;
    }
}

// Send UBX command to GPS
static void send_ubx_command(const uint8_t *cmd, int len) {
    uart_write_bytes(NEO6M_UART_NUM, (const char *)cmd, len);
    uart_wait_tx_done(NEO6M_UART_NUM, pdMS_TO_TICKS(100));
}

// Configure GPS for 5Hz update rate
static void configure_5hz_rate(void) {
    // UBX-CFG-RATE: Set measurement rate to 200ms (5Hz)
    uint8_t rate_cmd[] = {
        0xB5, 0x62,       // UBX header
        0x06, 0x08,       // CFG-RATE
        0x06, 0x00,       // Length: 6 bytes
        0xC8, 0x00,       // measRate: 200ms (5Hz)
        0x01, 0x00,       // navRate: 1 cycle
        0x01, 0x00,       // timeRef: UTC
        0x00, 0x00        // Checksum (to be calculated)
    };

    uint8_t ck_a, ck_b;
    ubx_checksum(rate_cmd, sizeof(rate_cmd), &ck_a, &ck_b);
    rate_cmd[sizeof(rate_cmd) - 2] = ck_a;
    rate_cmd[sizeof(rate_cmd) - 1] = ck_b;

    ESP_LOGI(TAG, "Configuring GPS for 5Hz update rate");
    send_ubx_command(rate_cmd, sizeof(rate_cmd));
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Configure GPS for airborne <1g dynamic model
static void configure_airborne_mode(void) {
    // UBX-CFG-NAV5: Set dynamic platform model to airborne <1g
    uint8_t nav5_cmd[] = {
        0xB5, 0x62,       // UBX header
        0x06, 0x24,       // CFG-NAV5
        0x24, 0x00,       // Length: 36 bytes
        0xFF, 0xFF,       // mask: apply dynamic model only
        0x06,             // dynModel: 6 = airborne <1g
        0x03,             // fixMode: auto 2D/3D
        0x00, 0x00, 0x00, 0x00,  // fixedAlt
        0x10, 0x27, 0x00, 0x00,  // fixedAltVar
        0x05,             // minElev
        0x00,             // drLimit
        0xFA, 0x00,       // pDop
        0xFA, 0x00,       // tDop
        0x64, 0x00,       // pAcc
        0x2C, 0x01,       // tAcc
        0x00,             // staticHoldThresh
        0x3C,             // dgnssTimeout
        0x00,             // cnoThreshNumSVs
        0x00,             // cnoThresh
        0x00, 0x00,       // reserved
        0x00, 0x00,       // staticHoldMaxDist
        0x00,             // utcStandard
        0x00, 0x00, 0x00, 0x00, 0x00,  // reserved
        0x00, 0x00        // Checksum (to be calculated)
    };

    uint8_t ck_a, ck_b;
    ubx_checksum(nav5_cmd, sizeof(nav5_cmd), &ck_a, &ck_b);
    nav5_cmd[sizeof(nav5_cmd) - 2] = ck_a;
    nav5_cmd[sizeof(nav5_cmd) - 1] = ck_b;

    ESP_LOGI(TAG, "Configuring GPS for airborne <1g mode");
    send_ubx_command(nav5_cmd, sizeof(nav5_cmd));
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Parse NMEA latitude/longitude format (DDMM.MMMM to decimal degrees)
static double parse_nmea_coordinate(const char *coord_str, char direction) {
    if (!coord_str || strlen(coord_str) == 0) return 0.0;

    double coord = atof(coord_str);
    int degrees = (int)(coord / 100.0);
    double minutes = coord - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

// Parse GPGGA sentence (position and fix data)
static void parse_gpgga(const char *sentence) {
    char *token;
    char *saveptr;
    char sentence_copy[NMEA_MAX_LENGTH];
    int field = 0;

    strncpy(sentence_copy, sentence, NMEA_MAX_LENGTH - 1);
    sentence_copy[NMEA_MAX_LENGTH - 1] = '\0';

    token = strtok_r(sentence_copy, ",", &saveptr);

    char lat_str[16] = {0};
    char lat_dir = 'N';
    char lon_str[16] = {0};
    char lon_dir = 'E';
    char alt_str[16] = {0};
    char quality[2] = {0};
    char sat_str[4] = {0};
    char hdop_str[8] = {0};

    while (token != NULL && field < 15) {
        switch (field) {
            case 2:  // Latitude
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 3:  // N/S
                lat_dir = token[0];
                break;
            case 4:  // Longitude
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 5:  // E/W
                lon_dir = token[0];
                break;
            case 6:  // Fix quality
                strncpy(quality, token, sizeof(quality) - 1);
                break;
            case 7:  // Number of satellites
                strncpy(sat_str, token, sizeof(sat_str) - 1);
                break;
            case 8:  // HDOP
                strncpy(hdop_str, token, sizeof(hdop_str) - 1);
                break;
            case 9:  // Altitude
                strncpy(alt_str, token, sizeof(alt_str) - 1);
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    // Update GPS data if we have a valid fix
    if (quality[0] > '0') {
        current_gps_data.valid = true;
        current_gps_data.latitude = parse_nmea_coordinate(lat_str, lat_dir);
        current_gps_data.longitude = parse_nmea_coordinate(lon_str, lon_dir);
        current_gps_data.altitude = atof(alt_str);
        current_gps_data.satellites = atoi(sat_str);
        current_gps_data.hdop = atof(hdop_str);
        current_gps_data.timestamp_us = esp_timer_get_time();
        gps_data_ready = true;

        #if GPS_DEBUG_ENABLED
        static uint32_t last_fix_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_fix_log) > pdMS_TO_TICKS(5000)) {  // Log every 5 seconds
            ESP_LOGI(TAG, "GPS FIX: Lat=%.6f Lon=%.6f Alt=%.1fm Sat=%d HDOP=%.1f",
                     current_gps_data.latitude, current_gps_data.longitude,
                     current_gps_data.altitude, current_gps_data.satellites, current_gps_data.hdop);
            last_fix_log = now;
        }
        #endif
    } else {
        current_gps_data.valid = false;

        #if GPS_DEBUG_ENABLED
        static uint32_t last_nofix_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_nofix_log) > pdMS_TO_TICKS(5000)) {  // Log every 5 seconds
            ESP_LOGI(TAG, "GPS NO FIX: Quality=%c Sat=%s (waiting for satellites...)",
                     quality[0], sat_str[0] ? sat_str : "0");
            last_nofix_log = now;
        }
        #endif
    }
}

// Parse GPRMC sentence (speed and heading)
static void parse_gprmc(const char *sentence) {
    char *token;
    char *saveptr;
    char sentence_copy[NMEA_MAX_LENGTH];
    int field = 0;

    strncpy(sentence_copy, sentence, NMEA_MAX_LENGTH - 1);
    sentence_copy[NMEA_MAX_LENGTH - 1] = '\0';

    token = strtok_r(sentence_copy, ",", &saveptr);

    char speed_knots_str[16] = {0};
    char heading_str[16] = {0};
    char status = 'V';  // V = invalid, A = valid

    while (token != NULL && field < 12) {
        switch (field) {
            case 2:  // Status
                status = token[0];
                break;
            case 7:  // Speed over ground (knots)
                strncpy(speed_knots_str, token, sizeof(speed_knots_str) - 1);
                break;
            case 8:  // Track angle (degrees)
                strncpy(heading_str, token, sizeof(heading_str) - 1);
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    if (status == 'A') {
        // Convert knots to km/h
        float speed_knots = atof(speed_knots_str);
        current_gps_data.speed_kmh = speed_knots * 1.852f;
        current_gps_data.heading = atof(heading_str);
    }
}

// Verify NMEA checksum
static bool verify_nmea_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;

    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) return false;

    // Calculate checksum
    uint8_t checksum = 0;
    for (const char *p = sentence + 1; p < asterisk; p++) {
        checksum ^= *p;
    }

    // Parse expected checksum
    char checksum_str[3] = {asterisk[1], asterisk[2], '\0'};
    uint8_t expected = (uint8_t)strtol(checksum_str, NULL, 16);

    return checksum == expected;
}

// GPS reading task
static void gps_task(void *pvParameters) {
    ESP_LOGI(TAG, "GPS task started on core %d", xPortGetCoreID());

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    char nmea_buffer[NMEA_MAX_LENGTH] = {0};
    int nmea_idx = 0;
    uint32_t last_activity_log = 0;
    bool synced = false;  // Track if we're synced to sentence boundaries

    while (1) {
        int len = uart_read_bytes(NEO6M_UART_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(20));

        if (len > 0) {
            data[len] = '\0';

            // Process byte by byte to extract NMEA sentences
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];

                if (c == '$') {
                    // Start of new sentence - reset buffer
                    nmea_idx = 0;
                    nmea_buffer[nmea_idx++] = c;
                    synced = true;  // We're now synchronized
                } else if ((c == '\r' || c == '\n') && synced) {
                    // End of sentence (only process if we're synced)
                    if (nmea_idx > 0 && nmea_buffer[0] == '$') {
                        nmea_buffer[nmea_idx] = '\0';
                        sentences_received++;

                        // Only log valid-looking sentences (must start with $ and have reasonable length)
                        if (nmea_idx > 10 && strchr(nmea_buffer, '*') != NULL) {
                            #if GPS_DEBUG_ENABLED
                            // Log raw NMEA sentences every 10 seconds
                            uint32_t now = xTaskGetTickCount();
                            if ((now - last_activity_log) > pdMS_TO_TICKS(10000)) {
                                ESP_LOGI(TAG, "NMEA: %s", nmea_buffer);
                                last_activity_log = now;
                            }
                            #endif

                            // Verify and parse sentence
                            if (verify_nmea_checksum(nmea_buffer)) {
                                sentences_valid++;
                                if (strstr(nmea_buffer, "$GPGGA") || strstr(nmea_buffer, "$GNGGA")) {
                                    parse_gpgga(nmea_buffer);
                                } else if (strstr(nmea_buffer, "$GPRMC") || strstr(nmea_buffer, "$GNRMC")) {
                                    parse_gprmc(nmea_buffer);
                                }
                            }
                        }
                        nmea_idx = 0;
                    }
                } else if (synced && nmea_idx > 0 && nmea_idx < NMEA_MAX_LENGTH - 1) {
                    // Only collect data if we're synced and have started a sentence
                    nmea_buffer[nmea_idx++] = c;
                }
                // Ignore all other characters (garbage before first '$')
            }
        } else {
            #if GPS_DEBUG_ENABLED
            // No data received - check if GPS is responsive
            uint32_t now = xTaskGetTickCount();
            static uint32_t last_timeout_log = 0;
            if ((now - last_timeout_log) > pdMS_TO_TICKS(15000)) {  // Log every 15 seconds
                ESP_LOGW(TAG, "No GPS data received (check wiring/power). Received: %lu Valid: %lu",
                         sentences_received, sentences_valid);
                last_timeout_log = now;
            }
            #endif
        }
    }

    free(data);
    vTaskDelete(NULL);
}

esp_err_t neo6m_init(void) {
    ESP_LOGI(TAG, "Initializing NEO-6m GPS module (UART2: TX=GPIO%d RX=GPIO%d, 9600 baud)",
             NEO6M_TXD_PIN, NEO6M_RXD_PIN);

    // Configure UART2
    const uart_config_t uart_config = {
        .baud_rate = NEO6M_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(NEO6M_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(NEO6M_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEO6M_UART_NUM, NEO6M_TXD_PIN, NEO6M_RXD_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 configured, waiting for GPS module to boot...");

    // Wait for GPS to boot
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure GPS
    configure_5hz_rate();
    configure_airborne_mode();

    ESP_LOGI(TAG, "NEO-6m configuration sent (5Hz rate, airborne <1g mode)");
    ESP_LOGI(TAG, "GPS will begin acquiring satellites (cold start: ~26s, check antenna/sky view)");
    return ESP_OK;
}

esp_err_t neo6m_start_task(void) {
    BaseType_t result = xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t neo6m_get_data(neo6m_data_t *data) {
    if (!gps_data_ready) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(data, &current_gps_data, sizeof(neo6m_data_t));
    return ESP_OK;
}

bool neo6m_has_fix(void) {
    return current_gps_data.valid && gps_data_ready;
}
