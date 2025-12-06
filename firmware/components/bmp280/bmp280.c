#include "bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BMP280";

// BMP280 Registers
#define BMP280_REG_TEMP_XLSB   0xFC
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB_START 0x88

// Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_data_t;

static bmp280_calib_data_t calib_data;
static int32_t t_fine;

// I2C helper functions
static esp_err_t bmp280_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(BMP280_I2C_PORT, BMP280_I2C_ADDR,
                                      write_buf, sizeof(write_buf),
                                      pdMS_TO_TICKS(1000));
}

static esp_err_t bmp280_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BMP280_I2C_PORT, BMP280_I2C_ADDR,
                                        &reg, 1, data, len,
                                        pdMS_TO_TICKS(1000));
}

// Load calibration data from sensor
static esp_err_t bmp280_load_calibration(void)
{
    uint8_t calib[24];
    esp_err_t ret = bmp280_read_reg(BMP280_REG_CALIB_START, calib, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    calib_data.dig_P9 = (calib[23] << 8) | calib[22];

    ESP_LOGI(TAG, "Calibration data loaded");
    return ESP_OK;
}

// Compensate temperature (returns in 0.01 degrees C)
static int32_t bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) *
            ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
            ((int32_t)calib_data.dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Compensate pressure (returns in Pa)
static uint32_t bmp280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) +
           ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);

    return (uint32_t)p / 256;
}

esp_err_t bmp280_init(void)
{
    esp_err_t ret;

    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BMP280_I2C_SDA,
        .scl_io_num = BMP280_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BMP280_I2C_FREQ_HZ,
    };

    ret = i2c_param_config(BMP280_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return ret;
    }

    ret = i2c_driver_install(BMP280_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }

    // Check chip ID
    uint8_t chip_id;
    ret = bmp280_read_reg(BMP280_REG_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != 0x58) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x58)", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "BMP280 chip ID: 0x%02X", chip_id);

    // Load calibration data
    ret = bmp280_load_calibration();
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure sensor: normal mode, oversampling x16 for both temp and pressure
    ret = bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0xB7);  // osrs_t=16, osrs_p=16, mode=normal
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor");
        return ret;
    }

    // Configure: standby 0.5ms, filter off
    ret = bmp280_write_reg(BMP280_REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor");
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 initialized successfully");
    return ESP_OK;
}

esp_err_t bmp280_read_all(float *pressure, float *temperature)
{
    uint8_t data[6];
    esp_err_t ret = bmp280_read_reg(BMP280_REG_PRESS_MSB, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Parse raw data
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    // Compensate temperature first (sets t_fine)
    int32_t temp_raw = bmp280_compensate_temperature(adc_T);
    *temperature = temp_raw / 100.0f;

    // Compensate pressure
    uint32_t press_raw = bmp280_compensate_pressure(adc_P);
    *pressure = press_raw;

    return ESP_OK;
}

esp_err_t bmp280_read_pressure(float *pressure)
{
    float temp;
    return bmp280_read_all(pressure, &temp);
}

esp_err_t bmp280_read_temperature(float *temperature)
{
    float press;
    return bmp280_read_all(&press, temperature);
}
