#include "mpu6500.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MPU6500";

// MPU6500 Registers
#define MPU6500_REG_WHO_AM_I       0x75
#define MPU6500_REG_PWR_MGMT_1     0x6B
#define MPU6500_REG_PWR_MGMT_2     0x6C
#define MPU6500_REG_CONFIG         0x1A
#define MPU6500_REG_GYRO_CONFIG    0x1B
#define MPU6500_REG_ACCEL_CONFIG   0x1C
#define MPU6500_REG_ACCEL_CONFIG_2 0x1D
#define MPU6500_REG_ACCEL_XOUT_H   0x3B
#define MPU6500_REG_GYRO_XOUT_H    0x43

#define MPU6500_WHO_AM_I_VALUE     0x70

// Gyro and accel scale factors
#define GYRO_SCALE_FACTOR  (2000.0f / 32768.0f)   // ±2000 deg/s
#define ACCEL_SCALE_FACTOR (16.0f / 32768.0f)     // ±16g

static spi_device_handle_t spi_handle;

// SPI read/write helpers
static esp_err_t mpu6500_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg & 0x7F, data};  // Clear MSB for write
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(spi_handle, &trans);
}

static esp_err_t mpu6500_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_data = reg | 0x80;  // Set MSB for read
    spi_transaction_t trans = {
        .length = 8,
        .rxlength = len * 8,
        .tx_buffer = &tx_data,
        .rx_buffer = data,
    };
    return spi_device_transmit(spi_handle, &trans);
}

static esp_err_t mpu6500_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret;
    uint8_t tx_buf[1 + len];
    uint8_t rx_buf[1 + len];

    memset(tx_buf, 0xFF, sizeof(tx_buf));
    tx_buf[0] = reg | 0x80;  // Set MSB for read

    spi_transaction_t trans = {
        .length = (1 + len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    ret = spi_device_transmit(spi_handle, &trans);
    if (ret == ESP_OK) {
        memcpy(data, &rx_buf[1], len);  // Skip first byte (dummy)
    }

    return ret;
}

esp_err_t mpu6500_init(void)
{
    esp_err_t ret;

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MPU6500_PIN_MOSI,
        .miso_io_num = MPU6500_PIN_MISO,
        .sclk_io_num = MPU6500_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    ret = spi_bus_initialize(MPU6500_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = MPU6500_SPI_FREQ_HZ,
        .mode = 0,  // SPI mode 0
        .spics_io_num = MPU6500_PIN_CS,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(MPU6500_SPI_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    // Small delay for sensor to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check WHO_AM_I register
    uint8_t who_am_i;
    ret = mpu6500_read_bytes(MPU6500_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I");
        return ret;
    }

    if (who_am_i != MPU6500_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, MPU6500_WHO_AM_I_VALUE);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU6500 WHO_AM_I: 0x%02X", who_am_i);

    // Reset device
    ret = mpu6500_write_reg(MPU6500_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Wake up device (clear sleep bit)
    ret = mpu6500_write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);  // Auto select clock
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable all sensors
    ret = mpu6500_write_reg(MPU6500_REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable sensors");
        return ret;
    }

    // Configure gyroscope: ±2000 deg/s
    ret = mpu6500_write_reg(MPU6500_REG_GYRO_CONFIG, 0x18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyro");
        return ret;
    }

    // Configure accelerometer: ±16g
    ret = mpu6500_write_reg(MPU6500_REG_ACCEL_CONFIG, 0x18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accel");
        return ret;
    }

    // Configure DLPF (Digital Low Pass Filter)
    // DLPF_CFG = 0: Gyro BW 250Hz, Accel BW 460Hz (low latency for fast projectile)
    ret = mpu6500_write_reg(MPU6500_REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }

    // Configure accelerometer DLPF
    ret = mpu6500_write_reg(MPU6500_REG_ACCEL_CONFIG_2, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accel DLPF");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6500 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6500_read_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    uint8_t data[6];
    esp_err_t ret = mpu6500_read_bytes(MPU6500_REG_GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyro data");
        return ret;
    }

    // Parse raw data (16-bit signed)
    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    // Convert to deg/s
    *gyro_x = raw_x * GYRO_SCALE_FACTOR;
    *gyro_y = raw_y * GYRO_SCALE_FACTOR;
    *gyro_z = raw_z * GYRO_SCALE_FACTOR;

    return ESP_OK;
}

esp_err_t mpu6500_read_accel(float *accel_x, float *accel_y, float *accel_z)
{
    uint8_t data[6];
    esp_err_t ret = mpu6500_read_bytes(MPU6500_REG_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accel data");
        return ret;
    }

    // Parse raw data (16-bit signed)
    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    // Convert to g
    *accel_x = raw_x * ACCEL_SCALE_FACTOR;
    *accel_y = raw_y * ACCEL_SCALE_FACTOR;
    *accel_z = raw_z * ACCEL_SCALE_FACTOR;

    return ESP_OK;
}

esp_err_t mpu6500_read_all(float *gyro_x, float *gyro_y, float *gyro_z,
                           float *accel_x, float *accel_y, float *accel_z)
{
    // Read all 14 bytes (accel + temp + gyro) starting from ACCEL_XOUT_H
    uint8_t data[14];
    esp_err_t ret = mpu6500_read_bytes(MPU6500_REG_ACCEL_XOUT_H, data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Parse accelerometer (first 6 bytes)
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

    // Skip temperature (bytes 6-7)

    // Parse gyroscope (bytes 8-13)
    int16_t raw_gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t raw_gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);

    // Convert to physical units
    *accel_x = raw_ax * ACCEL_SCALE_FACTOR;
    *accel_y = raw_ay * ACCEL_SCALE_FACTOR;
    *accel_z = raw_az * ACCEL_SCALE_FACTOR;
    *gyro_x = raw_gx * GYRO_SCALE_FACTOR;
    *gyro_y = raw_gy * GYRO_SCALE_FACTOR;
    *gyro_z = raw_gz * GYRO_SCALE_FACTOR;

    return ESP_OK;
}
