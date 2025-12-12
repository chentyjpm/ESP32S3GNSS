#include <stdio.h>
#include "platform_defs.h"
#include "hardware_config.h"
#include "gps_uart.h"
#include "i2c_sensors.h"
#include "lcd_display.h"

static const char *TAG = "app";

static esp_err_t init_peripherals(void) {
    if (gps_uart_enable_power() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable GPS power");
        return ESP_FAIL;
    }
    if (gps_uart_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init GPS UART");
        return ESP_FAIL;
    }
    if (lcd_power_on() != ESP_OK) {
        ESP_LOGE(TAG, "LCD power up failed");
        return ESP_FAIL;
    }
    if (lcd_init() != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed");
        return ESP_FAIL;
    }
    if (accelerometer_init() != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer init failed");
        return ESP_FAIL;
    }
    if (compass_init() != ESP_OK) {
        ESP_LOGE(TAG, "Compass init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t capture_sensor_snapshot(void) {
    gps_fix_t fix = {0};
    accel_sample_t accel = {0};
    compass_sample_t compass = {0};

    if (gps_uart_read_sample(&fix) != ESP_OK) {
        ESP_LOGE(TAG, "GPS sample read failed");
        return ESP_FAIL;
    }
    if (accelerometer_read(&accel) != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer sample failed");
        return ESP_FAIL;
    }
    if (compass_read(&compass) != ESP_OK) {
        ESP_LOGE(TAG, "Compass sample failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Snapshot: lat=%.6f lon=%.6f alt=%.1f m sat=%d accel=[%.2f,%.2f,%.2f]g",
             fix.latitude_deg, fix.longitude_deg, fix.altitude_m, fix.satellites,
             accel.x_g, accel.y_g, accel.z_g);
    ESP_LOGI(TAG, "Snapshot: compass heading source uT=[%.1f,%.1f,%.1f]", compass.x_ut, compass.y_ut, compass.z_ut);
    return ESP_OK;
}

int main(void) {
    ESP_LOGI(TAG, "Starting ESP32-S3 GNSS demo using ESP32-S3-WROOM-1");
    if (init_peripherals() != ESP_OK) {
        ESP_LOGE(TAG, "Peripheral initialization failed");
        return -1;
    }
    if (lcd_demo_screen() != ESP_OK) {
        ESP_LOGE(TAG, "LCD demo failed");
        return -1;
    }

    if (capture_sensor_snapshot() != ESP_OK) {
        ESP_LOGE(TAG, "Sensor snapshot failed");
        return -1;
    }

    ESP_LOGI(TAG, "System initialized. Replace stubs with ESP-IDF drivers for hardware bring-up.");
    return 0;
}
