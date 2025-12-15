#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    //    return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t capture_sensor_snapshot(gps_fix_t *fix, accel_sample_t *accel, compass_sample_t *compass) {
    if (!fix || !accel || !compass) {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps_uart_read_sample(fix) != ESP_OK) {
        ESP_LOGE(TAG, "GPS sample read failed");
        //return ESP_FAIL;
    }
    if (accelerometer_read(accel) != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer sample failed");
        //return ESP_FAIL;
    }
    if (compass_read(compass) != ESP_OK) {
        ESP_LOGE(TAG, "Compass sample failed");
        //return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Snapshot: lat=%.6f lon=%.6f alt=%.1f m sat=%d accel=[%.2f,%.2f,%.2f]g",
             fix->latitude_deg, fix->longitude_deg, fix->altitude_m, fix->satellites,
             accel->x_g, accel->y_g, accel->z_g);
    float heading_deg = compass_heading_deg(compass);
    float pitch_deg = 0.0f, roll_deg = 0.0f;
    accelerometer_tilt_deg(accel, &pitch_deg, &roll_deg);
    ESP_LOGI(TAG, "Snapshot: compass heading=%.1f° uT=[%.1f,%.1f,%.1f] pitch=%.1f° roll=%.1f°",
             heading_deg, compass->x_ut, compass->y_ut, compass->z_ut, pitch_deg, roll_deg);
    return ESP_OK;
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP32-S3 GNSS demo using ESP32-S3-WROOM-1");
    if (init_peripherals() != ESP_OK) {
        ESP_LOGE(TAG, "Peripheral initialization failed");
        return;
    }

    gps_fix_t fix = {0};
    accel_sample_t accel = {0};
    compass_sample_t compass = {0};

    while (1)
    {
        /* code */
        if (capture_sensor_snapshot(&fix, &accel, &compass) != ESP_OK) {
            ESP_LOGE(TAG, "Sensor snapshot failed");
            return;
        }

        if (lcd_show_status(&fix, &accel, &compass) != ESP_OK) {
            ESP_LOGE(TAG, "LCD demo failed");
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    

    ESP_LOGI(TAG, "System initialized. Replace stubs with ESP-IDF drivers for hardware bring-up.");
}
