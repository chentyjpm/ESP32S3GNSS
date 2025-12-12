#include <math.h>
#include "i2c_sensors.h"

static const char *TAG = "i2c";

static esp_err_t init_sensor(const i2c_sensor_t *sensor) {
    ESP_LOGI(TAG, "Initializing %s on SDA=%d SCL=%d address=0x%02X", sensor->name, sensor->sda_gpio, sensor->scl_gpio, sensor->address);
    return ESP_OK;
}

esp_err_t accelerometer_init(void) {
    i2c_sensor_t lis3dhtr = {
        .scl_gpio = ACC_I2C_SCL_GPIO,
        .sda_gpio = ACC_I2C_SDA_GPIO,
        .name = "LIS3DHTR",
        .address = 0x18,
    };
    return init_sensor(&lis3dhtr);
}

esp_err_t compass_init(void) {
    i2c_sensor_t qmc6309 = {
        .scl_gpio = QMC_I2C_SCL_GPIO,
        .sda_gpio = QMC_I2C_SDA_GPIO,
        .name = "QMC6309",
        .address = 0x0D,
    };
    return init_sensor(&qmc6309);
}

esp_err_t accelerometer_read(accel_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_FAIL;
    }
    out_sample->x_g = 0.01f;
    out_sample->y_g = -0.02f;
    out_sample->z_g = 1.00f; // resting on a table ~1g
    ESP_LOGI(TAG, "LIS3DHTR sample g=[%.2f, %.2f, %.2f]", out_sample->x_g, out_sample->y_g, out_sample->z_g);
    return ESP_OK;
}

esp_err_t compass_read(compass_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_FAIL;
    }
    out_sample->x_ut = 22.4f;
    out_sample->y_ut = -3.8f;
    out_sample->z_ut = -41.2f;
    const float pi = 3.14159265358979323846f;
    float heading = atan2f(out_sample->y_ut, out_sample->x_ut) * 180.0f / pi;
    if (heading < 0) {
        heading += 360.0f;
    }
    ESP_LOGI(TAG, "QMC6309 sample uT=[%.1f, %.1f, %.1f] heading=%.1f deg", out_sample->x_ut, out_sample->y_ut, out_sample->z_ut, heading);
    return ESP_OK;
}
