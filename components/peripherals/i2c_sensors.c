#include <math.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_sensors.h"

static const char *TAG = "i2c";

typedef struct {
    i2c_port_t port;
    gpio_num_t scl_gpio;
    gpio_num_t sda_gpio;
    const char *name;
    uint8_t address;
} i2c_sensor_t;

static bool bus_initialized[I2C_NUM_MAX] = {0};
static i2c_sensor_t lis3dhtr = {
    .port = I2C_NUM_0,
    .scl_gpio = ACC_I2C_SCL_GPIO,
    .sda_gpio = ACC_I2C_SDA_GPIO,
    .name = "LIS3DHTR",
    .address = 0x18,
};

static i2c_sensor_t qmc6309 = {
    .port = I2C_NUM_1,
    .scl_gpio = QMC_I2C_SCL_GPIO,
    .sda_gpio = QMC_I2C_SDA_GPIO,
    .name = "QMC6309",
    .address = 0x7c,
};

static esp_err_t ensure_bus(i2c_sensor_t *sensor) {
    if (bus_initialized[sensor->port]) {
        return ESP_OK;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sensor->sda_gpio,
        .scl_io_num = sensor->scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(sensor->port, &cfg), TAG, "I2C param config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(sensor->port, cfg.mode, 0, 0, 0), TAG, "I2C driver install failed");
    bus_initialized[sensor->port] = true;
    ESP_LOGI(TAG, "I2C port %d configured SDA=%d SCL=%d", sensor->port, sensor->sda_gpio, sensor->scl_gpio);
    return ESP_OK;
}

static esp_err_t ensure_device(i2c_sensor_t *sensor, const uint8_t *candidate_addresses, size_t num_candidates) {
    ESP_RETURN_ON_ERROR(ensure_bus(sensor), TAG, "I2C bus init failed");

    uint8_t tmp = 0;
    for (size_t i = 0; i < num_candidates; ++i) {
        sensor->address = candidate_addresses[i];
        esp_err_t ping = i2c_master_write_read_device(sensor->port, sensor->address, &tmp, 0, &tmp, 1, pdMS_TO_TICKS(50));
        if (ping == ESP_OK) {
            ESP_LOGI(TAG, "%s detected at 0x%02X", sensor->name, sensor->address);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "%s not detected on I2C bus", sensor->name);
    return ESP_FAIL;
}

static esp_err_t write_reg(const i2c_sensor_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(sensor->port, sensor->address, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t read_reg(const i2c_sensor_t *sensor, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(sensor->port, sensor->address, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t accelerometer_init(void) {
    const uint8_t candidate_addresses[] = {0x18, 0x19};
    ESP_RETURN_ON_ERROR(ensure_device(&lis3dhtr, candidate_addresses, sizeof(candidate_addresses) / sizeof(candidate_addresses[0])), TAG, "I2C init failed");
    ESP_LOGI(TAG, "Initializing %s at address 0x%02X", lis3dhtr.name, lis3dhtr.address);

    uint8_t who_am_i = 0;
    ESP_RETURN_ON_ERROR(read_reg(&lis3dhtr, 0x0F, &who_am_i, 1), TAG, "WHO_AM_I read failed");
    if (who_am_i != 0x33) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x33)", who_am_i);
    }
    // 100 Hz, all axes enabled
    ESP_RETURN_ON_ERROR(write_reg(&lis3dhtr, 0x20, 0x57), TAG, "CTRL_REG1 write failed");
    // Block data update, +/-2g, high resolution
    ESP_RETURN_ON_ERROR(write_reg(&lis3dhtr, 0x23, 0x88), TAG, "CTRL_REG4 write failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "LIS3DHTR configured for 100Hz, +/-2g");
    return ESP_OK;
}

esp_err_t compass_init(void) {
    const uint8_t candidate_addresses[] = {0x0c, 0x1c, 0x2c, 0x3c, 0x4c, 0x5c, 0x6c, 0x7c};
    ESP_RETURN_ON_ERROR(ensure_bus(&qmc6309), TAG, "I2C bus init failed");

    uint8_t chip_id = 0x00;
    bool found = false;
    for (size_t i = 0; i < sizeof(candidate_addresses) / sizeof(candidate_addresses[0]); ++i) {
        qmc6309.address = candidate_addresses[i];
        if (read_reg(&qmc6309, 0x00, &chip_id, 1) == ESP_OK && chip_id == 0x90) {
            found = true;
            ESP_LOGI(TAG, "%s detected at 0x%02X (chip_id=0x%02X)", qmc6309.name, qmc6309.address, chip_id);
            break;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "%s not detected on I2C bus (chip_id=0x%02X)", qmc6309.name, chip_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Initializing %s at address 0x%02X", qmc6309.name, qmc6309.address);
    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x0B, 0x80), TAG, "QMC soft reset");
    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x0B, 0x00), TAG, "QMC soft reset release");
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t status = 0x00;
    for (int i = 0; i < 5; ++i) {
        if (read_reg(&qmc6309, 0x09, &status, 1) == ESP_OK && (status & 0x10)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (qmc6309.address == 0x0c) {
        uint8_t reg40 = 0x00;
        if (read_reg(&qmc6309, 0x40, &reg40, 1) == ESP_OK) {
            reg40 = (reg40 & 0xe0) | 0x0d;
            write_reg(&qmc6309, 0x40, reg40);
        }
    }

    const uint8_t mode_hpfm = 0x03;
    const uint8_t osr1_8 = 0x00;
    const uint8_t osr2_4 = 0x02;
    const uint8_t zdbl_enb = (qmc6309.address == 0x0c) ? 0x00 : 0x01;
    const uint8_t ctrl1 = (mode_hpfm) | (zdbl_enb << 2) | (osr1_8 << 3) | (osr2_4 << 5);
    const uint8_t ctrl2 = 0x00; // set/reset on, 32G, ODR HPFM

    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x0B, ctrl2), TAG, "QMC config ctrl2 failed");
    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x0A, ctrl1), TAG, "QMC config ctrl1 failed");
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "QMC6309 configured for continuous heading updates");
    return ESP_OK;
}

esp_err_t accelerometer_read(accel_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = 0x28 | 0x80; // auto-increment
    uint8_t data[6] = {0};
    ESP_RETURN_ON_ERROR(read_reg(&lis3dhtr, reg, data, sizeof(data)), TAG, "Accel read failed");

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);
    raw_x >>= 4;
    raw_y >>= 4;
    raw_z >>= 4;

    // In high-resolution mode (+/-2g, 0x23 = 0x88) sensitivity is 1 mg/LSB.
    const float lsb_g = 0.001f;
    out_sample->x_g = raw_x * lsb_g;
    out_sample->y_g = raw_y * lsb_g;
    out_sample->z_g = raw_z * lsb_g;

    ESP_LOGI(TAG, "LIS3DHTR sample g=[%.3f, %.3f, %.3f]", out_sample->x_g, out_sample->y_g, out_sample->z_g);
    return ESP_OK;
}

esp_err_t compass_read(compass_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_ERR_INVALID_ARG;
    }

    static compass_sample_t last_sample = {0};
    uint8_t status = 0x00;
    uint8_t data[6] = {0};
    for (int i = 0; i < 5; ++i) {
        if (read_reg(&qmc6309, 0x09, &status, 1) == ESP_OK && (status & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!(status & 0x01)) {
        *out_sample = last_sample;
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(read_reg(&qmc6309, 0x01, data, sizeof(data)), TAG, "Compass read failed");

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    const float scale_ut = 0.1f; // 32G range -> 1000 LSB per 100 uT
    out_sample->x_ut = raw_x * scale_ut;
    out_sample->y_ut = raw_y * scale_ut;
    out_sample->z_ut = raw_z * scale_ut;
    last_sample = *out_sample;

    const float pi = 3.14159265358979323846f;
    float heading = atan2f(out_sample->y_ut, out_sample->x_ut) * 180.0f / pi;
    if (heading < 0) {
        heading += 360.0f;
    }
    ESP_LOGI(TAG, "QMC6309 sample uT=[%.1f, %.1f, %.1f] heading=%.1f deg", out_sample->x_ut, out_sample->y_ut, out_sample->z_ut, heading);
    return ESP_OK;
}

void accelerometer_tilt_deg(const accel_sample_t *accel, float *pitch_deg, float *roll_deg) {
    if (!accel || !pitch_deg || !roll_deg) {
        return;
    }
    const float pi = 3.14159265358979323846f;
    *roll_deg = atan2f(accel->y_g, accel->z_g) * 180.0f / pi;
    float denom = sqrtf(accel->y_g * accel->y_g + accel->z_g * accel->z_g);
    *pitch_deg = atan2f(-accel->x_g, denom) * 180.0f / pi;
}

float compass_heading_deg(const compass_sample_t *compass) {
    if (!compass) {
        return NAN;
    }
    const float pi = 3.14159265358979323846f;
    float heading = atan2f(compass->y_ut, compass->x_ut) * 180.0f / pi;
    return heading < 0 ? heading + 360.0f : heading;
}
