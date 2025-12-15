#include <math.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
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
    .address = 0x0D,
};

static esp_err_t write_reg(const i2c_sensor_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(sensor->port, sensor->address, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t read_reg(const i2c_sensor_t *sensor, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(sensor->port, sensor->address, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t init_bus(const i2c_sensor_t *sensor) {
    if (sensor->port >= I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bus_initialized[sensor->port]) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sensor->sda_gpio,
        .scl_io_num = sensor->scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000,
        },
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_param_config(sensor->port, &conf));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_install(sensor->port, conf.mode, 0, 0, 0));
    bus_initialized[sensor->port] = true;
    ESP_LOGI(TAG, "I2C port %d configured SDA=%d SCL=%d", sensor->port, sensor->sda_gpio, sensor->scl_gpio);
    return ESP_OK;
}

static esp_err_t init_sensor(const i2c_sensor_t *sensor) {
    esp_err_t err = init_bus(sensor);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "Initializing %s at address 0x%02X", sensor->name, sensor->address);
    return ESP_OK;
}

esp_err_t accelerometer_init(void) {
    ESP_RETURN_ON_ERROR(init_sensor(&lis3dhtr), TAG, "I2C init failed");
    // 100 Hz, all axes enabled
    ESP_RETURN_ON_ERROR(write_reg(&lis3dhtr, 0x20, 0x57), TAG, "CTRL_REG1 write failed");
    // Block data update, +/-2g, high resolution
    ESP_RETURN_ON_ERROR(write_reg(&lis3dhtr, 0x23, 0x88), TAG, "CTRL_REG4 write failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "LIS3DHTR configured for 100Hz, +/-2g");
    return ESP_OK;
}

esp_err_t compass_init(void) {
    ESP_RETURN_ON_ERROR(init_sensor(&qmc6309), TAG, "I2C init failed");
    // Soft reset then continuous mode: OSR=512, RNG=2G, ODR=50Hz, continuous
    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x0B, 0x01), TAG, "QMC reset failed");
    ESP_RETURN_ON_ERROR(write_reg(&qmc6309, 0x09, 0x1D), TAG, "QMC config failed");
    vTaskDelay(pdMS_TO_TICKS(10));
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

    const float lsb_g = 0.000061f; // 61 ug/LSB @ +/-2g
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

    uint8_t reg = 0x00;
    uint8_t data[6] = {0};
    ESP_RETURN_ON_ERROR(read_reg(&qmc6309, reg, data, sizeof(data)), TAG, "Compass read failed");

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    const float scale_ut = 0.1f; // approximate uT per LSB for 2G range
    out_sample->x_ut = raw_x * scale_ut;
    out_sample->y_ut = raw_y * scale_ut;
    out_sample->z_ut = raw_z * scale_ut;

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
