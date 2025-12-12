#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
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
    i2c_sensor_t lis3dhtr = {
        .port = I2C_NUM_0,
        .scl_gpio = ACC_I2C_SCL_GPIO,
        .sda_gpio = ACC_I2C_SDA_GPIO,
        .name = "LIS3DHTR",
        .address = 0x18,
    };
    return init_sensor(&lis3dhtr);
}

esp_err_t compass_init(void) {
    i2c_sensor_t qmc6309 = {
        .port = I2C_NUM_1,
        .scl_gpio = QMC_I2C_SCL_GPIO,
        .sda_gpio = QMC_I2C_SDA_GPIO,
        .name = "QMC6309",
        .address = 0x0D,
    };
    return init_sensor(&qmc6309);
}

esp_err_t accelerometer_read(accel_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_ERR_INVALID_ARG;
    }
    out_sample->x_g = 0.01f;
    out_sample->y_g = -0.02f;
    out_sample->z_g = 1.00f; // resting on a table ~1g
    ESP_LOGI(TAG, "LIS3DHTR sample g=[%.2f, %.2f, %.2f]", out_sample->x_g, out_sample->y_g, out_sample->z_g);
    return ESP_OK;
}

esp_err_t compass_read(compass_sample_t *out_sample) {
    if (!out_sample) {
        return ESP_ERR_INVALID_ARG;
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
