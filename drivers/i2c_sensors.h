#ifndef I2C_SENSORS_H
#define I2C_SENSORS_H

#include "platform_defs.h"
#include "hardware_config.h"

typedef struct {
    uint8_t scl_gpio;
    uint8_t sda_gpio;
    const char *name;
    uint8_t address;
} i2c_sensor_t;

typedef struct {
    float x_g;
    float y_g;
    float z_g;
} accel_sample_t;

typedef struct {
    float x_ut;
    float y_ut;
    float z_ut;
} compass_sample_t;

esp_err_t accelerometer_init(void);
esp_err_t compass_init(void);
esp_err_t accelerometer_read(accel_sample_t *out_sample);
esp_err_t compass_read(compass_sample_t *out_sample);

#endif // I2C_SENSORS_H
