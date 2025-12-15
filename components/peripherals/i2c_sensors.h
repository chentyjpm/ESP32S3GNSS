#ifndef I2C_SENSORS_H
#define I2C_SENSORS_H

#include "esp_err.h"
#include "esp_log.h"
#include "hardware_config.h"

#ifdef __cplusplus
extern "C" {
#endif

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
esp_err_t accelerometer_read(accel_sample_t *out_sample);

void accelerometer_tilt_deg(const accel_sample_t *accel, float *pitch_deg, float *roll_deg);

esp_err_t compass_init(void);
esp_err_t compass_read(compass_sample_t *out_sample);
float compass_heading_deg(const compass_sample_t *compass);

#ifdef __cplusplus
}
#endif

#endif // I2C_SENSORS_H
