#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "esp_err.h"
#include "esp_log.h"
#include "hardware_config.h"
#include "lvgl.h"
#include "gps_uart.h"
#include "i2c_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lcd_power_on(void);
esp_err_t lcd_init(void);
esp_err_t lcd_show_status(const gps_fix_t *fix, const accel_sample_t *accel, const compass_sample_t *compass);

#ifdef __cplusplus
}
#endif

#endif // LCD_DISPLAY_H
