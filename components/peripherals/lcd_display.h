#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "esp_err.h"
#include "esp_log.h"
#include "hardware_config.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lcd_power_on(void);
esp_err_t lcd_init(void);
esp_err_t lcd_demo_screen(void);

#ifdef __cplusplus
}
#endif

#endif // LCD_DISPLAY_H
