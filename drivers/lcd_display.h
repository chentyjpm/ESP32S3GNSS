#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include "platform_defs.h"
#include "hardware_config.h"

esp_err_t lcd_power_on(void);
esp_err_t lcd_init(void);
esp_err_t lcd_demo_screen(void);

#endif // LCD_DISPLAY_H
