idf_component_register(
    SRCS "gps_uart.c" "i2c_sensors.c" "lcd_display.c"
    INCLUDE_DIRS "." "../../include"
    REQUIRES driver esp_driver_uart esp_driver_i2c esp_driver_spi esp_driver_gpio esp_lcd lvgl
)
