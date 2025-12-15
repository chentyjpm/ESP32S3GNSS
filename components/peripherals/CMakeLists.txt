idf_component_register(
    SRCS "gps_uart.c" "i2c_sensors.c" "lcd_display.c"
    INCLUDE_DIRS "." "../../include"
    # Use the unified ESP-IDF v5.1 driver component to keep compatibility with
    # the toolchain installed in this environment.
    REQUIRES driver esp_lcd lvgl
)
