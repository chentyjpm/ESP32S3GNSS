#include "lcd_display.h"

static const char *TAG = "lcd";

esp_err_t lcd_power_on(void) {
    ESP_LOGI(TAG, "Enabling LCD power on GPIO %d", LCD_POWER_GPIO);
    return ESP_OK;
}

esp_err_t lcd_init(void) {
    ESP_LOGI(TAG, "Initializing ST7789 over SPI MOSI=%d CLK=%d CS=%d DC=%d RST=%d", LCD_SPI_MOSI_GPIO, LCD_SPI_CLK_GPIO, LCD_SPI_CS_GPIO, LCD_SPI_DC_GPIO, LCD_SPI_RST_GPIO);
    ESP_LOGI(TAG, "Binding LVGL display driver with default resolution 240x240");
    return ESP_OK;
}

esp_err_t lcd_demo_screen(void) {
    ESP_LOGI(TAG, "Drawing LVGL demo widgets (placeholder)");
    return ESP_OK;
}
