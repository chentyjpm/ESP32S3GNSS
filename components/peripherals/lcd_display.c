#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "lcd_display.h"

static const char *TAG = "lcd";
static esp_lcd_panel_handle_t panel_handle;

esp_err_t lcd_power_on(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << LCD_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&cfg));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LCD_POWER_GPIO, 1));
    ESP_LOGI(TAG, "LCD power enabled on GPIO %d", LCD_POWER_GPIO);
    return ESP_OK;
}

esp_err_t lcd_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = LCD_SPI_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 240 * 240 * 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_SPI_DC_GPIO,
        .cs_gpio_num = LCD_SPI_CS_GPIO,
        .pclk_hz = 40000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_new_panel_io_spi((spi_host_device_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_SPI_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    if (panel_handle) {
        esp_lcd_panel_reset(panel_handle);
        esp_lcd_panel_init(panel_handle);
        esp_lcd_panel_mirror(panel_handle, false, true);
        ESP_LOGI(TAG, "ST7789 panel initialized over SPI");
    }

    lv_init();
    ESP_LOGI(TAG, "LVGL initialized; bind to panel using LVGL port helpers when running on hardware.");
    return ESP_OK;
}

esp_err_t lcd_demo_screen(void) {
    ESP_LOGI(TAG, "LCD demo placeholder; add LVGL widgets once display driver is bound.");
    return ESP_OK;
}
