#include <math.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_display.h"

#ifndef LCD_OFFSET_X
#define LCD_OFFSET_X 0
#endif
#ifndef LCD_OFFSET_Y
#define LCD_OFFSET_Y 80   // Many 240x240 ST7789 modules map visible rows starting at RAM row 80
#endif

static const char *TAG = "lcd";
static esp_lcd_panel_handle_t panel_handle;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t buf1[240 * 40];
static esp_timer_handle_t lvgl_tick_timer;

static void lv_tick_cb(void *arg) {
    lv_tick_inc(2);
}

static bool lcd_color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_drv_t *drv = (lv_disp_drv_t *)user_ctx;
    if (drv) {
        lv_disp_flush_ready(drv);
    }
    return false;
}

static void lcd_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)disp->user_data;
    if (!panel) {
        lv_disp_flush_ready(disp);
        return;
    }
    // Flush completion is signaled from lcd_color_trans_done_cb when DMA finishes.
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

static void pump_lvgl(uint32_t duration_ms) {
    uint32_t elapsed = 0;
    const uint32_t step = 10;
    while (elapsed < duration_ms) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
    }
}

esp_err_t lcd_power_on(void) {
    ESP_LOGI(TAG, "LCD power enabled on GPIO");
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
        .on_color_trans_done = lcd_color_trans_done_cb,
        .user_ctx = &disp_drv,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_SPI_RST_GPIO,
        // Many ST7789 panels on ESP32-S3 kits are wired BGR; swap to get correct colors.
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    if (panel_handle) {
        esp_lcd_panel_reset(panel_handle);
        esp_lcd_panel_init(panel_handle);
        esp_lcd_panel_set_gap(panel_handle, LCD_OFFSET_X, LCD_OFFSET_Y);
        esp_lcd_panel_mirror(panel_handle, false, true);
        // Invert colors to match ST7789 default panel mode (avoids washed-out white screen).
        esp_lcd_panel_invert_color(panel_handle, true);
        esp_lcd_panel_disp_on_off(panel_handle, true);
        ESP_LOGI(TAG, "ST7789 panel initialized over SPI");
    }

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, sizeof(buf1) / sizeof(lv_color_t));
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 240;
    disp_drv.flush_cb = lcd_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name = "lv_tick",
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&tick_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    ESP_LOGI(TAG, "LVGL initialized and bound to ST7789");
    return ESP_OK;
}

esp_err_t lcd_show_status(const gps_fix_t *fix, const accel_sample_t *accel, const compass_sample_t *compass) {
    if (!panel_handle || !fix || !accel || !compass) {
        return ESP_ERR_INVALID_STATE;
    }

    float heading = compass_heading_deg(compass);
    float pitch_deg = 0.0f, roll_deg = 0.0f;
    accelerometer_tilt_deg(accel, &pitch_deg, &roll_deg);
    const char *cardinal = "N";
    if (heading >= 22.5f && heading < 67.5f) {
        cardinal = "NE";
    } else if (heading >= 67.5f && heading < 112.5f) {
        cardinal = "E";
    } else if (heading >= 112.5f && heading < 157.5f) {
        cardinal = "SE";
    } else if (heading >= 157.5f && heading < 202.5f) {
        cardinal = "S";
    } else if (heading >= 202.5f && heading < 247.5f) {
        cardinal = "SW";
    } else if (heading >= 247.5f && heading < 292.5f) {
        cardinal = "W";
    } else if (heading >= 292.5f && heading < 337.5f) {
        cardinal = "NW";
    }

    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "ESP32-S3 GNSS Demo");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ffaa), 0);
    lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *gps_label = lv_label_create(screen);
    lv_label_set_text_fmt(gps_label, "GNSS Fix:\nLat %.5f°\nLon %.5f°\nAlt: %.1fm Sats:%d Q:%d",
                          fix->latitude_deg, fix->longitude_deg, fix->altitude_m,
                          fix->satellites, fix->fix_quality);
    lv_obj_align(gps_label, LV_ALIGN_LEFT_MID, 6, -56);

    lv_obj_t *acc_label = lv_label_create(screen);
    lv_label_set_text_fmt(acc_label, "ACC g:\nX:%+.2f Y:%+.2f Z:%+.2f\nPitch:%+.1f° Roll:%+.1f°",
                          accel->x_g, accel->y_g, accel->z_g, pitch_deg, roll_deg);
    lv_obj_align(acc_label, LV_ALIGN_LEFT_MID, 6, 40);

    lv_obj_t *comp_label = lv_label_create(screen);
    lv_label_set_text_fmt(comp_label, "Compass uT:\nX:%+.1f Y:%+.1f Z:%+.1f\nHeading: %.1f° %s",
                          compass->x_ut, compass->y_ut, compass->z_ut, heading, cardinal);
    lv_obj_align(comp_label, LV_ALIGN_RIGHT_MID, -6, 6);

    pump_lvgl(200);
    return ESP_OK;
}
