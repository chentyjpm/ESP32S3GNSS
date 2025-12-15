#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "gps_uart.h"

static const char *TAG = "gps";
static const uart_port_t GPS_UART_NUM = UART_NUM_1;

static esp_err_t ensure_uart_started(void) {
    static bool started = false;
    if (started) {
        return ESP_OK;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(GPS_UART_NUM, 2048, 0, 0, NULL, 0));
    started = true;
    return ESP_OK;
}

static double parse_coordinate(const char *field, const char *direction) {
    double value = atof(field);
    double degrees = ((int)(value / 100));
    double minutes = value - degrees * 100;
    double decimal = degrees + minutes / 60.0;
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal *= -1.0;
    }
    return decimal;
}

static esp_err_t configure_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(ensure_uart_started());
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return ESP_OK;
}

esp_err_t gps_uart_enable_power(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << GPS_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&cfg));
    //set GPS tx rx low to avoid floating
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = (1ULL << GPS_UART_TX_GPIO) | (1ULL << GPS_UART_RX_GPIO);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&cfg));

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPS_POWER_GPIO, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPS_UART_TX_GPIO, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPS_UART_RX_GPIO, 0));

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(GPS_POWER_GPIO, 1));

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for GPS to power up
    
    ESP_LOGI(TAG, "GPS power enabled on GPIO %d", GPS_POWER_GPIO);
    return ESP_OK;
}

esp_err_t gps_uart_init(void) {
    ESP_LOGI(TAG, "Configuring UART for GPS RX=%d TX=%d", GPS_UART_RX_GPIO, GPS_UART_TX_GPIO);
    return configure_uart();
}

esp_err_t gps_uart_read_sample(gps_fix_t *out_fix) {
    if (!out_fix) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(ensure_uart_started());

    uint8_t raw[256] = {0};
    int len = uart_read_bytes(GPS_UART_NUM, raw, sizeof(raw) - 1, pdMS_TO_TICKS(500));
    if (len <= 0) {
        ESP_LOGW(TAG, "No NMEA data received from GPS");
        return ESP_FAIL;
    }
    raw[len] = '\0';

    char *gga = strstr((char *)raw, "$GPGGA");
    if (!gga) {
        gga = strstr((char *)raw, "$GNGGA");
    }
    if (!gga) {
        ESP_LOGW(TAG, "No GGA sentence found in NMEA stream");
        return ESP_FAIL;
    }

    char *end = strchr(gga, '\n');
    size_t line_len = end ? (size_t)(end - gga) : strlen(gga);
    if (line_len >= 159) {
        line_len = 159;
    }

    char buffer[160];
    memcpy(buffer, gga, line_len);
    buffer[line_len] = '\0';

    const char *fields[15] = {0};
    size_t idx = 0;
    char *token = strtok(buffer, ",");
    while (token && idx < 15) {
        fields[idx++] = token;
        token = strtok(NULL, ",");
    }
    if (idx < 10) {
        ESP_LOGE(TAG, "Incomplete NMEA sample");
        return ESP_FAIL;
    }

    out_fix->latitude_deg = parse_coordinate(fields[2], fields[3]);
    out_fix->longitude_deg = parse_coordinate(fields[4], fields[5]);
    out_fix->fix_quality = atoi(fields[6]);
    out_fix->satellites = atoi(fields[7]);
    out_fix->altitude_m = atof(fields[9]);

    ESP_LOGI(TAG, "Parsed GPS fix: lat=%.6f lon=%.6f alt=%.1f m satellites=%d quality=%d",
             out_fix->latitude_deg, out_fix->longitude_deg, out_fix->altitude_m,
             out_fix->satellites, out_fix->fix_quality);
    return ESP_OK;
}
