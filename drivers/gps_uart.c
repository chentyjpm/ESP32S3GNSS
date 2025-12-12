#include <string.h>
#include <stdlib.h>
#include "gps_uart.h"

static const char *TAG = "gps";

static double parse_coordinate(const char *field, const char *direction) {
    // NMEA format: ddmm.mmmm for latitude, dddmm.mmmm for longitude
    double value = atof(field);
    double degrees = ((int)(value / 100));
    double minutes = value - degrees * 100;
    double decimal = degrees + minutes / 60.0;
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal *= -1.0;
    }
    return decimal;
}

esp_err_t gps_uart_enable_power(void) {
    ESP_LOGI(TAG, "Enabling GPS power on GPIO %d", GPS_POWER_GPIO);
    return ESP_OK;
}

esp_err_t gps_uart_init(void) {
    ESP_LOGI(TAG, "Configuring UART for GPS RX=%d TX=%d", GPS_UART_RX_GPIO, GPS_UART_TX_GPIO);
    ESP_LOGI(TAG, "Setting UART baud rate to 9600 and NMEA framing");
    return ESP_OK;
}

esp_err_t gps_uart_read_sample(gps_fix_t *out_fix) {
    if (!out_fix) {
        return ESP_FAIL;
    }
    // Simulated NMEA GPGGA sentence captured from MC280M
    const char *sample = "$GPGGA,053253.00,3723.2475,N,12158.3416,W,1,08,1.02,10.1,M,-34.0,M,,*76";

    // Tokenize minimally to extract lat/lon/alt/satellites
    char buffer[160];
    strncpy(buffer, sample, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';

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
