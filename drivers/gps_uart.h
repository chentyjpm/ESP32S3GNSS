#ifndef GPS_UART_H
#define GPS_UART_H

#include "platform_defs.h"
#include "hardware_config.h"

typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    int fix_quality;
    int satellites;
} gps_fix_t;

esp_err_t gps_uart_init(void);
esp_err_t gps_uart_enable_power(void);
esp_err_t gps_uart_read_sample(gps_fix_t *out_fix);

#endif // GPS_UART_H
