#ifndef PLATFORM_DEFS_H
#define PLATFORM_DEFS_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1

#define ESP_LOGI(tag, fmt, ...) printf("[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR] %s: " fmt "\n", tag, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_DEFS_H
