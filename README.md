# ESP32S3GNSS

Standalone CMake project that documents the wiring and initialization sequence for an ESP32-S3-WROOM-1 board paired with GNSS, LCD, accelerometer, and compass peripherals. The code builds a host-side executable that mirrors the intended ESP-IDF initialization flow so it can be compiled and exercised inside this container.

## Hardware mapping

- **GPS (MC280M, UART/NMEA)**
  - RX: IO17
  - TX: IO18
  - Power enable: IO41
- **LCD (ST7789 over SPI, LVGL expected)**
  - MOSI (SDO): IO5
  - SCK: IO4
  - CS: IO15
  - D/C: IO7
  - Reset: IO6
  - Power enable: IO41
- **Accelerometer (LIS3DHTR over I2C)**
  - SDA: IO9
  - SCL: IO10
- **Compass (QMC6309 over I2C)**
  - SDA: IO12
  - SCL: IO11

## Building

```bash
cmake -S . -B build
cmake --build build
./build/esp32s3_gnss
```

The resulting executable logs a full bring-up sequence and prints simulated readings:
- GPS NMEA decode from an MC280M GPGGA sentence, including latitude/longitude, fix quality, and altitude.
- LIS3DHTR acceleration vector representing a stationary board (~1g on Z).
- QMC6309 magnetic field and derived heading.
- Placeholder LVGL screen initialization for the ST7789 display.

Use these outputs to sanity check wiring definitions before replacing the stub logic with ESP-IDF drivers on hardware.
