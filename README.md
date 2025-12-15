# ESP32S3GNSS

ESP-IDF project for ESP32-S3-WROOM-1 that wires a UART GNSS receiver (MC280M), an SPI LCD (ST7789 driven with LVGL), and two I2C sensors (LIS3DHTR accelerometer and QMC6309 compass).

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

## Build (ESP-IDF)

1. Fetch and install the ESP-IDF toolchain for ESP32-S3 (once):
   ```bash
   git clone --depth 1 https://github.com/espressif/esp-idf.git
   ./esp-idf/install.sh esp32s3
   ```
2. Export the environment for this shell:
   ```bash
   source ./esp-idf/export.sh
   ```
3. Configure the project target and build:
   ```bash
   idf.py set-target esp32s3
   idf.py build
   ```

   You can also run the helper script to perform the same build after exporting ESP-IDF:
   ```bash
   ./tools/build.sh
   ```

The `components/peripherals` component initializes GPIO, UART, SPI, and I2C for the mapped peripherals and now drives the LCD with an LVGL status screen that shows the latest GPS fix (latitude/longitude/altitude), compass heading with a cardinal direction, and accelerometer g-values plus computed pitch/roll tilt angles. Sensor reads come from live UART/I2C transactions so values should update as soon as the hardware is connected and powered.
