cmake_minimum_required(VERSION 3.16)
project(esp32s3_gnss C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(esp32s3_gnss
    main/main.c
    drivers/gps_uart.c
    drivers/i2c_sensors.c
    drivers/lcd_display.c
)

target_include_directories(esp32s3_gnss PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/main
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

target_link_libraries(esp32s3_gnss m)

install(TARGETS esp32s3_gnss RUNTIME DESTINATION bin)
