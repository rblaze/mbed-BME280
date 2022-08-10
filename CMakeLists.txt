add_library(mbed-BME280 INTERFACE)

target_include_directories(mbed-BME280
    INTERFACE
        .
)

target_sources(mbed-BME280
    INTERFACE
        bme280_driver.c
        BME280.cpp
)
