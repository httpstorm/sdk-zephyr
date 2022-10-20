# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(protect3_test)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/buzzer.c)
target_sources(app PRIVATE src/lte_link.c)
target_sources(app PRIVATE src/notify_sms.c)
target_sources(app PRIVATE src/fota_update.c)
target_sources(app PRIVATE src/mqtt_handler.c)
target_sources(app PRIVATE src/apn.c)
target_sources(app PRIVATE src/aws.c)
target_sources(app PRIVATE src/uart.c)
target_sources(app PRIVATE src/gpio_t.c)
target_sources(app PRIVATE src/iis2dlpc.c)
target_sources(app PRIVATE src/bme280.c)
target_sources(app PRIVATE src/BME280_driver-bme280_v3.5.0/bme280.c)
target_sources(app PRIVATE src/shtc3.c)
target_sources(app PRIVATE src/shtc3/shtc3.c)

zephyr_include_directories(src)
