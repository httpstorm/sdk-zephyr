#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8.2)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(protec3_mqtt_poc)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/buzzer.c)
target_sources(app PRIVATE src/uart.c)
target_sources(app PRIVATE src/apn.c)
target_sources(app PRIVATE src/fota_update.c)
target_sources(app PRIVATE src/drivers/shtc3.c)
target_sources(app PRIVATE src/drivers/iis2dlpc_reg.c)

zephyr_include_directories(src/include)
