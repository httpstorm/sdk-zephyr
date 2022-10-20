# Copyright (c) 2019 Nordic Semiconductor ASA.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(nrf52840_reset.c)
zephyr_library_sources(uart_ble.c)

if(CONFIG_TRACING_TEST)
  zephyr_library_sources(tracing_string_format_test.c)
endif()
