/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <inttypes.h>
#include <string.h>
#include <nrf_modem.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>
#include <drivers/eeprom.h>
#include <drivers/sensor.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/at_cmd.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <power/reboot.h>

#include <adp536x-mod.h>
#include <buzzer.h>


/****************************** SHTC3 *************************************/
#include "shtc3/shtc3.h"
/****************************** END SHTC3 *********************************/


#define SHTC3_I2C_ADDR 0x70

struct shtc3_t
{
	float temperature;
	float humidity;
};

void shtc3_init_api(const struct device * shtc3_dev);
int shtc3_read(const struct device * shtc3_dev, struct shtc3_t * shtc3);
