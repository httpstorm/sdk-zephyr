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
#include "shtc3.h"



/****************************** SHTC3 *************************************/
void shtc3_init_api(const struct device * shtc3_dev)
{
	etError error;
	uint16_t id;
	float    temperature; // temperature
	float    humidity;    // relative humidity


	// initalize sensor module with the i2c address 0x70
	SHTC3_Init(SHTC3_I2C_ADDR, shtc3_dev);

	// demonstartion of SoftReset command
	error = SHTC3_SoftReset();
	printk("SHTC3_SoftReset: %d\n", error);

	// wake up the sensor from sleep mode
	SHTC3_Wakeup();

	// wait for sensor to reset
	k_usleep(100);

	// demonstartion of GetId command
	error = SHTC3_GetId(&id);
	printk("SHTC3_GetId: %d 0x%x\n", error, id);

	// read temperature and relative humidity
	error = SHTC3_GetTempAndHumiPolling(&temperature, &humidity);
	printf("GetTempAndHumoPolling: %d, %f, %f\n", error, temperature, humidity);

	// activate the sleep mode of the sensor to save energy
	SHTC3_Sleep();
}

int shtc3_read(const struct device * shtc3_dev, struct shtc3_t * shtc3)
{
	// wake up the sensor from sleep mode
	SHTC3_Wakeup();

	etError error = SHTC3_GetTempAndHumiPolling(&shtc3->temperature, &shtc3->humidity);

	// activate the sleep mode of the sensor to save energy
	SHTC3_Sleep();

	return (int)error;
}
/****************************** END SHTC3 *********************************/
