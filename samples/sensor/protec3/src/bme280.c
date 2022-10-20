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


 /********************* BME280 ******************************************/
#include "BME280_driver-bme280_v3.5.0/bme280.h"
/********************** END BME *****************************************/


/********************** BME280 ******************************************/
typedef struct
{
	const struct device * i2c2_dev;
	uint16_t dev_addr;
} tDevDataForBME;

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t * reg_data, uint32_t len, void * intf_ptr)
{
	tDevDataForBME * devData = (tDevDataForBME *)intf_ptr;

	return i2c_write_read(
		devData->i2c2_dev,
		devData->dev_addr,
		&reg_addr,
		sizeof(reg_addr),
		reg_data,
		len
	);
}

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t * reg_data, uint32_t len, void * intf_ptr)
{
	tDevDataForBME * devData = (tDevDataForBME *)intf_ptr;

	return i2c_burst_write(
		devData->i2c2_dev,
		devData->dev_addr,
		reg_addr,
		reg_data,
		len
	);
}

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
static void bme280_delay(uint32_t period, void * intf_ptr)
{
	k_usleep(period);
}

void bme280_print_sensor_data(struct bme280_data * comp_data)
{
#ifdef BME280_FLOAT_ENABLE
	printf("%0.2f, %0.2f, %0.2f\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
	printf("%ld, %ld, %ld\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void bme280_init_api(const struct device * i2c_dev)
{
	struct bme280_dev dev;
	int8_t rslt = BME280_OK;
	tDevDataForBME devDataForBME = { i2c_dev, BME280_I2C_ADDR_SEC };

	dev.intf_ptr = &devDataForBME;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280_i2c_read;
	dev.write = bme280_i2c_write;
	dev.delay_us = bme280_delay;
	rslt = bme280_init(&dev);

	if (BME280_OK == rslt)
	{
		uint8_t settings_sel;
		struct bme280_data comp_data;

		/* Recommended mode of operation: Indoor navigation */
		dev.settings.osr_h = BME280_OVERSAMPLING_1X;
		dev.settings.osr_p = BME280_OVERSAMPLING_16X;
		dev.settings.osr_t = BME280_OVERSAMPLING_2X;
		dev.settings.filter = BME280_FILTER_COEFF_16;
		dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

		settings_sel = BME280_OSR_PRESS_SEL;
		settings_sel |= BME280_OSR_TEMP_SEL;
		settings_sel |= BME280_OSR_HUM_SEL;
		settings_sel |= BME280_STANDBY_SEL;
		settings_sel |= BME280_FILTER_SEL;

		rslt = bme280_set_sensor_settings(settings_sel, &dev);
		rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
		printf("BME280: Temperature, Pressure, Humidity\r\n");

		dev.delay_us(70, dev.intf_ptr);

		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		bme280_print_sensor_data(&comp_data);
	}
	else
	{
		printk("bme280_init_api() failed %d\n", rslt);
	}
}

/********************** END BME *****************************************/
