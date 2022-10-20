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
#include "iis2dlpc.h"


static uint8_t iis2dlpc_fs_scale = IIS2DLPC_CTRL6_FS_2G;
static bool iis2dlpc_ready = false;


void iis2dlpc_init_api(const struct device * iis2dlpc_dev)
{
	uint8_t id = 0;
	int b = i2c_reg_read_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_WHO_AM_I, &id);

	if (b)
	{
		printk("IIS2DLPC failed to read ID\n");
		return;
	}

	iis2dlpc_ready = (IIS2DLPC_VAL_WHO_AM_I == id);

	printk(
		"IIS2DLPC accelerometer WHO_AM_I: 0x%02x%s\n",
		id,
		(iis2dlpc_ready ? "" : "  invalid")
	);

	if (!iis2dlpc_ready)
	{
		return;
	}


	// CTRL2
	uint8_t ctrl2 = IIS2DLPC_CTRL2_BOOT | IIS2DLPC_CTRL2_SOFT_RESET;

	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL2, ctrl2);

	if (b)
	{
		printk("IIS2DLPC failed to write CTRL2\n");
		return;
	}

	int i = 4;

	do
	{
		b = i2c_reg_read_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL2, &ctrl2);

		if (b)
		{
			printk("IIS2DLPC failed to read CTRL2, %u\n", i);
			return;
		}
	} while (--i && (ctrl2 & (IIS2DLPC_CTRL2_BOOT | IIS2DLPC_CTRL2_SOFT_RESET)));

	ctrl2 = IIS2DLPC_CTRL2_BDU | IIS2DLPC_CTRL2_IF_ADD_INC;

	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL2, ctrl2);

	if (b)
	{
		printk("IIS2DLPC failed to write CTRL2\n");
		return;
	}


	// CTRL1
	uint8_t ctrl1 = IIS2DLPC_CTRL1_ODR_12_5 | IIS2DLPC_CTRL1_MODE_LP | IIS2DLPC_CTRL1_LP_MODE1;
	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL1, ctrl1);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL1\n");
		return;
	}


	// CTRL3
	uint8_t ctrl3 =
		IIS2DLPC_CTRL3_ST_DISABLED |
		IIS2DLPC_CTRL3_PP_OD_PUSH_PULL |
		IIS2DLPC_CTRL3_LIR_PULSED |
		IIS2DLPC_CTRL3_H_LACTIVE_HIGH |
		IIS2DLPC_CTRL3_SLP_MODE_SEL_INT2 |
		IIS2DLPC_CTRL3_SLP_MODE_1_INACTIVE |
		0;

	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL3, ctrl3);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL3\n");
		return;
	}


	// CTRL4
	uint8_t ctrl4 = 0;
	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL4, ctrl4);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL4\n");
		return;
	}


	// CTRL5
	uint8_t ctrl5 = 0;
	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL5, ctrl5);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL5\n");
		return;
	}


	// CTRL6
	uint8_t ctrl6 =
		IIS2DLPC_CTRL6_BW_FLT_ORD_2 |
		iis2dlpc_fs_scale |
		IIS2DLPC_CTRL6_FDS_LOW |
		IIS2DLPC_CTRL6_LOW_NOISE_DISABLED |
		0;

	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL6, ctrl6);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL6\n");
		return;
	}


	// CTRL7
	uint8_t ctrl7 = IIS2DLPC_CTRL7_DRDY_MODE_LATCHED;
	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_CTRL7, ctrl7);

	if (b)
	{
		printk("IIS2DLPC failed to read CTRL7\n");
		return;
	}


	// FIFO_CTRL
	uint8_t fifo_ctrl = IIS2DLPC_FIFO_CTRL_MODE_BYPASS | (IIS2DLPC_FIFO_CTRL_FTH_MASK & 0);
	b = i2c_reg_write_byte(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, IIS2DLPC_REG_FIFO_CTRL, fifo_ctrl);

	if (b)
	{
		printk("IIS2DLPC failed to read FIFO_CTRL\n");
		return;
	}


	// read values
	struct iis2dlpc_t iis2dlpc;
	b = iis2dlpc_read(iis2dlpc_dev, &iis2dlpc);

	if (b)
	{
		printk("\n");
		return;
	}

	printf(
		"IIS2DLPC  x: %.3f, y: %.3f, z: %.3f\n\n",
		((float)iis2dlpc.x) * 0.001,
		((float)iis2dlpc.y) * 0.001,
		((float)iis2dlpc.z) * 0.001
	);
}

int iis2dlpc_read(const struct device * iis2dlpc_dev, struct iis2dlpc_t * iis2dlpc)
{
	if (!iis2dlpc_ready)
	{
		return -1;
	}

	uint8_t reg = IIS2DLPC_REG_OUT_X_L;
	int b = i2c_write_read(iis2dlpc_dev, IIS2DLPC_I2C_ADDR, &reg, sizeof(reg), iis2dlpc->arr, sizeof(iis2dlpc->arr));

	if (b)
	{
		printk("IIS2DLPC failed to read OUT_XYZ\n");
		return -1;
	}

	uint32_t fs_scale = 2 + ((iis2dlpc_fs_scale && IIS2DLPC_CTRL6_FS_MASK) >> IIS2DLPC_CTRL6_FS_OFFS);
	iis2dlpc->x = (int16_t)((((int32_t)iis2dlpc->x << fs_scale) * 9800) >> 16);
	iis2dlpc->y = (int16_t)((((int32_t)iis2dlpc->y << fs_scale) * 9800) >> 16);
	iis2dlpc->z = (int16_t)((((int32_t)iis2dlpc->z << fs_scale) * 9800) >> 16);

	return 0;
}
