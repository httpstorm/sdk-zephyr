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


#define IIS2DLPC_I2C_ADDR 0x18
#define IIS2DLPC_VAL_WHO_AM_I 0x44

enum IIS2DLPC_REG
{
	IIS2DLPC_REG_OUT_T_L = 0x0d,
	IIS2DLPC_REG_OUT_T_H = 0x0e,
	IIS2DLPC_REG_WHO_AM_I = 0x0f,
	// IIS2DLPC_RESERVED = 10-1f,
	IIS2DLPC_REG_CTRL1 = 0x20,
	IIS2DLPC_REG_CTRL2 = 0x21,
	IIS2DLPC_REG_CTRL3 = 0x22,
	IIS2DLPC_REG_CTRL4 = 0x23,
	IIS2DLPC_REG_CTRL5 = 0x24,
	IIS2DLPC_REG_CTRL6 = 0x25,
	IIS2DLPC_REG_OUT_T = 0x26,
	IIS2DLPC_REG_STATUS = 0x27,
	IIS2DLPC_REG_OUT_X_L = 0x28,
	IIS2DLPC_REG_OUT_X_H = 0x29,
	IIS2DLPC_REG_OUT_Y_L = 0x2a,
	IIS2DLPC_REG_OUT_Y_H = 0x2b,
	IIS2DLPC_REG_OUT_Z_L = 0x2c,
	IIS2DLPC_REG_OUT_Z_H = 0x2d,
	IIS2DLPC_REG_FIFO_CTRL = 0x2e,
	IIS2DLPC_REG_FIFO_SAMP = 0x2f,
	IIS2DLPC_REG_THS_X = 0x30,
	IIS2DLPC_REG_THS_Y = 0x31,
	IIS2DLPC_REG_THS_Z = 0x32,
	IIS2DLPC_REG_INT_DUR = 0x33,
	IIS2DLPC_REG_WAKE_UP_THS = 0x34,
	IIS2DLPC_REG_WAKE_UP_DUR = 0x35,
	IIS2DLPC_REG_FREE_FALL = 0x36,
	IIS2DLPC_REG_STATUS_DUP = 0x37,
	IIS2DLPC_REG_WAKE_UP_SRC = 0x38,
	IIS2DLPC_REG_TAP_SRC = 0x39,
	IIS2DLPC_REG_SIXD_SRC = 0x3a,
	IIS2DLPC_REG_ALL_INT_SRC = 0x3b,
	IIS2DLPC_REG_X_OFS_USR = 0x3c,
	IIS2DLPC_REG_Y_OFS_USR = 0x3d,
	IIS2DLPC_REG_Z_OFS_USR = 0x3e,
	IIS2DLPC_REG_CTRL7 = 0x3f,
};

enum IIS2DLPC_CTRL1
{
	IIS2DLPC_CTRL1_ODR_OFF = 0x00,
	IIS2DLPC_CTRL1_ODR_1_6 = 0x10,
	IIS2DLPC_CTRL1_ODR_12_5 = 0x20,
	IIS2DLPC_CTRL1_ODR_25 = 0x30,
	IIS2DLPC_CTRL1_ODR_50 = 0x40,
	IIS2DLPC_CTRL1_ODR_100 = 0x50,
	IIS2DLPC_CTRL1_ODR_200 = 0x60,
	IIS2DLPC_CTRL1_ODR_400 = 0x70,
	IIS2DLPC_CTRL1_ODR_800 = 0x80,
	IIS2DLPC_CTRL1_ODR_1600 = 0x90,

	IIS2DLPC_CTRL1_MODE_LP = 0x00,
	IIS2DLPC_CTRL1_MODE_HP = 0x04,
	IIS2DLPC_CTRL1_MODE_SS = 0x08,

	IIS2DLPC_CTRL1_LP_MODE1 = 0x00,
	IIS2DLPC_CTRL1_LP_MODE2 = 0x01,
	IIS2DLPC_CTRL1_LP_MODE3 = 0x02,
	IIS2DLPC_CTRL1_LP_MODE4 = 0x03,
};

enum IIS2DLPC_CTRL2
{
	IIS2DLPC_CTRL2_BOOT = 0x80,
	IIS2DLPC_CTRL2_SOFT_RESET = 0x40,

	IIS2DLPC_CTRL2_CS_PU_DISC = 0x10,
	IIS2DLPC_CTRL2_BDU = 0x08,
	IIS2DLPC_CTRL2_IF_ADD_INC = 0x04,
	IIS2DLPC_CTRL2_I2C_DISABLE = 0x02,
	IIS2DLPC_CTRL2_SIM = 0x01,
};

enum IIS2DLPC_CTRL3
{
	IIS2DLPC_CTRL3_ST_DISABLED = 0x00,
	IIS2DLPC_CTRL3_ST_POSITIVE = 0x40,
	IIS2DLPC_CTRL3_ST_NEGATIVE = 0x80,

	IIS2DLPC_CTRL3_PP_OD_PUSH_PULL = 0x00,
	IIS2DLPC_CTRL3_PP_OD_OPEN_DRAIN = 0x20,

	IIS2DLPC_CTRL3_LIR_PULSED = 0x00,
	IIS2DLPC_CTRL3_LIR_LATCHED = 0x10,

	IIS2DLPC_CTRL3_H_LACTIVE_HIGH = 0x00,
	IIS2DLPC_CTRL3_H_LACTIVE_LOW = 0x08,

	IIS2DLPC_CTRL3_SLP_MODE_SEL_INT2 = 0x00,
	IIS2DLPC_CTRL3_SLP_MODE_SEL_MODE_1 = 0x02,

	IIS2DLPC_CTRL3_SLP_MODE_1_INACTIVE = 0x00,
	IIS2DLPC_CTRL3_SLP_MODE_1_TRIGGER = 0x01,
};

enum IIS2DLPC_CTRL4
{
	IIS2DLPC_CTRL4_INT1_6D = 0x80,
	IIS2DLPC_CTRL4_INT1_SINGLE_TAP = 0x40,
	IIS2DLPC_CTRL4_INT1_WU = 0x20,
	IIS2DLPC_CTRL4_INT1_FF = 0x10,
	IIS2DLPC_CTRL4_INT1_DOUBLE_TAP = 0x08,
	IIS2DLPC_CTRL4_INT1_DIFF5 = 0x04,
	IIS2DLPC_CTRL4_INT1_FTH = 0x02,
	IIS2DLPC_CTRL4_INT1_DRDY = 0x01,
};

enum IIS2DLPC_CTRL5
{
	IIS2DLPC_CTRL5_INT2_SLEEP_STATE = 0x80,
	IIS2DLPC_CTRL5_INT2_SLEEP_CHG = 0x40,
	IIS2DLPC_CTRL5_INT2_BOOT = 0x20,
	IIS2DLPC_CTRL5_INT2_DRDY_T = 0x10,
	IIS2DLPC_CTRL5_INT2_OVR = 0x08,
	IIS2DLPC_CTRL5_INT2_DIFF5 = 0x04,
	IIS2DLPC_CTRL5_INT2_FTH = 0x02,
	IIS2DLPC_CTRL5_INT2_DRDY = 0x01,
};

enum IIS2DLPC_CTRL6
{
	IIS2DLPC_CTRL6_BW_FLT_ORD_2 = 0x00,
	IIS2DLPC_CTRL6_BW_FLT_ORD_4 = 0x40,
	IIS2DLPC_CTRL6_BW_FLT_ORD_10 = 0x80,
	IIS2DLPC_CTRL6_BW_FLT_ORD_20 = 0xc0,

	IIS2DLPC_CTRL6_FS_2G = 0x00,
	IIS2DLPC_CTRL6_FS_4G = 0x10,
	IIS2DLPC_CTRL6_FS_8G = 0x20,
	IIS2DLPC_CTRL6_FS_16G = 0x30,
	IIS2DLPC_CTRL6_FS_MASK = 0x30,
	IIS2DLPC_CTRL6_FS_OFFS = 4,

	IIS2DLPC_CTRL6_FDS_LOW = 0x00,
	IIS2DLPC_CTRL6_FDS_HIGH = 0x08,

	IIS2DLPC_CTRL6_LOW_NOISE_DISABLED = 0x00,
	IIS2DLPC_CTRL6_LOW_NOISE_ENABLED = 0x40,
};

enum IIS2DLPC_CTRL7
{
	IIS2DLPC_CTRL7_DRDY_MODE_LATCHED = 0x00,
	IIS2DLPC_CTRL7_DRDY_MODE_PULSED = 0x80,

	IIS2DLPC_CTRL7_INT2_ON_INT1 = 0x40,
	IIS2DLPC_CTRL7_INT_ENABLE = 0x20,
	IIS2DLPC_CTRL7_USR_OFF_ON_OUT = 0x10,
	IIS2DLPC_CTRL7_USR_OFF_ON_WU = 0x08,
	IIS2DLPC_CTRL7_USR_OFF_W = 0x04,
	IIS2DLPC_CTRL7_HP_REF_MODE = 0x02,
	IIS2DLPC_CTRL7_LPASS_ON_6D = 0x01,
};

enum IIS2DLPC_FIFO_CTRL
{
	IIS2DLPC_FIFO_CTRL_MODE_BYPASS = 0x00,
	IIS2DLPC_FIFO_CTRL_MODE_FIFO = 0x20,
	IIS2DLPC_FIFO_CTRL_MODE_CONTINUOUS_TO_FIFO = 0x60,
	IIS2DLPC_FIFO_CTRL_MODE_BYPASS_TO_CONTINUOUS = 0x80,
	IIS2DLPC_FIFO_CTRL_MODE_CONTINUOUS = 0xc0,

	IIS2DLPC_FIFO_CTRL_FTH_MASK = 0x1f,
};


struct iis2dlpc_t
{
	union
	{
		uint8_t arr[6];

		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

void iis2dlpc_init_api(const struct device * iis2dlpc_dev);
int iis2dlpc_read(const struct device * iis2dlpc_dev, struct iis2dlpc_t * iis2dlpc);
