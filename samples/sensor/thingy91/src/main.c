/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>


#if CONFIG_NETWORKING
int btn_init(void);
int led_init(void);
ssize_t lte_send(const char * msg, size_t size);
#endif


#define OK 0
#define FAIL -1

#define MSG_SIZE 256

#define ENABLE_UART_OUT 1
#define ENABLE_ADXL362  1
#define ENABLE_ADXL372  1
#define ENABLE_BH1479   1
#define ENABLE_BME680   1

#define ADXL372_SKIP    4
#define BME680_SKIP    32

//#undef CONFIG_ADXL372_PEAK_DETECT_MODE
#define BH1479_THRESHOLD_UPPER          50
#define BH1479_THRESHOLD_LOWER          0
#define BH1479_TRIGGER_ON_DATA_READY    1


K_SEM_DEFINE(adxl362_sem, 0, 1);
K_SEM_DEFINE(adxl372_sem, 0, 1);
K_SEM_DEFINE(bh1479_sem, 0, 1);



#if ENABLE_ADXL362
// ADXL362
static void trigger_handler_adxl362(const struct device * dev, const struct sensor_trigger * trig)
{
	switch (trig->type)
	{
	case SENSOR_TRIG_DATA_READY:
		if (sensor_sample_fetch(dev) < 0)
		{
			printf("ADXL362  Sample fetch error\n");
			return;
		}

		k_sem_give(&adxl362_sem);

		break;

	case SENSOR_TRIG_MOTION:
		printf("ADXL362  Motion trigger\n");

		break;

	case SENSOR_TRIG_STATIONARY:
		printf("ADXL362  Stationary trigger\n");

		break;

	default:
		printf("ADXL362  Unknown trigger\n");
	}
}
#endif


#if ENABLE_ADXL372
// ADXL372
static void trigger_handler_adxl372(const struct device * dev, const struct sensor_trigger * trigger)
{
	ARG_UNUSED(trigger);

	if (sensor_sample_fetch(dev))
	{
		printf("ADXL372  sensor_sample_fetch failed\n");
		return;
	}

	k_sem_give(&adxl372_sem);
}
#endif


#if ENABLE_BH1479
// BH1479
static void bh1479_trigger_handler(const struct device * dev, const struct sensor_trigger * trigger)
{
#if 0
	ARG_UNUSED(dev);

	switch (trigger->type)
	{
	case SENSOR_TRIG_THRESHOLD:
		printf("BH1479  Threshold trigger\r\n");
		break;

	case SENSOR_TRIG_DATA_READY:
		printf("BH1479  Data ready trigger\r\n");
		break;

	default:
		printf("BH1479  Unknown trigger event %d\r\n", trigger->type);
		break;
	}
#endif


	// The sensor does only support fetching SENSOR_CHAN_ALL
	int ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);

	if (ret)
	{
		printf("sensor_sample_fetch failed ret %d\n", ret);
		return;
	}

	k_sem_give(&bh1479_sem);
}

static int bh1479_set_attribute(
	const struct device * dev,
	enum sensor_channel chan,
	enum sensor_attribute attr,
	int value
)
{
	int ret;
	struct sensor_value sensor_val;

	sensor_val.val1 = (value);

	ret = sensor_attr_set(dev, chan, attr, &sensor_val);

	if (ret)
	{
		printf("BH1479  sensor_attr_set failed ret %d\n", ret);
	}

	return ret;
}
#endif


void main(void)
{
	char msg[MSG_SIZE];
	size_t size = 0;

#if CONFIG_NETWORKING
	// init button and LED
	led_init();
	btn_init();
#endif


#if ENABLE_ADXL362
	// ADXL362
	const struct device * adxl362_dev = DEVICE_DT_GET_ONE(adi_adxl362);

	if (!device_is_ready(adxl362_dev))
	{
		printf("ADXL362  Device %s is not ready\n", adxl362_dev->name);
		return;
	}

	if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
	{
		struct sensor_trigger trig = { .chan = SENSOR_CHAN_ACCEL_XYZ };

		trig.type = SENSOR_TRIG_MOTION;

		if (sensor_trigger_set(adxl362_dev, &trig, trigger_handler_adxl362))
		{
			printf("ADXL362  Trigger set error\n");
			return;
		}

		trig.type = SENSOR_TRIG_DATA_READY;

		if (sensor_trigger_set(adxl362_dev, &trig, trigger_handler_adxl362))
		{
			printf("ADXL362  Trigger set error\n");
		}
	}
#endif


#if ENABLE_ADXL372
	// ADXL372
	const struct device * adxl372_dev = DEVICE_DT_GET_ANY(adi_adxl372);

	if (!device_is_ready(adxl372_dev))
	{
		printf("ADXL372  Device %s is not ready\n", adxl372_dev->name);
		return;
	}

	struct sensor_trigger adxl372_trig =
	{
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (IS_ENABLED(CONFIG_ADXL372_PEAK_DETECT_MODE))
	{
		adxl372_trig.type = SENSOR_TRIG_THRESHOLD;
	}

	if (IS_ENABLED(CONFIG_ADXL372_TRIGGER))
	{
		if (sensor_trigger_set(adxl372_dev, &adxl372_trig, trigger_handler_adxl372))
		{
			printf("ADXL372  Could not set trigger\n");
			return;
		}
	}
#endif


#if ENABLE_BH1479
	// BH1479
	const struct device * bh1479_dev;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT))
	{
		// Give RTT log time to be flushed before executing tests
		//-k_sleep(K_MSEC(500));
	}

	bh1479_dev = device_get_binding("BH1749");

	if (bh1479_dev == NULL)
	{
		printf("BH1479  Failed to get device binding\n");
		return;
	}

	printf("BH1479  Device %p name is %s\n", bh1479_dev, bh1479_dev->name);

	struct sensor_trigger sensor_trig_conf =
	{
#if (BH1479_TRIGGER_ON_DATA_READY)
		.type = SENSOR_TRIG_DATA_READY,
#else
		.type = SENSOR_TRIG_THRESHOLD,
#endif

		.chan = SENSOR_CHAN_RED,
	};

	if (IS_ENABLED(CONFIG_BH1749_TRIGGER))
	{
		bh1479_set_attribute(
			bh1479_dev,
			SENSOR_CHAN_ALL,
			SENSOR_ATTR_LOWER_THRESH,
			BH1479_THRESHOLD_LOWER
		);

		bh1479_set_attribute(
			bh1479_dev,
			SENSOR_CHAN_ALL,
			SENSOR_ATTR_UPPER_THRESH,
			BH1479_THRESHOLD_UPPER
		);

		if (sensor_trigger_set(bh1479_dev, &sensor_trig_conf, bh1479_trigger_handler))
		{
			printf("BH1479  Could not set trigger\n");
			return;
		}
	}

#endif


#if ENABLE_BME680
	// BME680
	const struct device * bme680_dev = DEVICE_DT_GET_ONE(bosch_bme680);

	if (!device_is_ready(bme680_dev))
	{
		printk("BME680  Device not ready.\n");
		return;
	}

	printf("BME680  Device %p name is %s\n", bme680_dev, bme680_dev->name);
#endif


	//
	int i = 0;


	while (true)
	{
		i++;

#if ENABLE_ADXL362
		// ADXL362
		if (1)
		{
			if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
			{
				k_sem_take(&adxl362_sem, K_FOREVER);
			}
			else
			{
				k_sleep(K_MSEC(1000));

				if (sensor_sample_fetch(adxl362_dev) < 0)
				{
					printf("ADXL372  Sample fetch error\n");
					return;
				}
			}

			struct sensor_value adxl362_accel[3];

			if (sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_X, &adxl362_accel[0]) < 0)
			{
				printf("ADXL372  Channel get error\n");
				return;
			}

			if (sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_Y, &adxl362_accel[1]) < 0)
			{
				printf("ADXL372  Channel get error\n");
				return;
			}

			if (sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_Z, &adxl362_accel[2]) < 0)
			{
				printf("ADXL372  Channel get error\n");
				return;
			}

			size = snprintf(
				msg,
				sizeof(msg),
				"adxl362={\"x\": %.1f, \"y\": %.1f, \"z\": %.1f}\n",
				sensor_value_to_double(&adxl362_accel[0]),
				sensor_value_to_double(&adxl362_accel[1]),
				sensor_value_to_double(&adxl362_accel[2])
			);

#if ENABLE_UART_OUT
			printf("%s", msg);
#endif

#if CONFIG_NETWORKING
			lte_send(msg, size);
#endif
		}
#endif


#if ENABLE_ADXL372
		// ADXL372
		if (!(i % ADXL372_SKIP))
		{
			if (sensor_sample_fetch(adxl372_dev))
			{
				printf("ADXL372  sensor_sample_fetch failed\n");
			}

			struct sensor_value adxl372_accel[3];
			sensor_channel_get(adxl372_dev, SENSOR_CHAN_ACCEL_XYZ, adxl372_accel);

			size = snprintf(
				msg,
				sizeof(msg),
				"adxl372={\"x\": %.1f, \"y\": %.1f, \"z\": %.1f}\n",
				sensor_value_to_double(&adxl372_accel[0]),
				sensor_value_to_double(&adxl372_accel[1]),
				sensor_value_to_double(&adxl372_accel[2])
			);

#if ENABLE_UART_OUT
			printf("%s", msg);
#endif

#if CONFIG_NETWORKING
			lte_send(msg, size);
#endif
		}
#endif


#if ENABLE_BH1479
		// BH1479
		if (OK == k_sem_take(&bh1479_sem, K_NO_WAIT))
		{
			struct sensor_value bh1479_r;
			struct sensor_value bh1479_g;
			struct sensor_value bh1479_b;
			struct sensor_value bh1479_i;

			int ret_r = sensor_channel_get(bh1479_dev, SENSOR_CHAN_RED, &bh1479_r);
			int ret_g = sensor_channel_get(bh1479_dev, SENSOR_CHAN_GREEN, &bh1479_g);
			int ret_b = sensor_channel_get(bh1479_dev, SENSOR_CHAN_BLUE, &bh1479_b);
			int ret_i = sensor_channel_get(bh1479_dev, SENSOR_CHAN_IR, &bh1479_i);

			if (ret_r || ret_g || ret_b || ret_i)
			{
				printf(
					"sensor_channel_get failed  r %d  g %d  b %d  i %d\n",
					ret_r,
					ret_g,
					ret_b,
					ret_i
				);

				return;
			}

			size = snprintf(
				msg,
				sizeof(msg),
				"bh1479={\"r\": %d, \"g\": %d, \"b\": %d, \"i\": %d}\n",
				bh1479_r.val1,
				bh1479_g.val1,
				bh1479_b.val1,
				bh1479_i.val1
			);

#if ENABLE_UART_OUT
			printf("%s", msg);
#endif

#if CONFIG_NETWORKING
			lte_send(msg, size);
#endif
		}
#endif


#if ENABLE_BME680
		// BME680
		if (!(i % BME680_SKIP))
		{
			//k_sleep(K_MSEC(3000));
			struct sensor_value temp, press, humidity, gas_res;

			sensor_sample_fetch(bme680_dev);
			sensor_channel_get(bme680_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(bme680_dev, SENSOR_CHAN_PRESS, &press);
			sensor_channel_get(bme680_dev, SENSOR_CHAN_HUMIDITY, &humidity);
			sensor_channel_get(bme680_dev, SENSOR_CHAN_GAS_RES, &gas_res);

			size = snprintf(
				msg,
				sizeof(msg),
				"bme680={\"temp\": %d.%03d, \"pressure\": %d.%03d, \"humidity\": %d.%03d, \"gas\": %d}\n",
				temp.val1, temp.val2 / 1000,
				press.val1, press.val2 / 1000,
				humidity.val1, humidity.val2 / 1000,
				gas_res.val1
			);

#if ENABLE_UART_OUT
			printf("%s", msg);
#endif

#if CONFIG_NETWORKING
			lte_send(msg, size);
#endif
		}
#endif
	}
}
