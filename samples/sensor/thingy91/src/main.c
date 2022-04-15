/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
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
#define ENABLE_BH1749   1
#define ENABLE_BME680   1

#define ADXL372_SKIP    4
#define BME680_SKIP    32

//#undef CONFIG_ADXL372_PEAK_DETECT_MODE
#define BH1749_THRESHOLD_UPPER          50
#define BH1749_THRESHOLD_LOWER          0
#define BH1749_TRIGGER_ON_DATA_READY    1


K_SEM_DEFINE(adxl362_sem, 0, 1);
K_SEM_DEFINE(adxl372_sem, 0, 1);
K_SEM_DEFINE(bh1749_sem, 0, 1);



#if ENABLE_ADXL362
// ADXL362
static void trigger_handler_adxl362(const struct device * dev, const struct sensor_trigger * trig)
{
	switch (trig->type)
	{
	case SENSOR_TRIG_DATA_READY:
	{
		int ret = sensor_sample_fetch(dev);

		if (ret < 0)
		{
			printf("ADXL362  Sample fetch error %d\n", ret);
			return;
		}

		k_sem_give(&adxl362_sem);

		break;
	}

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

	int ret = sensor_sample_fetch(dev);

	if (ret < 0)
	{
		printf("ADXL372  sensor_sample_fetch failed %d\n", ret);
		return;
	}

	k_sem_give(&adxl372_sem);
}
#endif


#if ENABLE_BH1749
// BH1749
static void bh1749_trigger_handler(const struct device * dev, const struct sensor_trigger * trigger)
{
#if 0
	ARG_UNUSED(dev);

	switch (trigger->type)
	{
	case SENSOR_TRIG_THRESHOLD:
		printf("BH1749  Threshold trigger\r\n");
		break;

	case SENSOR_TRIG_DATA_READY:
		printf("BH1749  Data ready trigger\r\n");
		break;

	default:
		printf("BH1749  Unknown trigger event %d\r\n", trigger->type);
		break;
	}
#endif


	// The sensor does only support fetching SENSOR_CHAN_ALL
	int ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);

	if (ret)
	{
		printf("BH1749  sensor_sample_fetch failed ret %d\n", ret);
		return;
	}

	k_sem_give(&bh1749_sem);
}

static int bh1749_set_attribute(
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
		printf("BH1749  sensor_attr_set failed ret %d\n", ret);
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

	printf(
		"ADXL362 Device %p name is %s  "
#if CONFIG_ADXL362_TRIGGER
		"TRIGGER"
#else
		"NO TRIGGER"
#endif
		"\n",
		adxl362_dev,
		adxl362_dev->name
	);

	if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
	{
		struct sensor_trigger trig =
		{
			.type = SENSOR_TRIG_MOTION,
			.chan = SENSOR_CHAN_ACCEL_XYZ,
		};

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

	printf(
		"ADXL372 Device %p name is %s  "
#if CONFIG_ADXL372_TRIGGER
#if CONFIG_ADXL372_PEAK_DETECT_MODE
		"THRESHOLD"
#else
		"DATA_READY"
#endif
#else
		"NO TRIGGER"
#endif
		"\n",
		adxl372_dev,
		adxl372_dev->name
	);

	if (IS_ENABLED(CONFIG_ADXL372_TRIGGER))
	{
		const struct sensor_trigger trig =
		{
#if CONFIG_ADXL372_PEAK_DETECT_MODE
			.type = SENSOR_TRIG_THRESHOLD,
#else
			.type = SENSOR_TRIG_DATA_READY,
#endif
			.chan = SENSOR_CHAN_ACCEL_XYZ,
		};

		if (sensor_trigger_set(adxl372_dev, &trig, trigger_handler_adxl372))
		{
			printf("ADXL372  Could not set trigger\n");
			return;
		}
	}
#endif


#if ENABLE_BH1749
	// BH1749
	const struct device * bh1749_dev;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT))
	{
		// Give RTT log time to be flushed before executing tests
		//-k_sleep(K_MSEC(500));
	}

	bh1749_dev = DEVICE_DT_GET_ONE(rohm_bh1749);

	if (bh1749_dev == NULL)
	{
		printf("BH1749  Failed to get device binding\n");
		return;
	}

	printf(
		"BH1749  Device %p name is %s  "
#if CONFIG_BH1749_TRIGGER
		"TRIGGER"
#else
		"NO TRIGGER"
#endif
		"\n",
		bh1749_dev,
		bh1749_dev->name
	);

	if (IS_ENABLED(CONFIG_BH1749_TRIGGER))
	{
		const struct sensor_trigger sensor_trig_conf =
		{
#if BH1749_TRIGGER_ON_DATA_READY
			.type = SENSOR_TRIG_DATA_READY,
#else
			.type = SENSOR_TRIG_THRESHOLD,
#endif
			.chan = SENSOR_CHAN_RED,
		};

		bh1749_set_attribute(
			bh1749_dev,
			SENSOR_CHAN_ALL,
			SENSOR_ATTR_LOWER_THRESH,
			BH1749_THRESHOLD_LOWER
		);

		bh1749_set_attribute(
			bh1749_dev,
			SENSOR_CHAN_ALL,
			SENSOR_ATTR_UPPER_THRESH,
			BH1749_THRESHOLD_UPPER
		);

		if (sensor_trigger_set(bh1749_dev, &sensor_trig_conf, bh1749_trigger_handler))
		{
			printf("BH1749  Could not set trigger\n");
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
#if !CONFIG_ADXL362_TRIGGER
		{
			k_sleep(K_MSEC(1000));
			const int ret = sensor_sample_fetch(adxl362_dev);

			if (ret < 0)
			{
				printf("ADXL362  Sample fetch error %d\n", ret);
				return;
			}
		}
#else
		if (OK == k_sem_take(&adxl362_sem, K_FOREVER))
#endif
		{
			struct sensor_value adxl362_accel[3];

			if (sensor_channel_get(adxl362_dev, SENSOR_CHAN_ACCEL_XYZ, adxl362_accel))
			{
				printf("ADXL362  Channel get error\n");
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
#if !CONFIG_ADXL372_TRIGGER
		if (!(i % ADXL372_SKIP))
		{
			const int ret = sensor_sample_fetch(adxl372_dev);

			if (ret < 0)
			{
				printf("ADXL372  sensor_sample_fetch failed %d\n", ret);
				return;
			}
#else
		if (OK == k_sem_take(&adxl372_sem, K_NO_WAIT))
		{
#endif
			struct sensor_value adxl372_accel[3];

			if (sensor_channel_get(adxl372_dev, SENSOR_CHAN_ACCEL_XYZ, adxl372_accel))
			{
				printf("ADXL372  Channel get error\n");
				return;
			}

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


#if ENABLE_BH1749
		// BH1749
#if !CONFIG_BH1749_TRIGGER
		{
			const int ret = sensor_sample_fetch_chan(bh1749_dev, SENSOR_CHAN_ALL);

			if (ret < 0)
			{
				printf("BH1749  sensor_sample_fetch failed %d\n", ret);
				return;
			}
		}
#else
		if (OK == k_sem_take(&bh1749_sem, K_NO_WAIT))
#endif
		{
			struct sensor_value bh1749_r;
			struct sensor_value bh1749_g;
			struct sensor_value bh1749_b;
			struct sensor_value bh1749_i;

			int ret_r = sensor_channel_get(bh1749_dev, SENSOR_CHAN_RED, &bh1749_r);
			int ret_g = sensor_channel_get(bh1749_dev, SENSOR_CHAN_GREEN, &bh1749_g);
			int ret_b = sensor_channel_get(bh1749_dev, SENSOR_CHAN_BLUE, &bh1749_b);
			int ret_i = sensor_channel_get(bh1749_dev, SENSOR_CHAN_IR, &bh1749_i);

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
				"bh1749={\"r\": %d, \"g\": %d, \"b\": %d, \"i\": %d}\n",
				bh1749_r.val1,
				bh1749_g.val1,
				bh1749_b.val1,
				bh1749_i.val1
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
			const int ret = sensor_sample_fetch(bme680_dev);

			if (ret < 0)
			{
				printf("BME680  sensor_sample_fetch failed %d\n", ret);
				return;
			}

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
