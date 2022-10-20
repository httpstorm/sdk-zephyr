/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <drivers/pwm.h>
#include <drivers/gpio.h>

//#include "ui.h"
#include "buzzer.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(buzzer, 3);

#define CONFIG_UI_BUZZER_PWM_DEV_NAME "PWM_1"
#define CONFIG_UI_BUZZER_PIN 0
#define CONFIG_UI_BUZZER_MIN_FREQUENCY 1000
#define CONFIG_UI_BUZZER_MAX_FREQUENCY 5000

#define BUZZER_MIN_FREQUENCY		CONFIG_UI_BUZZER_MIN_FREQUENCY
#define BUZZER_MAX_FREQUENCY		CONFIG_UI_BUZZER_MAX_FREQUENCY
#define BUZZER_MIN_INTENSITY		0
#define BUZZER_MAX_INTENSITY		100
#define BUZZER_MIN_DUTY_CYCLE_DIV	100
#define BUZZER_MAX_DUTY_CYCLE_DIV	2

#define BUZZER_ENABLE_PIN 2

const struct device *pwm_dev = NULL;
const struct device *gpio0;
static atomic_t buzzer_enabled;

static u32_t intensity_to_duty_cycle_divisor(u8_t intensity)
{
	return MIN(
		MAX(((intensity - BUZZER_MIN_INTENSITY) *
		    (BUZZER_MAX_DUTY_CYCLE_DIV - BUZZER_MIN_DUTY_CYCLE_DIV) /
		    (BUZZER_MAX_INTENSITY - BUZZER_MIN_INTENSITY) +
		    BUZZER_MIN_DUTY_CYCLE_DIV),
		    BUZZER_MAX_DUTY_CYCLE_DIV),
		BUZZER_MIN_DUTY_CYCLE_DIV);
}

static int pwm_out(u32_t frequency, u8_t intensity)
{
	static u32_t prev_period;
	u32_t period = (frequency > 0) ? USEC_PER_SEC / frequency : 0;
	u32_t duty_cycle = (intensity == 0) ? 0 :
		period / intensity_to_duty_cycle_divisor(intensity);

	/* Applying workaround due to limitations in PWM driver that doesn't
	 * allow changing period while PWM is running. Setting pulse to 0
	 * disables the PWM, but not before the current period is finished.
	 */
	if (prev_period) {
		pwm_pin_set_usec(pwm_dev, CONFIG_UI_BUZZER_PIN,
				 prev_period, 0, 0);
		k_sleep(K_MSEC(MAX((prev_period / USEC_PER_MSEC), 1)));
	}

	prev_period = period;

	return pwm_pin_set_usec(pwm_dev, CONFIG_UI_BUZZER_PIN,
				period, duty_cycle, 0);
}

static void buzzer_disable(void)
{
	atomic_set(&buzzer_enabled, 0);

	pwm_out(0, 0);
        gpio_pin_set(gpio0, BUZZER_ENABLE_PIN, 0);


#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	int err = device_set_power_state(pwm_dev,
					 DEVICE_PM_SUSPEND_STATE,
					 NULL, NULL);
	if (err) {
		LOG_ERR("PWM disable failed");
	}
#endif
}

static int buzzer_enable(void)
{
	int err = 0;

	atomic_set(&buzzer_enabled, 1);
        gpio_pin_set(gpio0, BUZZER_ENABLE_PIN, 1);

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	err = device_set_power_state(pwm_dev,
					 DEVICE_PM_ACTIVE_STATE,
					 NULL, NULL);
	if (err) {
		LOG_ERR("PWM enable failed");
		return err;
	}
#endif

	return err;
}

int ui_buzzer_init(void)
{
	const char dev_name[] = CONFIG_UI_BUZZER_PWM_DEV_NAME;
	int err = 0;

	gpio0 = device_get_binding("GPIO_0");
        if (gpio0 == NULL) {
            printk("Buzzer init: error, no GPIO device!\n");
            return -ENODEV;
        }

        //if (gpio0) {
              if (gpio_pin_configure(gpio0, 1, GPIO_OUTPUT) == 0 && gpio_pin_configure(gpio0, BUZZER_ENABLE_PIN, GPIO_OUTPUT) == 0) {
                    if (gpio_pin_set(gpio0, 1, 0) == 0 && gpio_pin_set(gpio0, BUZZER_ENABLE_PIN, 0) == 0) {
                          pwm_dev = device_get_binding(dev_name);
                          if (!pwm_dev) {
                                LOG_ERR("Could not bind to device %s", log_strdup(dev_name));
                                err = -ENODEV;
                          }
                    }
              }
	//}

	//buzzer_enable();

	return err;
}

int ui_buzzer_set_frequency(u32_t frequency, u8_t intensity)
{
        if (pwm_dev == NULL) {
              LOG_DBG("Buzzer is not initialized!\n");
              return -ENODEV;
        }
	if (frequency == 0 || intensity == 0) {
		LOG_DBG("Frequency set to 0, disabling PWM\n");
		buzzer_disable();
		return 0;
	}

	if ((frequency < BUZZER_MIN_FREQUENCY) ||
	    (frequency > BUZZER_MAX_FREQUENCY)) {
		return -EINVAL;
	}

	if ((intensity < BUZZER_MIN_INTENSITY) ||
	    (intensity > BUZZER_MAX_INTENSITY)) {
		return -EINVAL;
	}

	if (!atomic_get(&buzzer_enabled)) {
		buzzer_enable();
	}

	return pwm_out(frequency, intensity);
}
