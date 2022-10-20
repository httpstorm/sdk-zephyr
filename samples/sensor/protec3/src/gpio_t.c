/*
 * Copyright (c) 2020 EUROS Embedded Systems GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <sys/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>

 // LTE modem
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>

#include "gpio_t.h"


// LED
struct LEDS_t led;


int led_init(void)
{
	int b = OK;
	size_t i = 0;

	memset(&led, 0, sizeof(led));

	if ((LED_COUNT != LED_COUNT_PINS) || (LED_COUNT != LED_COUNT_FLAGS))
	{
		printk("led_init()  led_names, led_nodes, and led_flags should have the same length\n");
		return FAIL;
	}

	for (i = 0; i < LED_COUNT; i++)
	{
		led.list[i].dev = device_get_binding(led_names[i]);
		led.list[i].pin = led_pins[i];

		// include GPIO_INPUT so we can read the state of the output using pin_get()
		if (
			!led.list[i].dev ||
			(gpio_pin_configure(
				led.list[i].dev,
				led.list[i].pin,
				led_flags[i] | GPIO_INPUT | GPIO_OUTPUT_INACTIVE
			) < 0)
			)
		{
			led.list[i].dev = NULL;
			b = FAIL;
		}
	}

	return b;
}

int pin_get(struct PIN_obj_t pin)
{
	if (!pin.dev)
	{
		return 0;
	}

	return gpio_pin_get(pin.dev, pin.pin);
}

int pin_set(struct PIN_obj_t pin, int on)
{
	if (!pin.dev)
	{
		return FAIL;
	}

	return gpio_pin_set(pin.dev, pin.pin, on);
}

int led_set_rgb(struct LED_t * led, int r, int g, int b)
{
	int err = 0;
	err |= pin_set(led->r, r);
	err |= pin_set(led->g, g);
	err |= pin_set(led->b, b);

	return err;
}


// BUTTONS
static struct gpio_callback btn_0_cb_data;
struct PIN_obj_t btn_0;
int btn_0_trigger = 0;

void btn_0_pressed(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	btn_0_trigger = 1;
	led_set_rgb(&led.sense, 1, 1, 1);
}

int btn_init(void)
{
	int b = OK;

	btn_0.dev = device_get_binding(BTN_0);
	btn_0.pin = BTN_0_PIN;

	if (
		!btn_0.dev ||
		(OK != gpio_pin_configure(btn_0.dev, btn_0.pin, BTN_0_FLAGS | GPIO_INPUT)) ||
		(OK != gpio_pin_interrupt_configure(btn_0.dev, btn_0.pin, GPIO_INT_EDGE_TO_ACTIVE))
		)
	{
		btn_0.dev = NULL;
		b = FAIL;
	}
	else
	{
		gpio_init_callback(&btn_0_cb_data, btn_0_pressed, BIT(btn_0.pin));
		gpio_add_callback(btn_0.dev, &btn_0_cb_data);
	}

	return OK;
}
