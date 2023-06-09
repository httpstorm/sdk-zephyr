/*
 * Copyright (c) 2020 EUROS Embedded Systems GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if CONFIG_NETWORKING
#include <zephyr/kernel.h>
#include <nrf_socket.h>
#include <zephyr/net/socket.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <string.h>

 // LTE modem
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>


#define OK 0
#define FAIL -1

#define CONFIG_TCP_SERVER_ADDRESS_STATIC "81.161.241.2" // gvalkov.com
//#define CONFIG_TCP_SERVER_ADDRESS_STATIC "87.138.172.131" // euros-embedded.com
#define CONFIG_TCP_SERVER_PORT 81


// LED
#if CONFIG_BOARD_THINGY91_NRF9160_NS
#define LED_R_NODE DT_ALIAS(led0)
#define LED_G_NODE DT_ALIAS(led1)
#define LED_B_NODE DT_ALIAS(led2)
#define LED_SR_NODE DT_ALIAS(sense_led0)
#define LED_SG_NODE DT_ALIAS(sense_led1)
#define LED_SB_NODE DT_ALIAS(sense_led2)
#elif CONFIG_BOARD_NRF9160DK_NRF9160_NS
#define LED_R_NODE DT_ALIAS(led1)
#define LED_G_NODE DT_ALIAS(led2)
#define LED_B_NODE DT_ALIAS(led3)
#define LED_SR_NODE DT_ALIAS(led1)
#define LED_SG_NODE DT_ALIAS(led2)
#define LED_SB_NODE DT_ALIAS(led3)
#else
#error "Unsupported board: please use nRF9160dk or Thingy:91"
#endif

#if DT_NODE_HAS_STATUS(LED_R_NODE, okay)
#define LED_R	  GPIO_DT_SPEC_GET(LED_R_NODE, gpios)
#define LED_G	  GPIO_DT_SPEC_GET(LED_G_NODE, gpios)
#define LED_B	  GPIO_DT_SPEC_GET(LED_B_NODE, gpios)
#define LED_SR	  GPIO_DT_SPEC_GET(LED_SR_NODE, gpios)
#define LED_SG	  GPIO_DT_SPEC_GET(LED_SG_NODE, gpios)
#define LED_SB	  GPIO_DT_SPEC_GET(LED_SB_NODE, gpios)
#define PIN_R	  DT_GPIO_PIN(LED_R_NODE, gpios)
#define PIN_G	  DT_GPIO_PIN(LED_G_NODE, gpios)
#define PIN_B	  DT_GPIO_PIN(LED_B_NODE, gpios)
#define PIN_SR	  DT_GPIO_PIN(LED_SR_NODE, gpios)
#define PIN_SG	  DT_GPIO_PIN(LED_SG_NODE, gpios)
#define PIN_SB	  DT_GPIO_PIN(LED_SB_NODE, gpios)
#define FLAGS_R	  DT_GPIO_FLAGS(LED_R_NODE, gpios)
#define FLAGS_G	  DT_GPIO_FLAGS(LED_G_NODE, gpios)
#define FLAGS_B	  DT_GPIO_FLAGS(LED_B_NODE, gpios)
#define FLAGS_SR  DT_GPIO_FLAGS(LED_SR_NODE, gpios)
#define FLAGS_SG  DT_GPIO_FLAGS(LED_SG_NODE, gpios)
#define FLAGS_SB  DT_GPIO_FLAGS(LED_SB_NODE, gpios)

gpio_flags_t led_flags[] = { FLAGS_R, FLAGS_G, FLAGS_B, FLAGS_SR, FLAGS_SG, FLAGS_SB };
#define LED_COUNT (sizeof(led_flags) / sizeof(*led_flags))
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

struct LED_t
{
	const struct gpio_dt_spec r;
	const struct gpio_dt_spec g;
	const struct gpio_dt_spec b;
};

struct LEDS_t
{
	union
	{
		struct
		{
			struct LED_t main;
			struct LED_t sense;
		};

		const struct gpio_dt_spec list[LED_COUNT];
	};
};

static struct LEDS_t led =
{
	.list =
	{
		LED_R, LED_G, LED_B, LED_SR, LED_SG, LED_SB
	}
};

static int led_g_state = false;

int led_init(void)
{
	int b = OK;
	size_t i = 0;

	for (i = 0; i < LED_COUNT; i++)
	{
		gpio_pin_configure_dt(&led.list[i], led_flags[i] | GPIO_OUTPUT_INACTIVE);
	}

	return b;
}

int led_set(const struct gpio_dt_spec * led, int on)
{
	return gpio_pin_set_dt(led, on);
}

int led_set_rgb(struct LED_t * led, int r, int g, int b)
{
	int err = 0;
	err |= led_set(&led->r, r);
	err |= led_set(&led->g, g);
	err |= led_set(&led->b, b);

	return err;
}

int led_toggle_g(void)
{
	led_g_state = led_g_state ? false : true;

	return led_set(&led.main.g, led_g_state);
}


// BUTTONS
#define BTN_0_NODE DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(BTN_0_NODE, okay)
#define BTN_0		GPIO_DT_SPEC_GET(BTN_0_NODE, gpios)
#define BTN_0_FLAGS	DT_GPIO_FLAGS(BTN_0_NODE, gpios)
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static struct gpio_callback btn_0_cb_data;
static const struct gpio_dt_spec btn_0 = BTN_0;
static int btn_0_trigger_count = 0;
static int btn_0_trigger = 0;

void btn_0_pressed(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	if (++btn_0_trigger_count >= 5)
	{
		btn_0_trigger_count = 0;
		btn_0_trigger = 1;
		led_set_rgb(&led.sense, 1, 1, 1);
	}
}

int btn_init(void)
{
	int b = FAIL;

	gpio_pin_configure_dt(&btn_0, BTN_0_FLAGS | GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&btn_0, GPIO_INT_EDGE_TO_ACTIVE);

	if (
		(OK == gpio_pin_configure_dt(&btn_0, BTN_0_FLAGS | GPIO_INPUT)) &&
		(OK == gpio_pin_interrupt_configure_dt(&btn_0, GPIO_INT_EDGE_TO_ACTIVE))
		)
	{
		b = OK;
		gpio_init_callback(&btn_0_cb_data, btn_0_pressed, BIT(btn_0.pin));
		gpio_add_callback(btn_0.port, &btn_0_cb_data);
	}

	return OK;
}

int btn_get(const struct gpio_dt_spec * btn)
{
	return gpio_pin_get_dt(btn);
}


static struct modem_param_info modem_param;
static struct sockaddr_storage host_addr;
static int lte_enabled = 0;
static int lte_ready = 0;
static int so = FAIL;


// connect to TCP server
int tcp_connect(void)
{
	led_set_rgb(&led.main, 1, 1, 1);
	printk("connecting %s:%u  ", CONFIG_TCP_SERVER_ADDRESS_STATIC, CONFIG_TCP_SERVER_PORT);

	// server address init
	struct sockaddr_in * server4 = ((struct sockaddr_in *)&host_addr);

	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_TCP_SERVER_PORT);

	inet_pton(AF_INET, CONFIG_TCP_SERVER_ADDRESS_STATIC, &server4->sin_addr);

	// server connect
	int so = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (so < 0)
	{
		led_set_rgb(&led.main, 1, 0, 1);
		printk("\n\tcannot create TCP socket  %u %s\n\n", errno, strerror(errno));

		return FAIL;
	}

	led_set_rgb(&led.main, 0, 1, 1);

	int err = connect(so, (struct sockaddr *)&host_addr, sizeof(struct sockaddr_in));

	if (err < 0)
	{
		close(so);
		led_set_rgb(&led.main, 1, 0, 0);
		printk("\n\tFAILED  %u %s\n\n", errno, strerror(errno));

		return FAIL;
	}

	led_set_rgb(&led.main, 0, 1, 0);
	printk("OK\n\n");

	return so;
}

int lte_init(void)
{
	// init LTE
	led_set_rgb(&led.sense, 0, 0, 1);
	printk("LTE modem starting...  ");
	int b = lte_lc_init_and_connect();

	if (b != OK)
	{
		led_set_rgb(&led.sense, 1, 0, 0);
		printk("FAILED %u\n\n", -b);
		return FAIL;
	}

	led_set_rgb(&led.sense, 0, 1, 1);
	printk("OK  ");

	// init AT CMD parser
	b = modem_info_init();

	if (b != OK)
	{
		led_set_rgb(&led.sense, 1, 0, 1);
		printk("\n\tLTE modem AT CMD parser failed to start: %d\n\n", -b);

		return FAIL;
	}

	// init the modem information structure
	b = modem_info_params_init(&modem_param);

	if (b != OK)
	{
		led_set_rgb(&led.sense, 1, 1, 0);
		printk("\n\tLTE modem_param is NULL: %d\n\n", -b);

		return FAIL;
	}

	// LTE link ready
	led_set_rgb(&led.sense, 0, 0, 0);
	printk("link ready\n");
	lte_ready = true;

	return OK;
}

ssize_t lte_send(const char * msg, size_t size)
{
	if (!msg || !size)
	{
		return FAIL;
	}

#if CONFIG_BOARD_THINGY91_NRF9160_NS
	led_set_rgb(&led.sense, 0, 0, 0);
#endif

	if (btn_0_trigger)
	{
		// toggle TCP connection
		lte_enabled = !lte_enabled;
		btn_0_trigger = 0;
	}

	if (!lte_ready && lte_enabled)
	{
		lte_ready = (OK == lte_init());

		if (!lte_ready)
		{
			return FAIL;
		}
	}

	if (so == FAIL)
	{
		if (lte_enabled)
		{
			so = tcp_connect();
		}

		if (so == FAIL)
		{
			if (btn_0_trigger)
			{
				led_set_rgb(&led.main, 0, 0, 0);
			}

			return FAIL;
		}
	}
	else if (!lte_enabled)
	{
		shutdown(so, SHUT_RDWR);
		close(so);
		so = FAIL;
		led_set_rgb(&led.main, 0, 0, 0);
		printk("TCP log disabled\n\n");

		return FAIL;
	}

	ssize_t bytes_sent = send(so, msg, size, 0);

	if (bytes_sent <= 0)
	{
		close(so);
		so = FAIL;
		led_set_rgb(&led.main, 1, 1, 0);
		printk("send failed\n\n");

		return FAIL;
	}

	led_toggle_g();

	return bytes_sent;
}
#endif
