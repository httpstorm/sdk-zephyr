/*
 * Copyright (c) 2020 EUROS Embedded Systems GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if CONFIG_NETWORKING
#include <zephyr.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <stdio.h>
#include <string.h>

 // LTE modem
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>

#include "gpio_t.h"


#if defined(CONFIG_LTE_NETWORK_MODE_NBIOT_GPS)
# define CONFIG_TCP_SERVER_ADDRESS_STATIC "81.161.241.2" // gvalkov.com
#else
# define CONFIG_TCP_SERVER_ADDRESS_STATIC "87.138.172.131" // euros-embedded.com
#endif
#define CONFIG_TCP_SERVER_PORT 81


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

int tcp_connect_disconnect(int connect)
{
	lte_enabled = connect;

#if 0
	// lte_lc_init_and_connect can be called only once
	// and will return an error on following attempts
	if (!lte_ready && lte_enabled)
	{
		lte_ready = (OK == lte_init());

		if (!lte_ready)
		{
			return FAIL;
		}
	}
#endif

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

	return OK;
}

ssize_t lte_send(const char * msg, size_t size)
{
	if (!msg || !size)
	{
		return FAIL;
	}

	if (btn_0_trigger)
	{
		// toggle TCP connection
		lte_enabled = !lte_enabled;
		btn_0_trigger = 0;

#if CONFIG_BOARD_THINGY91_NRF9160_NS
		led_set_rgb(&led.sense, 0, 0, 0);
#endif
	}

	if (OK != tcp_connect_disconnect(lte_enabled))
	{
		return FAIL;
	}

	pin_set(led.main.g, 1);
	ssize_t bytes_sent = send(so, msg, size, 0);

	if (bytes_sent <= 0)
	{
		close(so);
		so = FAIL;
		led_set_rgb(&led.main, 1, 1, 0);
		printk("send failed\n\n");

		return FAIL;
	}

	pin_set(led.main.g, 0);

	return bytes_sent;
}
#endif
