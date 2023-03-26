/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

 /** @file
  *  @brief Nordic UART Bridge Service (NUS) sample
  */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <soc.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>
#include "uart_ble.h"

#define LOG_MODULE_NAME uart_ble
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 1000

//-#define UART_BUF_COUNT 8
#define UART_BUF_COUNT 16
#define UART_BUF_SIZE 512//CONFIG_BRIDGE_BUF_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT_MS 1

static K_SEM_DEFINE(ble_init_ok, 0, 1);

const static struct device * uart = NULL;
static struct k_work_delayable uart_work;

struct uart_data_t
{
	void * fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static struct uart_data_t buffers_rx[UART_BUF_COUNT];
static int buffers_rx_index = 0;

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);


//static struct uart_data_t * get_buffers_tx()
//{
//	int index = buffers_tx_index++;
//
//	if (buffers_tx_index >= sizeof(buffers_tx) / sizeof(*buffers_tx))
//	{
//		buffers_tx_index = 0;
//	}
//
//	return &buffers_tx[index];
//}

static struct uart_data_t * get_buffers_rx()
{
	int index = buffers_rx_index++;

	if (buffers_rx_index >= sizeof(buffers_rx) / sizeof(*buffers_rx))
	{
		buffers_rx_index = 0;
	}

	return &buffers_rx[index];
}

static void uart_cb(const struct device * dev, struct uart_event * evt, void * user_data)
{
	ARG_UNUSED(dev);

	static uint8_t * current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t * buf;
	static uint8_t * aborted_buf;

	switch (evt->type)
	{
	case UART_TX_DONE:
		if ((evt->data.tx.len == 0) || (!evt->data.tx.buf))
		{
			return;
		}

		if (aborted_buf)
		{
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);

		if (!buf)
		{
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
		{
			printk("Failed to send data over UART\n");
		}

		break;

	case UART_RX_RDY:
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;
		buf_release = false;

		if (buf->len == UART_BUF_SIZE)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
		}
		else if (
			(evt->data.rx.buf[buf->len - 1] == '\n') ||
			(evt->data.rx.buf[buf->len - 1] == '\r')
			)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
			current_buf = evt->data.rx.buf;
			buf_release = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		buf = get_buffers_rx();

		if (buffers_rx_index >= sizeof(buffers_rx) / sizeof(*buffers_rx))
		{
			buffers_rx_index = 0;
		}

		if (buf)
		{
			buf->len = 0;
		}
		else
		{
			printk("UART_RX_DISABLED: Not able to allocate UART receive buffer\n");
			k_work_schedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT_MS);

		break;

	case UART_RX_BUF_REQUEST:
		buf = get_buffers_rx();

		if (buf)
		{
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		}
		else
		{
			printk("UART_RX_BUF_REQUEST: Not able to allocate UART receive buffer\n");
		}

		break;

	case UART_RX_BUF_RELEASED:
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data);

		if (buf_release && (current_buf != evt->data.rx_buf.buf))
		{
			buf_release = false;
			current_buf = NULL;
		}

		break;

	case UART_TX_ABORTED:
		if (!aborted_buf)
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);

		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work * item)
{
	struct uart_data_t * buf;

	buf = get_buffers_rx();

	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		printk("uart_work_handler: Not able to allocate UART receive buffer\n");
		k_work_schedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);

		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT_MS);
}

static int uart_init(void)
{
	int err;
	struct uart_data_t * rx;

	uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

	if (!uart)
	{
		return -ENXIO;
	}

	rx = get_buffers_rx();

	if (rx)
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	err = uart_callback_set(uart, uart_cb, NULL);

	if (err)
	{
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
}

void main_1(void)
{
	int err = 0;

	err = uart_init();

	if (err)
	{
		return;
	}

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	printk("Starting Nordic UART service example\n");

	for (;;)
	{
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

int uart_ble_send(const char * msg, uint16_t length)
{
	if (!uart)
	{
		int err = uart_init();

		if (err)
		{
			printk("Failed to init UART\n");
			return -1;
		}
	}

	struct uart_data_t * tx = k_malloc(sizeof(*tx));

	if (!tx)
	{
		printk("uart_ble_send: unable to allocate UART send buffer\n");
		return -1;
	}

	tx->len = MIN(length, sizeof(tx->data) - 1);

	memcpy(tx->data, msg, tx->len);

	if (uart_tx(uart, tx->data, length, SYS_FOREVER_MS))
	{
		k_fifo_put(&fifo_uart_tx_data, tx);
	}

	return 0;
}

/*
void ble_write_thread(void)
{
	// Don't go any further until BLE is initialized
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;)
	{
		// Wait indefinitely for data to be sent over bluetooth
		struct uart_data_t * buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

		//if (bt_gatt_nus_send(NULL, buf->data, buf->len))
		//{
		//	printk("Failed to send data over BLE connection\n");
		//}

		//-k_free(buf);
	}
}

//K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
*/
