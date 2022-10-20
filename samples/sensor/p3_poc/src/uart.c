/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <ctype.h>
#include <logging/log.h>
#include <drivers/uart.h>
#include <string.h>
#include <init.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/lte_lc.h>
#include <power/reboot.h>

#include "uart.h"
#include "apn.h"
#include "fota_update.h"
#include "eeprom.h"

LOG_MODULE_REGISTER(my_uart, CONFIG_AT_CMD_LOG_LEVEL);

/* Stack definition for uart host workqueue */
#define UART_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(uart_stack_area, UART_STACK_SIZE);
#define INVALID_DESCRIPTOR      -1

#define OK_STR    "OK\r\n"
#define ERROR_STR "ERROR\r\n"

#if CONFIG_AT_HOST_CMD_MAX_LEN > CONFIG_AT_CMD_RESPONSE_MAX_LEN
#define AT_BUF_SIZE CONFIG_AT_HOST_CMD_MAX_LEN
#else
#define AT_BUF_SIZE CONFIG_AT_CMD_RESPONSE_MAX_LEN
#endif

extern void modem_configure(void);

static const struct device * uart_device;
static char at_buf[AT_BUF_SIZE]; /* AT command and modem response buffer */
static struct k_work_q uart_work_q;
struct k_work uart_work;
static enum uart_modes uart_mode;
static bool inside_quotes;
static size_t at_cmd_len;
static int ptr = 0;
static int nl_ptr = 0;

char str[128];

const char update_cmd[] = "update=";
const char apn_get_cmd[] = "apn_get=";
const char apn_set_cmd[] = "apn_set=";

struct update_req
{
	int build;
	const char * checksum;
	const char * path;
	int size;
};

struct json_obj_descr update_req_descr[] =
{
	JSON_OBJ_DESCR_PRIM(struct update_req, build, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct update_req, checksum, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct update_req, path, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct update_req, size, JSON_TOK_NUMBER)
};

uint8_t strtou8(const char * s)
{
	int i;
	uint8_t res = 0;

	for (i = 0; 2 != i; ++i)
	{
		res <<= 4;

		if (('0' <= *s) && ('9' >= *s))
		{
			res |= *s - '0';
		}
		else if (('a' <= *s) && ('f' >= *s))
		{
			res |= *s - 'a' + 10;
		}
		else
		{
			// assume upper case hex
			res |= *s - 'A' + 10;
		}

		s++;
	}

	return res;
}

int uart_fota_send(const uint8_t * buf, size_t len)
{
	printk("%s\n", buf);
	return strlen(buf);
}

int uart_fota_recv(uint8_t * buf, size_t len, int look_for_newline, int flush)
{
	size_t actual_len;

	if (flush)
	{
		ptr = 0;
		nl_ptr = 0;
		inside_quotes = false;
		at_cmd_len = 0;

		return 0;
	}

	if (look_for_newline)
	{
		memcpy(buf, at_buf, nl_ptr);
		return nl_ptr;
	}

	actual_len = MIN(len, at_cmd_len - ptr);

	memcpy(buf, at_buf + ptr, actual_len);
	ptr += actual_len;

	return actual_len;
}

void uart_fota_finish(void)
{
	uart_set_mode(CMD_MODE);
}

static inline void write_uart_string(const char * str)
{
	/* Send characters until, but not including, null */
	for (size_t i = 0; str[i]; i++)
	{
		uart_poll_out(uart_device, str[i]);
	}
}

void uart_worker(struct k_work * work)
{
	enum at_cmd_state state;
	int               err, res;
	size_t            len = strlen(at_buf);
	char              cmd_reset[] = "reset";
	//char              cmd_button[] = "button";

	ARG_UNUSED(work);

	if ((sizeof(cmd_reset) - 1) == len)
	{
		if (!memcmp(at_buf, cmd_reset, len))
		{
			LOG_INF("Reset board");
			sys_reboot(SYS_REBOOT_WARM);
		}
	}

	/*if ((sizeof(cmd_button) - 1) == len)
	{
		if (!memcmp(at_buf, cmd_button, len))
		{
			button_int();
		}
	}*/

	if ((len >= 2) && ('A' == at_buf[0]) && ('T' == at_buf[1]))
	{
		/* Assume we are dealing with an AT command */

		err = at_cmd_write(at_buf, at_buf, sizeof(at_buf), &state);

		if (err < 0)
		{
			LOG_ERR("Error while processing AT command: %d", err);
			state = AT_CMD_ERROR;
		}

		/* Handle the various error responses from modem */
		switch (state)
		{
		case AT_CMD_OK:
			write_uart_string(at_buf);
			write_uart_string(OK_STR);
			break;

		case AT_CMD_ERROR:
			write_uart_string(ERROR_STR);
			break;

		case AT_CMD_ERROR_CMS:
			sprintf(str, "+CMS ERROR: %d\r\n", err);
			write_uart_string(str);
			break;

		case AT_CMD_ERROR_CME:
			sprintf(str, "+CME ERROR: %d\r\n", err);
			write_uart_string(str);
			break;

		default:
			break;
		}
	}
	else if (0 == memcmp(update_cmd, at_buf, sizeof(update_cmd) - 1))
	{
		struct update_req req;

		res = json_obj_parse(
			at_buf + sizeof(update_cmd) - 1,
			len,
			update_req_descr,
			4,
			&req
		);

		char check[20];

		// at least size (bit 3) and checksum (bit 1) must be present
		if ((0 < res) && ((0xA & res) == 0xA))
		{
			for (len = 0; 20 != len; ++len)
			{
				check[len] = strtou8(&req.checksum[2 * len]);
			}

			uart_set_mode(NEWLINE_MODE);
			uart_irq_rx_enable(uart_device);
			fota_uart_start(req.size, check, uart_fota_finish);
		}
	}
	else if (0 == memcmp(apn_get_cmd, at_buf, sizeof(apn_get_cmd) - 1))
	{
		apn_get(
			at_buf + sizeof(apn_get_cmd) - 1,
			len,
			&apn_config
		);
	}
	else if (0 == memcmp(apn_set_cmd, at_buf, sizeof(apn_set_cmd) - 1))
	{
		err = apn_set(at_buf + sizeof(apn_set_cmd) - 1, len, &apn_config);

		if (OK == err)
		{

			snprintf(
				str,
				sizeof(str),
				"apn_eeprom={\"select\": %d, \"APN\": \"%s\"}\n",
				apn_config.select,
				apn_config.APN
			);

			uart_fota_send(str, strlen(str));
			lte_lc_power_off();
			lte_lc_deinit();
			modem_configure();
		}
	}

	uart_irq_rx_enable(uart_device);
}

static void uart_rx_handler(uint8_t character)
{
	if (RAW_MODE == uart_mode)
	{
		at_buf[at_cmd_len] = character;
		at_cmd_len++;
		return;
	}

	if (NEWLINE_MODE == uart_mode)
	{
		if ((!nl_ptr) && (('\r' == character) || ('\n' == character)))
		{
			nl_ptr = at_cmd_len;
			ptr = at_cmd_len;
		}
		else
		{
			at_buf[at_cmd_len] = character;
			at_cmd_len++;
		}

		return;
	}

	/* Handle control characters */
	switch (character)
	{
	case 0x08: /* Backspace. */
		/* Fall through. */
	case 0x7F: /* DEL character */
		if (at_cmd_len > 0)
		{
			at_cmd_len--;
		}

		return;
	}

	/* Handle termination characters, if outside quotes. */
	if (!inside_quotes)
	{
		if (('\r' == character) || ('\n' == character))
		{
			goto send;
		}
	}

	/* Detect AT command buffer overflow, leaving space for null */
	if (at_cmd_len + 1 > sizeof(at_buf) - 1)
	{
		LOG_ERR("Buffer overflow, dropping '%c'\n", character);
		return;
	}

	/* Write character to AT buffer */
	at_buf[at_cmd_len] = character;
	at_cmd_len++;

	/* Handle special written character */
	if (character == '"')
	{
		inside_quotes = !inside_quotes;
	}

	return;

send:
	/* Terminate the command string */
	at_buf[at_cmd_len] = '\0';

	/* Reset UART handler state */
	inside_quotes = false;
	at_cmd_len = 0;

	/* Check for the presence of one printable non-whitespace character */
	for (const char * c = at_buf;; c++)
	{
		if (*c > ' ')
		{
			break;
		}
		else if (*c == '\0')
		{
			/* Drop command, if it has no such character */
			return;
		}
	}

	/* Send the command, if there is one to send */
	if (at_buf[0])
	{
		uart_irq_rx_disable(uart_device); /* Stop UART to protect at_buf */
		k_work_submit_to_queue(&uart_work_q, &uart_work);
	}
}

void my_uart_isr(const struct device * dev, void *data)
{
	uint8_t character;

	uart_irq_update(dev);

	if (!uart_irq_rx_ready(dev))
	{
		return;
	}

	/*
	 * Check that we are not sending data (buffer must be preserved then),
	 * and that a new character is available before handling each character
	 */
	while (
		(!k_work_is_pending(&uart_work)) &&
		(uart_fifo_read(dev, &character, 1))
		)
	{
		uart_rx_handler(character);
	}
}

void my_uart_init(const struct device * uart_dev)
{
	uart_device = uart_dev;
	uart_mode = CMD_MODE;
	inside_quotes = false;
	uart_irq_callback_set(uart_dev, my_uart_isr);
	uart_irq_rx_enable(uart_dev);
	k_work_init(&uart_work, uart_worker);

	k_work_queue_start(
		&uart_work_q,
		uart_stack_area,
		K_THREAD_STACK_SIZEOF(uart_stack_area),
		CONFIG_AT_CMD_THREAD_PRIO,
		NULL
	);
}

void uart_set_mode(enum uart_modes mode)
{
	uart_mode = mode;
	inside_quotes = false;
	ptr = 0;
	nl_ptr = 0;
	at_cmd_len = 0;
}
