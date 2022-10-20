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


#define CGSN_RESP_LEN 19


 /************************* SMS *********************************/
 /*
  * converts an ASCII string to an hex octet encoded septet string
  * for use with SMS service
  * inbuf: ASCII string input, no extended charactes (with coodes >= 0x80) are allowed
  * outbuf: output buffer, must be at least CEILING(7*strlen(inbuf)/4)+1, null terminated
  * outbuf_len: size of outbuf
  * returns -1 on error, number of octets on success
  */
static int ascii_to_septet(const char * inbuf, char * outbuf, size_t outbuf_len)
{
	char septet;
	int bitcount = 0;
	char bitmask = 1;
	size_t i = 0;
	size_t len = strlen(inbuf);
	int result = 0;

	if (!len)
	{
		return 0; // nothing to do
	}

	if (((7 * len) + 3) / 4 + 1 > outbuf_len)
	{
		return -1; // not enough space
	}

	while (len - 1 > i)
	{
		septet = (inbuf[i] & 0x7F) >> bitcount;
		snprintf(outbuf, 3, "%02X", septet | ((inbuf[i + 1] & bitmask) << (7 - bitcount)));
		outbuf += 2;
		result += 1;
		i += 1;

		if (7 == (i % 8))
		{
			// septet already completely consumed
			i += 1;
			// start all over
			bitcount = 0;
			bitmask = 1;
		}
		else
		{
			bitcount += 1;
			bitmask = (bitmask << 1) | 1;
		}
	}

	if (i % 8)
	{
		// left over
		septet = (inbuf[i] & 0x7F) >> bitcount;
		snprintf(outbuf, 3, "%02X", septet);
		result += 1;
	}

	return result;
}

static int pdu_sms(const char * number, const char * msg, char * pdu, size_t pdu_len)
{
	size_t count, i;
	size_t len = 8 + (((strlen(number) + 1) << 1) >> 1) + ((7 * strlen(msg)) + 3) / 4 + 1;

	if ((len > pdu_len) || (!strlen(number)))
	{
		// not enough space or no number
		return -1;
	}

	count = snprintf(pdu, pdu_len, "001100%02X91", strlen(number));
	pdu += count;
	count /= 2; // we are countng octets
	i = 0;

	while (strlen(number) - 1 > i)
	{
		pdu[i] = number[i + 1];
		pdu[i + 1] = number[i];
		i += 2;
		count += 1;
	}

	pdu += i;

	if (strlen(number) != i)
	{
		pdu[0] = 'F';
		pdu[1] = number[i];
		count += 1;
		pdu += 2;
	}

	i = snprintf(pdu, pdu_len - 2 * count, "00000B%02X", strlen(msg));
	pdu += i;
	count += i / 2;

	i = ascii_to_septet(msg, pdu, pdu_len - 2 * count);
	count += i;

	return count - 1; //first octet for SMSC is not counted
}

/* Send an SMS
 * number: international telephon number without leading +, e. g. 499113003280 for EUROS
 * msg: The SMS messsage, plain ASCII
 * pdu: a buffer for creating the AT command
 * pdu_len: length of buffer
 * return: 0 on success, -1 on failure
 */
int send_sms(const char * number, const char * msg, char * cmd_buf, size_t buf_len)
{
	int err;
	enum at_cmd_state at_state;
	char imei_buf[CGSN_RESP_LEN] = { 0 };
	size_t len = 8 + (((strlen(number) + 1) << 1) >> 1) + ((7 * strlen(msg)) + 3) / 4; // pdu length + SMSC data length
	size_t add_len = (len - 2 >= 100) ? 3 : ((len - 2 < 10) ? 1 : 2);

	if (8 + add_len + 4 + len + 9 > buf_len)
	{
		return -1; // not enough space
	}

	err = pdu_sms(number, msg, cmd_buf + 8 + add_len + 4, buf_len - 8 - add_len - 4);

	if (-1 == err)
	{
		return -1; // something went wrong
	}

	snprintf(cmd_buf, 12, "AT+CMGS=%i", err);
	cmd_buf[add_len + 8] = '<';
	cmd_buf[add_len + 9] = 'C';
	cmd_buf[add_len + 10] = 'R';
	cmd_buf[add_len + 11] = '>';
	snprintf(cmd_buf + 8 + add_len + 4 + err * 2 + 2, buf_len - 8 - add_len - 4 - 2 * err - 2, "<ctrl-z>");

	printf("%s\n\r", cmd_buf);
	err = at_cmd_write("AT+CMFG=0", imei_buf, sizeof(imei_buf), &at_state);

	printk("%i %d buf=%s\n\r", err, at_state, imei_buf);

	if (AT_CMD_OK == at_state)
	{
		err = at_cmd_write(cmd_buf, imei_buf, sizeof(imei_buf), &at_state);
		printk("%i %d buf=%s\n\r", err, at_state, imei_buf);

		if (AT_CMD_OK == at_state)
		{
			return 0;
		}
		else
		{
			printk("AT+CMFG=0  SIM card does not support SMS %u\n", at_state);
		}
	}
	else
	{
		printk("send SMS  SIM card does not support SMS %u\n", at_state);
	}

	return -1;
}
/********************** END SMS ********************************/
