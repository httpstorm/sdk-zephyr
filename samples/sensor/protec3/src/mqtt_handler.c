#include <zephyr.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/at_cmd.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <stdio.h>
#include <net/tls_credentials.h>
#include <modem/modem_key_mgmt.h>
#include <net/mqtt.h>

#include "gpio_t.h"
#include "main.h"
#include "aws.h"
#include "mqtt_handler.h"


// flags
uint8_t alarm_arm = MQTT_REQUEST_OFF;

/* worker_queue for handling received data */
#define MQTT_WORK_COUNT 4
static struct k_work_q mqtt_work_q;
static uint8_t work_index = 0;
mqtt_work_piece_t mqtt_work[MQTT_WORK_COUNT];

/* storage for up to 4 MAC addresses in human readable format */
#define MAX_NUMBER_OF_USERS 4

/* 01-23-45-67-89-AB: 17 charcters */
#define MAX_USER_NAME_LEN 17
static char users[MAX_NUMBER_OF_USERS][MAX_USER_NAME_LEN + 1] = { "!", "!", "!", "!" };

#define MQTT_STACK_SIZE 1024
#define MQTT_THREAD_PRIO 10

K_THREAD_STACK_DEFINE(mqtt_stack_area, MQTT_STACK_SIZE);


// payload must be NULL terminated
enum MQTT_REQUEST mqtt_data_get(const char * payload, size_t count)
{
	if (count == 1)
	{
		enum MQTT_REQUEST r = (enum MQTT_REQUEST)(*payload - '0');

		if ((uint8_t)r >= MQTT_REQUEST__END_MARKER)
		{
			r = MQTT_REQUEST_INVALID;
		}

		return r;
	}

	if (count == 2)
	{
		if (*(uint16_t *)payload == *(uint16_t *)"ON")
		{
			return MQTT_REQUEST_ON;
		}
	}

	if (count == 3)
	{
		if (*(uint32_t *)payload == *(uint32_t *)"OFF")
		{
			return MQTT_REQUEST_OFF;
		}
		else if (*(uint32_t *)payload == *(uint32_t *)"GET")
		{
			return MQTT_REQUEST_GET;
		}
	}

	return MQTT_REQUEST_INVALID;
}

int mqtt_publish_h(struct mqtt_client * client, const char * topic, size_t topic_count, const char * payload, size_t payload_count)
{
	// topic:
	// /nrf-123456789abcdef/item
	// /nrf-123456789abcdef/group/item
	// item should be at least one character long
	if (topic_count < (CLIENT_ID_LEN + 2))
	{
		return FAIL;
	}

	// /client-id/ is surrounded by /
	if ((*topic != '/') || (topic[CLIENT_ID_LEN] != '/'))
	{
		return FAIL;
	}

	// client-id refers to us
	if (memcmp(topic + 1, client_id_buf, CLIENT_ID_LEN - 1))
	{
		return FAIL;
	}

	strncpy(mqtt_work[work_index].subtopic, topic + CLIENT_ID_LEN + 1, SUBTOPIC_MAX_LENGTH);
	mqtt_work[work_index].subtopic_len = topic_count - CLIENT_ID_LEN - 1;
	strncpy(mqtt_work[work_index].payload, payload, PAYLOAD_MAX_LENGTH);
	mqtt_work[work_index].payload_len = payload_count;
	mqtt_work[work_index].client = client;
	k_work_submit_to_queue(&mqtt_work_q, &mqtt_work[work_index].work);

	work_index = (work_index + 1) % MQTT_WORK_COUNT;

	return OK;
}

void mqtt_worker(struct k_work * work)
{
	char buf[32];
	mqtt_work_piece_t * mqtt_work_piece = CONTAINER_OF(work, mqtt_work_piece_t, work);

	// item
	// group/item
	const char * item = mqtt_work_piece->subtopic;
	size_t count = mqtt_work_piece->subtopic_len;

	// try to get the payload as an enum
	enum MQTT_REQUEST request = mqtt_data_get(
		mqtt_work_piece->payload,
		mqtt_work_piece->payload_len
	);


	const char alarm[] = "alarm";

	if (
		(count == sizeof(alarm) - 1) &&
		!memcmp(item, alarm, sizeof(alarm) - 1)
		)
	{
		// alarm
		if ((uint32_t)request > MQTT_REQUEST_ON)
		{
			if ((uint32_t)request != MQTT_REQUEST_GET)
			{
				return;
			}
		}
		else
		{
			// change alarm state
			alarm_arm = request;
		}

		printk("/status/alarm %u\n", alarm_arm);

		data_publish1(
			mqtt_work_piece->client,
			MQTT_QOS_1_AT_LEAST_ONCE,
			"/status/alarm",
			alarm_arm ? "ON" : "OFF"
		);

		return;
	}


	const char beep[] = "beep";

	if (
		(count > sizeof(beep)) &&
		(item[sizeof(beep) - 1] == '/') &&
		!memcmp(item, beep, sizeof(beep) - 1)
		)
	{
		// beep/
		item += sizeof(beep);
		count -= sizeof(beep);

		const char loud[] = "loud";
		const char normal[] = "normal";

		int time = atoi(mqtt_work_piece->payload);

		if (time <= 0)
		{
			return;
		}

		if (time > BEEP_TIME_MAX)
		{
			time = BEEP_TIME_MAX;
		}

		if (
			(count == (sizeof(loud) - 1)) &&
			!memcmp(item, loud, sizeof(loud) - 1)
			)
		{
			// beep/loud

			set_beep_time(BEEP_TYPE_LOUD, time);
			printk("/status/beep/loud %u ms\n", time);

			return;
		}

		if (
			(count == (sizeof(normal) - 1)) &&
			!memcmp(item, normal, sizeof(normal) - 1)
			)
		{
			// beep/normal

			set_beep_time(BEEP_TYPE_NORMAL, time);
			printk("/status/beep/normal %u ms\n", time);

			return;
		}

		return;
	}


	const char led_[] = "led/";

	if (
		(count == sizeof(led_)) &&
		(*(const uint32_t *)item == *(const uint32_t *)led_)
		)
	{
		// led/[0-2] = OFF|ON
		item += sizeof(led_) - 1;
		count -= sizeof(led_) - 1;
		const uint8_t index = *item - '0';

		if (index >= LED_COUNT)
		{
			return;
		}

		if ((uint32_t)request > MQTT_REQUEST_ON)
		{
			if ((uint32_t)request == MQTT_REQUEST_GET)
			{
				request = pin_get(led.list[index]);
			}
			else
			{
				return;
			}
		}

		pin_set(led.list[index], request);

		printk("/status/led/%u = %u\n", index, request);
		snprintf(buf, sizeof(buf), "/status/led/%u", index);

		data_publish1(
			mqtt_work_piece->client,
			MQTT_QOS_1_AT_LEAST_ONCE,
			buf,
			request ? "ON" : "OFF"
		);

		return;
	}


	const char user[] = "user";

	if (
		(count > sizeof(user)) &&
		(item[sizeof(user) - 1] == '/') &&
		!memcmp(item, user, sizeof(user) - 1)
		)
	{
		// user/
		item += sizeof(user);
		count -= sizeof(user);

		const char add[] = "add";
		const char del[] = "del";
		const char list[] = "list";

		if (
			(count == (sizeof(add) - 1)) &&
			!memcmp(item, add, sizeof(add) - 1)
			)
		{
			// user/add

			int success = add_user(mqtt_work_piece->payload);
			printk("/status/user/add %s %d\n", mqtt_work_piece->payload, success);

			data_publish1(
				mqtt_work_piece->client,
				MQTT_QOS_1_AT_LEAST_ONCE,
				"/status/user/add",
				OK == success ? "OK" : "FAIL"
			);

			return;
		}

		if (
			(count == (sizeof(del) - 1)) &&
			!memcmp(item, del, sizeof(del) - 1)
			)
		{
			// user/del

			int success = del_user(mqtt_work_piece->payload);
			printk("/status/user/del %s %d\n", mqtt_work_piece->payload, success);

			data_publish1(
				mqtt_work_piece->client,
				MQTT_QOS_1_AT_LEAST_ONCE,
				"/status/user/del",
				OK == success ? "OK" : "FAIL"
			);

			return;
		}

		if (
			(count == (sizeof(list) - 1)) &&
			!memcmp(item, list, sizeof(list) - 1)
			)
		{
			// user/list

			int i, j;

			for (i = 0, j = 0; MAX_NUMBER_OF_USERS > i; ++i)
			{
				if ('!' != users[i][0])
				{
					printk("/status/list/%d %s\n", j - 1, users[i]);

					snprintf(
						buf,
						sizeof(buf),
						"/status/user/list/%d",
						j++
					);

					data_publish1(
						mqtt_work_piece->client,
						MQTT_QOS_1_AT_LEAST_ONCE,
						buf,
						users[i]
					);
				}
			}

			return;
		}

		return;
	}


	const char uptime[] = "uptime";

	if (
		(count == sizeof(uptime) - 1) &&
		!memcmp(item, uptime, sizeof(uptime) - 1)
		)
	{
		// uptime
		if ((uint32_t)request != MQTT_REQUEST_GET)
		{
			return;
		}

		int64_t time = k_uptime_get() / 1000;

		/* %lld doesn't work in sprintf, so we must do it ourselves */
		if (0 == time)
		{
			snprintf(buf, sizeof(buf), "0");
		}
		else
		{
			int i = 1;
			buf[sizeof(buf) - 1] = '\0';

			while ((time) && (sizeof(buf) > i))
			{
				char digit = (time % 10) + '0';
				buf[sizeof(buf) - 1 - i] = digit;
				++i;
				time /= 10;
			}

			memmove(buf, &buf[sizeof(buf) - i], i);
		}

		printk("/status/uptime %s seconds\n", buf);

		data_publish1(
			mqtt_work_piece->client,
			MQTT_QOS_1_AT_LEAST_ONCE,
			"/status/uptime",
			buf
		);
	}
}

void mqtt_work_queue_init()
{
	/* worker_queue to handle received data */
	k_work_init(&mqtt_work[0].work, mqtt_worker);
	k_work_init(&mqtt_work[1].work, mqtt_worker);
	k_work_init(&mqtt_work[2].work, mqtt_worker);
	k_work_init(&mqtt_work[3].work, mqtt_worker);

	k_work_queue_start(
		&mqtt_work_q,
		mqtt_stack_area,
		K_THREAD_STACK_SIZEOF(mqtt_stack_area),
		MQTT_THREAD_PRIO,
		NULL
	);
}

int add_user(const char * mac)
{
	// TODO: prevent duplicates
	int i = 0;

	if ('!' == mac[0])
	{
		return FAIL; // reserved
	}

	for (i = 0; MAX_NUMBER_OF_USERS > i; ++i)
	{
		if ('!' == users[i][0])
		{
			strncpy(users[i], mac, MAX_USER_NAME_LEN + 1);
			return OK;
		}
	}

	return FAIL;
}

int del_user(const char * mac)
{
	int i = 0;

	if ('!' == mac[0])
	{
		return FAIL; // reserved
	}

	for (i = 0; MAX_NUMBER_OF_USERS > i; ++i)
	{
		if (0 == strncmp(users[i], mac, MAX_USER_NAME_LEN + 1))
		{
			memset(users[i], 0, MAX_USER_NAME_LEN + 1);
			users[i][0] = '!';

			return OK;
		}
	}

	return FAIL;
}

int mqtt_pub_btn_press(void)
{
	return data_publish1(
		&client,
		MQTT_QOS_1_AT_LEAST_ONCE,
		"/button",
		"ON"
	);
}

int mqtt_pub(const char * topic, /*const */char * msg)
{
	return data_publish1(
		&client,
		MQTT_QOS_1_AT_LEAST_ONCE,
		topic,
		msg
	);
}
