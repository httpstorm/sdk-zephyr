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
#include "mqtt_handler.h"
#include "main.h"
#include "aws.h"


#define CONFIG_COAP_SERVER_PORT 61002
#define CONFIG_COAP_SERVER_HOSTNAME "84.40.94.129"

LOG_MODULE_REGISTER(aws, CONFIG_PROTEC3_LOG_LEVEL);

static struct sockaddr_storage server;

static struct rsrp_data rsrp =
{
	.value = 0,
	.offset = MODEM_INFO_RSRP_OFFSET_VAL,
};

static struct modem_param_info modem_param;

static sec_tag_t sec_tag_list[] = { 13496857 };

static const char ca_cert[] =
{
	#include "../certs/ca.crt"
};

static const char private_cert[] =
{
	#include "../certs/client.crt"
};

static const char private_key[] =
{
	#include "../certs/client.key"
};

/* Buffers for MQTT client. */
static uint8_t rx_buffer[MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t tx_buffer[MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t payload_buf[MQTT_PAYLOAD_BUFFER_SIZE + 1];

/* The mqtt client struct */
struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* Connected flag */
static bool connected;

/* File descriptor */
static struct pollfd fds;

/************************************************ Functions ********************************************/

static int server_resolve(void)
{
	int err;
	struct addrinfo * result;

	struct addrinfo hints =
	{
		.ai_flags = AI_NUMERICHOST,
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM
	};

	char ipv4_addr[NET_IPV4_ADDR_LEN];

	err = getaddrinfo(CONFIG_COAP_SERVER_HOSTNAME, NULL, &hints, &result);

	if (err != 0)
	{
		LOG_ERR("ERROR: getaddrinfo failed %d\n", err);
		return -EIO;
	}

	if (result == NULL)
	{
		LOG_ERR("ERROR: Address not found\n");
		return -ENOENT;
	}

	struct sockaddr_in * server4 = ((struct sockaddr_in *)&server);

	server4->sin_addr.s_addr = ((struct sockaddr_in *)result->ai_addr)->sin_addr.s_addr;
	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_COAP_SERVER_PORT);

	inet_ntop(AF_INET, &server4->sin_addr.s_addr, ipv4_addr, sizeof(ipv4_addr));
	printk("IPv4 Address found %s\n", ipv4_addr);

	freeaddrinfo(result);

	return 0;
}

static void modem_rsrp_handler(char rsrp_value)
{
	if (rsrp_value > 97)
	{
		return;
	}

	rsrp.value = rsrp_value;

	char buffer[CONFIG_MODEM_INFO_BUFFER_SIZE] = { 0 };
	int32_t rsrp_current;
	size_t len;
	rsrp_current = rsrp.value - rsrp.offset;

	len = snprintf(buffer, CONFIG_MODEM_INFO_BUFFER_SIZE, "signal strength: %d dBm", rsrp_current);
	printk("%s\n", buffer);

}

static void modem_data_init(void)
{
	int err;
	err = modem_info_init();

	if (err)
	{
		printk("Modem info could not be established: %d\n", err);
		return;
	}

	modem_info_params_init(&modem_param);
	modem_info_rsrp_register(modem_rsrp_handler);
}

int cert_provision1(nrf_sec_tag_t sec_tag, enum modem_key_mgmt_cred_type cred_type, const void * buf, size_t len)
{
	int err;
	bool exists;
	uint8_t unused;

	err = modem_key_mgmt_exists(
		sec_tag,
		cred_type,
		&exists,
		&unused
	);

	if (err)
	{
		printk("Failed to check for certificates err %d\n", err);
		return err;
	}

	//	if (exists)
	//	{
	//		/* For the sake of simplicity we delete what is provisioned
	//		 * with our security tag and reprovision our certificate.
	//		 */
	//		err = modem_key_mgmt_delete(sec_tag, cred_type);
	//
	//		if (err)
	//		{
	//			printk("Failed to delete existing certificate, err %d\n", err);
	//		}
	//	}


	if (!exists)
	{
		printk("Provisioning certificate\n");

		/*  Provision certificate to the modem */
		err = modem_key_mgmt_write(sec_tag, cred_type, buf, len - 1);

		if (err)
		{
			printk("Failed to provision certificate, err %d\n", err);
			return err;
		}
	}
	else
	{
		printk("Certificate already provisioned\n");
	}


	return 0;
}

/* TODO should be a libary function, but I couldn't get the library included
   cf. https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/random/index.html?highlight=sys_rand#c.sys_rand32_get
 */
uint32_t sys_rand32_get(void)
{
	/* algorithm from EUROS soruces */
	static uint32_t seed = 0;

	if (!seed)
	{
		seed = k_uptime_get_32();
	}

	seed = seed * 1103515245L + 12345L;

	return seed >> 16;
}

/**@brief Function to print strings without null-termination
 */
static void data_print(const uint8_t * prefix, const uint8_t * data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}

int data_publish1(
	struct mqtt_client * c,
	enum mqtt_qos qos,
	const uint8_t * subtopic,
	/*const */uint8_t * data
)
{
	uint8_t topic[64];
	struct mqtt_publish_param param;

	snprintf(topic, sizeof(topic), "/%s%s", client_id_buf, subtopic);

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = topic;
	param.message.topic.topic.size = strlen(topic);
	param.message.payload.data = data;
	param.message.payload.len = strlen(data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	printk("MQTT PUB: %s = ", topic);
	data_print("", data, strlen(data));

	return mqtt_publish(c, &param);
}



/**@brief Function to subscribe to the configured topic
 */
static int subscribe(void)
{
	uint8_t alarm[64];
	uint8_t beep[64];
	uint8_t led_[64];
	uint8_t user[64];
	uint8_t uptime[64];

	snprintf(alarm, sizeof(alarm), "/%s/alarm", client_id_buf);
	snprintf(beep, sizeof(beep), "/%s/beep/+", client_id_buf);
	snprintf(led_, sizeof(led_), "/%s/led/+", client_id_buf);
	snprintf(user, sizeof(user), "/%s/user/+", client_id_buf);
	snprintf(uptime, sizeof(uptime), "/%s/uptime", client_id_buf);

	struct mqtt_topic subscribe_topics[] =
	{
		{
			.topic =
			{
				.utf8 = alarm,
				.size = strlen(alarm)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = beep,
				.size = strlen(beep)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = led_,
				.size = strlen(led_)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = user,
				.size = strlen(user)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = uptime,
				.size = strlen(uptime)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		// TODO add additional topics
	};

	size_t count = sizeof(subscribe_topics) / sizeof(subscribe_topics[0]);

	const struct mqtt_subscription_list subscription_list =
	{
		.list = subscribe_topics,
		.list_count = count,
		.message_id = 1234
	};

	printk("\nMQTT subscribing to %u topics:\n", count);

	size_t i = 0;

	for (i = 0; i < count; i++)
	{
		printk("  %s\n", subscribe_topics[i].topic.utf8);
	}

	printk("\n");

	return mqtt_subscribe(&client, &subscription_list);
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client * c, size_t length)
{
	// reserve space for a NULL terminator
	if (length >= sizeof(payload_buf))
	{
		return -EMSGSIZE;
	}

	uint8_t * buf = payload_buf;
	uint8_t * end = buf + length;

	while (buf < end)
	{
		int count = mqtt_read_publish_payload(c, buf, end - buf);

		if (count < 0)
		{
			int err;

			if (count != -EAGAIN)
			{
				return count;
			}

			printk("mqtt_read_publish_payload: EAGAIN\n");

			err = poll(&fds, 1, CONFIG_MQTT_KEEPALIVE * MSEC_PER_SEC);

			if (err > 0 && (fds.revents & POLLIN) == POLLIN)
			{
				continue;
			}
			else
			{
				return -EIO;
			}
		}

		if (count == 0)
		{
			return -EIO;
		}

		buf += count;
	}

	// add a NULL terminator
	*buf = '\0';

	return 0;
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client * const c, const struct mqtt_evt * evt)
{
	int err;

	switch (evt->type)
	{
	case MQTT_EVT_CONNACK:
		if (evt->result != 0)
		{
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}

		connected = true;
		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
		int b = subscribe();

		printk(
			"MQTT mqtt_subscribe %i  %s\n",
			b,
			(b == -12) ? "FAIL: check MQTT_MESSAGE_BUFFER_SIZE" : b ? "FAIL" : "OK"
		);

		data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/link", "ONLINE");

		break;

	case MQTT_EVT_DISCONNECT:
		printk(
			"[%s:%d] MQTT client disconnected %d\n",
			__func__,
			__LINE__,
			evt->result
		);

		connected = false;

		break;

	case MQTT_EVT_PUBLISH:
	{
		const struct mqtt_publish_param * p = &evt->param.publish;

		uint8_t topic[p->message.topic.topic.size + 1];
		snprintf(topic, sizeof(topic), "%s", p->message.topic.topic.utf8);

		printk(
			"[%s:%d] MQTT PUBLISH result=%d len=%d  %s = ",
			__func__,
			__LINE__,
			evt->result,
			p->message.payload.len,
			topic
		);

		err = publish_get_payload(c, p->message.payload.len);

		if (err >= 0)
		{
			printk("%s\n", payload_buf);
			mqtt_publish_h(&client, topic, strlen(topic), payload_buf, p->message.payload.len);

#if 0
			data_publish1(
				&client,
				MQTT_QOS_1_AT_LEAST_ONCE,
				"/status/echo",
				payload_buf
				/*, p->message.payload.len*/
			);
#endif
		}
		else
		{
			printk("ERROR %d\nmqtt_read_publish_payload: Failed!\n", err);
			printk("Disconnecting MQTT client...\n");

			err = mqtt_disconnect(c);

			if (err)
			{
				printk("Could not disconnect: %d\n", err);
			}
		}
	}

	break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0)
		{
			printk("MQTT PUBACK error %d\n", evt->result);
			break;
		}

		printk(
			"[%s:%d] PUBACK packet id: %u\n",
			__func__,
			__LINE__,
			evt->param.puback.message_id
		);

		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0)
		{
			printk("MQTT SUBACK error %d\n", evt->result);
			break;
		}

		printk(
			"[%s:%d] SUBACK packet id: %u\n",
			__func__,
			__LINE__,
			evt->param.suback.message_id
		);

		break;

	default:
		printk(
			"[%s:%d] default: %d\n",
			__func__,
			__LINE__,
			evt->type
		);

		break;
	}
}

/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static void broker_init(void)
{
	int err;
	struct addrinfo * result;
	struct addrinfo * addr;
	struct addrinfo hints =
	{
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};


	struct nrf_in_addr dns;
	dns.s_addr = 0x08080808; // Google DNS, 8.8.8.8
	err = nrf_setdnsaddr(NRF_AF_INET, &dns);
	printk("LTE| set DNS: %d\n", err);

	err = getaddrinfo(MQTT_BROKER_HOSTNAME, NULL, &hints, &result);

	if (err)
	{
		printk("ERROR: getaddrinfo failed %d\n", err);

		return;
	}

	addr = result;
	err = -ENOENT;

	/* Look for address of the broker. */
	while (addr != NULL)
	{
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in))
		{
			struct sockaddr_in * broker4 = ((struct sockaddr_in *)&broker);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr = ((struct sockaddr_in *)addr->ai_addr)->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(MQTT_BROKER_PORT);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr, ipv4_addr, sizeof(ipv4_addr));
			printk("IPv4 Address found %s\n", ipv4_addr);

			break;
		}
		else
		{
			printk(
				"ai_addrlen = %u should be %u or %u\n",
				(unsigned int)addr->ai_addrlen,
				(unsigned int)sizeof(struct sockaddr_in),
				(unsigned int)sizeof(struct sockaddr_in6)
			);
		}

		addr = addr->ai_next;
		break;
	}

	/* Free the address. */
	freeaddrinfo(result);
}

/**@brief Initialize the MQTT client structure
 */
static void client_init(struct mqtt_client * client)
{
	static struct mqtt_utf8 password;
	static struct mqtt_utf8 user_name;
	static struct mqtt_utf8 will_message;
	static struct mqtt_topic will_topic;
	static struct mqtt_utf8 will_topic_topic;

	static uint8_t client_id[64];
	static uint8_t topic[64];

	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;

	snprintf(client_id, sizeof(client_id), "protec3-%s", client_id_buf);
	client->client_id.utf8 = (uint8_t *)client_id;
	client->client_id.size = strlen(client_id);

	password.utf8 = MQTT_CLIENT_PASSWORD;
	password.size = strlen(password.utf8);
	client->password = &password;

	user_name.utf8 = (uint8_t *)MQTT_CLIENT_USERNAME;
	user_name.size = strlen(user_name.utf8);
	client->user_name = &user_name;

	will_message.utf8 = (uint8_t *)"OFFLINE";
	will_message.size = strlen(will_message.utf8);
	client->will_message = &will_message;

	snprintf(topic, sizeof(topic), "/%s/status/link", client_id_buf);
	will_topic_topic.utf8 = topic;
	will_topic_topic.size = strlen(topic);

	will_topic.qos = MQTT_QOS_2_EXACTLY_ONCE;
	will_topic.topic = will_topic_topic;
	client->will_topic = &will_topic;

	client->protocol_version = MQTT_VERSION_3_1_1;

	//printk("username: %s, password: %s\n", client->user_name->utf8, client->password->utf8);

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
	struct mqtt_sec_config * tls_config = &client->transport.tls.config;

	client->transport.type = MQTT_TRANSPORT_SECURE;

	tls_config->peer_verify = PEER_VERIFY;
	tls_config->cipher_count = 0;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_count = ARRAY_SIZE(sec_tag_list);
	tls_config->sec_tag_list = sec_tag_list;
	tls_config->hostname = MQTT_BROKER_HOSTNAME;
#else /* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

		/* worker_queue to handle received data */
		mqtt_work_queue_init();
}

/**@brief Initialize the file descriptor structure used by poll.
 */
static int fds_init(struct mqtt_client * c)
{
	if (c->transport.type == MQTT_TRANSPORT_NON_SECURE)
	{
		fds.fd = c->transport.tcp.sock;
	}
	else
	{
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = c->transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	return 0;
}

void modem_thread(void)
{
	struct modem_param_info * modem_ptr = NULL;
	int ret, err;

	char buffer[CONFIG_MESSAGE_BUFFER_SIZE] = { 0 };

	k_msleep(100);
	printk("Starting LTE link initialization\n");

	/* Provision certificates before connecting to the LTE network */
	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
		ca_cert,
		sizeof(ca_cert)
	);

	if (err)
	{
		printk("Failed to install ca_cert\n");
	}

	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
		private_cert,
		sizeof(private_cert)
	);

	if (err)
	{
		printk("Failed to install client cert\n");
	}

	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
		private_key,
		sizeof(private_key)
	);

	if (err)
	{
		printk("Failed to install client key\n");
	}

	// init button and LED
	led_init();
	//+btn_init();
	lte_init();

	k_sem_take(&thread_sync_sem, K_FOREVER);

	//lte_lc_init_and_connect();
	server_resolve();
	printk("LTE link initialization done\n");

	client_init(&client);

	modem_data_init();

	while (1)
	{
		ret = modem_info_params_get(&modem_param);

		if (ret < 0)
		{
			printk("Unable to obtain modem parameters: %d\n", ret);
		}
		else
		{
			modem_ptr = &modem_param;

			if (IS_ENABLED(CONFIG_MODEM_INFO_ADD_NETWORK))
			{
				snprintf(
					buffer,
					CONFIG_MESSAGE_BUFFER_SIZE,
					"operator id: %s, area code: %s (%d), cell id: %s (%d), band: %d, ip: %s, apn: %s, mode: %s",
					modem_ptr->network.current_operator.value_string,
					modem_ptr->network.area_code.value_string,
					modem_ptr->network.area_code.value,
					modem_ptr->network.cellid_hex.value_string,
					(int)modem_ptr->network.cellid_dec,
					modem_ptr->network.current_band.value,
					modem_ptr->network.ip_address.value_string,
					modem_ptr->network.apn.value_string,
					(modem_ptr->network.nbiot_mode.value == 1 && modem_ptr->network.gps_mode.value == 1 ? "NB-IoT, GPS" :
						(modem_ptr->network.lte_mode.value == 1 && modem_ptr->network.gps_mode.value == 1 ? "LTE-M, GPS" :
							(modem_ptr->network.nbiot_mode.value == 0 && modem_ptr->network.lte_mode.value == 0 && modem_ptr->network.gps_mode.value == 1 ? "GPS" :
								"unknown"
								)
							)
						)
				);

				printk("%s\n", buffer);

				if (IS_ENABLED(CONFIG_MODEM_INFO_ADD_DATE_TIME))
				{
					snprintf(buffer, CONFIG_MESSAGE_BUFFER_SIZE, "network time: %s", modem_ptr->network.date_time.value_string);
					printk("%s\n", buffer);
				}
			}
		}

		err = mqtt_connect(&client);

		if (err != 0)
		{
			printk("ERROR: mqtt_connect %d\n", err);
			//return;
		}
		else
		{

			err = fds_init(&client);

			if (err != 0)
			{
				printk("ERROR: fds_init %d\n", err);
				//return;
			}
			else
			{

				while (1)
				{
					err = poll(&fds, 1, mqtt_keepalive_time_left(&client));

					if (err < 0)
					{
						printk("ERROR: poll %d\n", errno);
						break;
					}

					err = mqtt_live(&client);

					if ((err != 0) && (err != -EAGAIN))
					{
						printk("ERROR: mqtt_live %d\n", err);
						break;
					}

					if ((fds.revents & POLLIN) == POLLIN)
					{
						err = mqtt_input(&client);
						if (err != 0)
						{
							printk("ERROR: mqtt_input %d\n", err);
							break;
						}
					}

					if ((fds.revents & POLLERR) == POLLERR)
					{
						printk("POLLERR\n");
						break;
					}

					if ((fds.revents & POLLNVAL) == POLLNVAL)
					{
						printk("POLLNVAL\n");
						break;
					}

				}

				printk("Disconnecting MQTT client...\n");

				err = mqtt_disconnect(&client);

				if (err)
				{
					printk("Could not disconnect MQTT client. Error: %d\n", err);
				}
			}
		}

		k_sleep(K_SECONDS(30));
	}
}


K_THREAD_DEFINE(modem_thread_id, 4096, modem_thread, NULL, NULL, NULL, 7, 0, 0);
