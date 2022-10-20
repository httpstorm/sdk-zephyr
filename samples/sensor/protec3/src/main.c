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
#include <net/socket.h>
#include <nrf_socket.h>
#include <dfu/mcuboot.h>

#include <adp536x-mod.h>
#include <buzzer.h>

#include <data/json.h>

#include "gpio_t.h"
#include "fota_update.h"
#include "apn.h"
#include "aws.h"
#include "eeprom.h"
#include "uart.h"
#include "main.h"
#include "mqtt_handler.h"

#define MSG_SIZE 768
char msg[MSG_SIZE];

// must be at least sizeof(nmea_out)
#define MQTT_SIZE 256
char mqtt[MQTT_SIZE];


#if CONFIG_NETWORKING
int tcp_connect_disconnect(int connect);
ssize_t lte_send(const char * msg, size_t size);
#endif

#ifdef CONFIG_UPDATE_FILE
void fota_dl_init(void);
void fota_dl_trigger(void);

static const struct device * uart0_dev;
#endif

#if 1
int send_sms(const char * number, const char * msg, char * cmd_buf, size_t buf_len);
void bme280_init_api(const struct device * i2c_dev);
#endif

#if 1
#include "shtc3.h"
#endif

#if 1
#include "iis2dlpc.h"
#endif


LOG_MODULE_REGISTER(protec3, CONFIG_PROTEC3_LOG_LEVEL);


// EEPROM device
const struct device * eeprom_dev = NULL;

const struct gpio_struct button_node = PIN_INPUT(button);
const struct gpio_struct gps_en_node = PIN_OUTPUT(gps_enable);
const struct gpio_struct alarm_loud_node = PIN_OUTPUT(alarm_loud);
const struct gpio_struct rtc_node = PIN_INPUT(rtc_int);
const struct gpio_struct pir_node = PIN_INPUT(pir_int);
const struct gpio_struct pmic_node = PIN_INPUT(pmic_int);
const struct gpio_struct nrf52840_int_node = PIN_INPUT(nrf52840_int);

struct eeprom_apn_config apn_config;

static uint16_t beep_length[2] = { 100, 100 };
static bool loud_on = false;

const static struct device * pin_init(const struct gpio_struct * gpio)
{
	const struct device * gpio_dev;
	int ret;

	gpio_dev = device_get_binding(gpio->gpio_dev_name);

	if (gpio_dev == NULL)
	{
		printk("Error: didn't find %s device\n", gpio->gpio_dev_name);
		return NULL;
	}

	ret = gpio_pin_configure(gpio_dev, gpio->gpio_pin, gpio->gpio_flags);

	if (ret != 0)
	{
		printk("Error %d: failed to configure pin %d '%s'\n", ret, gpio->gpio_pin, gpio->gpio_pin_name);
		return NULL;
	}

	return gpio_dev;
}

const static struct device * pin_int_init(
	const struct gpio_struct * gpio,
	struct gpio_callback * cb_data,
	void (*cb_func)(const struct device *, struct gpio_callback *, uint32_t),
	gpio_flags_t flags
)
{
	const struct device * gpio_dev;
	int ret;

	gpio_dev = pin_init(gpio);

	if (gpio_dev == NULL)
	{
		return NULL;
	}

	ret = gpio_pin_interrupt_configure(
		gpio_dev,
		gpio->gpio_pin,
		flags ? flags : GPIO_INT_EDGE_TO_ACTIVE
	);

	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio->gpio_pin_name, gpio->gpio_pin);
		return NULL;
	}

	gpio_init_callback(cb_data, cb_func, BIT(gpio->gpio_pin));
	gpio_add_callback(gpio_dev, cb_data);

	printk("Set up button at %s pin %d\n", gpio->gpio_pin_name, gpio->gpio_pin);

	return gpio_dev;
}


volatile bool button_int_event = false;
volatile bool pir_int_event = false;
volatile bool nrf52840_int_level = false;

static struct gpio_callback button_cb_data;
static struct gpio_callback rtc_int_cb_data;
static struct gpio_callback pir_int_cb_data;
static struct gpio_callback pmic_int_cb_data;
static struct gpio_callback nrf52840_int_cb_data;


const struct device * iis2dlpc_dev;
const struct device * shtc3_dev;
const struct device * alarm_loud;

struct service_info
{
	struct k_work work;
	const struct device * device;
	const struct gpio_struct * gpio_node;
	char data[256];
}
beep_service, button_debounce_service, pir_debounce_service, pmic_service,
nrf52840_int_debounce_service, gnss_uart_service;


void button_int(void)
{
	button_int_event = true;
	LOG_INF("Button click");
}

void button_pressed(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	k_work_submit(&button_debounce_service.work);
}

void button_debounce_worker(struct k_work * item)
{
	k_msleep(50);
	struct service_info * the_service = CONTAINER_OF(item, struct service_info, work);

	if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin))
	{
		button_int();
	}
}

void rtc_int_occured(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	printk("RTC Interrupt\n");
}

void pir_int_occured(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	k_work_submit(&pir_debounce_service.work);
}

void pir_debounce_worker(struct k_work * item)
{
	k_msleep(50);
	struct service_info * the_service = CONTAINER_OF(item, struct service_info, work);

	if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin))
	{
		pir_int_event = true;
	}
}

void nrf52840_int_h(void)
{
	struct service_info * the_service = CONTAINER_OF(&nrf52840_int_debounce_service.work, struct service_info, work);
	nrf52840_int_level = gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin);
}

void nrf52840_int_occured(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	nrf52840_int_h();
	k_work_submit(&nrf52840_int_debounce_service.work);
}

void nrf52840_int_debounce_worker(struct k_work * item)
{
	printk("nrf52840_int level %x\n", nrf52840_int_level);
}


void pmic_int_occured(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	k_work_submit(&pmic_service.work);
}

void pmic_worker(struct k_work * item)
{
	uint8_t int1_f, int2_f;

	adp536x_reg_read(0x34, &int1_f);
	adp536x_reg_read(0x34, &int2_f);

	printk("PMIC Interrupt: 0x%02x 0x%02x\n", int1_f, int2_f);
}

static uint8_t nmea_message[128];
static uint8_t nmea_message_pos = 0;
static char nmea_tmp[256];
static char nmea_out[256];
static char * nmea_tmp_p = NULL;
static char * nmea_out_p = nmea_out;
static int nmea_out_lock = 0;
static volatile size_t uart0_ptr;
static volatile size_t uart0_ctr;
static volatile int found_newline = 0;

size_t nmea_tmp_set_start(const char * msg, size_t size)
{
	size_t i = 0;
	nmea_tmp_p = NULL;

	// skip until the first start marker
	while (i < size)
	{
		if (msg[i] == '$')
		{
			nmea_tmp_p = nmea_tmp;
			break;
		}
	}

	return i;
}

void gnss_uart_cb(const struct device * x, void * user_data)
{
	static uint8_t uart_buf[1024];

	static uint8_t counter = 0;

	uart_irq_update(x);
	int data_length = 0;

	if (uart_irq_rx_ready(x))
	{
		data_length = uart_fifo_read(x, uart_buf, sizeof(uart_buf));
		uart_buf[data_length] = 0;
	}

	size_t j = 0;

	if (!nmea_tmp_p)
	{
		j = nmea_tmp_set_start(uart_buf, data_length);
	}

	if (nmea_tmp_p)
	{
		const char * const e = nmea_tmp + sizeof(nmea_tmp);

		while (j < data_length)
		{
			size_t size = data_length - j;

			if (size > (e - nmea_tmp_p))
			{
				size = e - nmea_tmp_p;
			}

			if (size)
			{
				// copy data
				memcpy(nmea_tmp_p, &uart_buf[j], size);
				nmea_tmp_p += size;
				j += size;
			}

			if (j < data_length)
			{
				// nmea_tmp full, we need to drop a packet
				char * p = nmea_tmp + 1;

				for (; p < e; p++)
				{
					if (*p == '$')
					{
						// drop the oldest packet
						size = e - p;
						//-printk("[%p-%p] %p %p  %u\n", nmea_tmp, e, p, nmea_tmp_p, size);
						memmove(nmea_tmp, p, size);
						nmea_tmp_p -= size;

						break;
					}
				}

				if (p == e)
				{
					printk(
						"WARNING: nmea_tmp full, and we did not find new packets. "
						"NMEA packets should not be larger than 80 characters.\n"
					);

					j = nmea_tmp_set_start(uart_buf + j, data_length - j);

					if (!nmea_tmp_p)
					{
						// no start marker in the received data
						// we will continue scanning as new data is received
						break;
					}
				}
			}
		}

		if (nmea_tmp_p && !nmea_out_lock)
		{
			const char * const f = nmea_out + sizeof(nmea_out) - 1;
			const char * m = nmea_tmp;
			const char * p = m + 1;
			size_t size = 0;

			for (; p < nmea_tmp_p; p++)
			{
				if (*p == '$')
				{
					// select a packet to copy
					size = p - m;

					if (size > (f - nmea_out))
					{
						printk("WARNING: nmea_out is too small\n");
						continue;
					}

					while (size > (f - nmea_out_p))
					{
						// not enough space in nmea_out
						// drop the first packet
						char * a = nmea_out + 1;

						for (; a <= nmea_out_p; a++)
						{
							if (*a == '$')
							{
								size_t size_copy = nmea_out_p - a;
								size_t size_drop = a - nmea_out;

								if (size_copy)
								{
									memmove(nmea_out, a, size_copy);
								}

								nmea_out_p -= size_drop;

								break;
							}
						}
					}

					// copy a packet
					memcpy(nmea_out_p, m, size);
					nmea_out_p += size;
					*nmea_out_p = '\0';
					m = p;
				}
			}

			size = nmea_tmp_p - m;
			size_t size_drop = m - nmea_tmp;

			if (size && size_drop)
			{
				// consolidate nmea_tmp
				memmove(nmea_tmp, m, size);
				nmea_tmp_p -= size_drop;
			}
		}
	}

	for (int i = 0; i < data_length; i++)
	{
		if (uart_buf[i] == '$' || uart_buf[i] == '!')
		{
			nmea_message_pos = 0;
		}

		if (nmea_message_pos < 127 || uart_buf[i] != '\r')
		{
			nmea_message[nmea_message_pos++] = uart_buf[i];
		}

		if (uart_buf[i] == '\n')
		{
			nmea_message[nmea_message_pos - 1] = 0;

			if (nmea_message[0] == '$' && strncmp(nmea_message + 3, "GGA", 3) == 0)
			{
				if (counter++ >= 10)
				{
					sprintf(gnss_uart_service.data, "%s", nmea_message);
					k_work_submit(&gnss_uart_service.work);
					//client_send(nmea_message);
					counter = 0;
				}
			}
		}
	}
}


void gnss_uart_worker(struct k_work * item)
{
	struct service_info * the_service = CONTAINER_OF(item, struct service_info, work);
	printk("%s\n", the_service->data);
}

static uint8_t sht3xd_compute_crc(uint16_t value)
{
	uint8_t buf[2] = { value >> 8, value & 0xFF };
	uint8_t crc = 0xFF;
	uint8_t polynom = 0x31;
	int i, j;

	for (i = 0; i < 2; ++i)
	{
		crc = crc ^ buf[i];

		for (j = 0; j < 8; ++j)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ polynom;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}

	return crc;
}


int sht3xd_write_command(struct device * dev, uint16_t cmd)
{
	uint8_t tx_buf[2] = { cmd >> 8, cmd & 0xFF };

	return i2c_write(shtc3_dev, tx_buf, sizeof(tx_buf), SHTC3_I2C_ADDR);
}

int shtc3_init(const char * dev_name)
{
	int err = 0;

	shtc3_dev = device_get_binding(dev_name);

	if (err)
	{
		err = -ENODEV;
	}

	return err;
}

struct rtc_date_time
{
	uint16_t y;
	uint8_t m;
	uint8_t d;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
};

int rtc_decode(const uint8_t * msg, struct rtc_date_time * p_date_time)
{
	uint16_t y[] = { msg[6] & 0x0f, ((msg[6] & 0xf0) >> 4) * 10, ((uint16_t)((msg[5] & 0xc0) >> 6)) * 100, 2000 };
	uint8_t m[] = { msg[5] & 0x0f, ((msg[5] & 0x10) >> 4) * 10 };
	uint8_t d[] = { msg[4] & 0x0f, ((msg[4] & 0x30) >> 4) * 10 };
	uint8_t hh[] = { msg[2] & 0x0f, ((msg[2] & 0x30) >> 4) * 10 };
	uint8_t mm[] = { msg[1] & 0x0f, ((msg[1] & 0x70) >> 4) * 10 };
	uint8_t ss[] = { msg[0] & 0x0f, ((msg[0] & 0x70) >> 4) * 10 };

#if 0
	if ((y[0] > 9) || (y[1] > 90))
	{
		printk("y %u %u %u %u\n", y[0], y[1], y[2], y[3]);
		return FAIL;
	}

	// invalid value from RTC
	if ((m[0] > 9) || (m[1] > 10) || !m[0])
	{
		printk("m %u %u\n", m[0], m[1]);
		return FAIL;
	}

	if ((d[0] > 9) || (d[1] > 30) || !d[0])
	{
		printk("d %u %u\n", d[0], d[1]);
		return FAIL;
	}

	if ((hh[0] > 9) || (hh[1] > 20))
	{
		printk("hh %u %u\n", hh[0], hh[1]);
		return FAIL;
	}

	if ((mm[0] > 9) || (mm[1] > 50))
	{
		printk("mm %u %u\n", mm[0], mm[1]);
		return FAIL;
	}

	if ((ss[0] > 9) || (ss[1] > 50))
	{
		printk("ss %u %u\n", ss[0], ss[1]);
		return FAIL;
	}
#endif

	struct rtc_date_time n =
	{
		.y = y[0] + y[1] + y[2] + y[3],
		.m = m[0] + m[1],
		.d = d[0] + d[1],
		.hh = hh[0] + hh[1],
		.mm = mm[0] + mm[1],
		.ss = ss[0] + ss[1],
	};

#if 0
	if (!n.m || !n.d || (n.m > 12) || (n.d > 31) || (n.hh > 23) || (n.mm > 59) || (n.ss > 59))
	{
		return FAIL;
	}
#endif

	* p_date_time = n;

	return OK;
}

void rtc_print(const uint8_t * msg)
{
	struct rtc_date_time date_time;

	if (OK != rtc_decode(msg, &date_time))
	{
		printk("cannot decode RTC date and time\n");
		return;
	}

	printk(
		"%04u-%02u-%02u  %02u:%02u:%02u\n",
		date_time.y,
		date_time.m,
		date_time.d,
		date_time.hh,
		date_time.mm,
		date_time.ss
	);
}


K_SEM_DEFINE(thread_sync_sem, 0, 2);

void beep_worker(struct k_work * item)
{
	//+ui_buzzer_set_frequency(2400, 100);
	ui_buzzer_set_frequency(2000, 100);
	k_msleep(loud_on ? beep_length[BEEP_TYPE_LOUD] : beep_length[BEEP_TYPE_NORMAL]);
	ui_buzzer_set_frequency(0, 0);
}

uint8_t client_id_buf[CLIENT_ID_LEN] = { 0 };


static int client_id_get(char * id_buf, size_t len)
{
	enum at_cmd_state at_state;
	char imei_buf[CGSN_RESP_LEN] = { 0 };
	int err = at_cmd_write("AT+CGSN", imei_buf, sizeof(imei_buf), &at_state);

	if (err)
	{
		printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
	}

	snprintf(id_buf, len, "%.*s", IMEI_LEN, imei_buf);

	return 0;
}

#if 0
/*
 * retrieves the ICCID of the inserted SIM card
 * modem must be in normal mode
 */
static int iccid_get(char * id_buf, size_t len)
{
	enum at_cmd_state at_state;
	char * final_doublequote;
	int i;
	char tmp;
	char iccid_buf[ICCID_BUF_LEN] = { 0 };
	int err = at_cmd_write("AT+CRSM=176,12258,0,0,10", iccid_buf, sizeof(iccid_buf), &at_state);
	//int err = at_cmd_write("AT%XICCID", iccid_buf, sizeof(iccid_buf), &at_state);

	if (err)
	{
		printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
		return -1;
	}

	//В Format of answer:
	// +CRSM: 144,0,"<ICCID, up to 22 digits>"

	// first get rid of final double quote
	final_doublequote = strstr(iccid_buf + 14, "\"");

	if (final_doublequote)
	{
		*final_doublequote = '\0';
	}

	strncpy(id_buf, iccid_buf + 14, len);

	// The retrieved ICCID is in little endian order, let's fix it
	len = strlen(id_buf);

	for (i = 0; i < len; i += 2)
	{
		tmp = id_buf[i + 1];
		id_buf[i + 1] = id_buf[i];
		id_buf[i] = tmp;
	}

	return 0;
}
#endif

enum FOTA_UPDATE_STATUS
{
	FOTA_UPDATE_STATUS_NORMAL = 0,
	FOTA_UPDATE_STATUS_NOTIFY = 1,
};

// notify the system after reboot that the firmware has been upgraded
void fota_update_notify(void)
{
	uint8_t update_status = FOTA_UPDATE_STATUS_NOTIFY;
	int err = eeprom_write(eeprom_dev, EEPROM_ADDRESS_UPDATE_STATUS, &update_status, sizeof(update_status));

	if (OK != err)
	{
		printk("fota_update_notify()  EEPROM write failed\n");
	}
}

// checks whether we have started for the first time after update
void fota_update_complete(void)
{
	uint8_t update_status = FOTA_UPDATE_STATUS_NORMAL;
	int err = eeprom_read(eeprom_dev, EEPROM_ADDRESS_UPDATE_STATUS, &update_status, sizeof(update_status));

	if (OK == err)
	{
		if (update_status == FOTA_UPDATE_STATUS_NOTIFY)
		{
			char msg[] = "update_status={\"status\": \"ready\"}\n";
			uart_fota_send(msg, sizeof(msg) - 1);
			//?boot_request_upgrade(BOOT_UPGRADE_PERMANENT);

			// clear the update status
			update_status = FOTA_UPDATE_STATUS_NORMAL;
			err = eeprom_write(eeprom_dev, EEPROM_ADDRESS_UPDATE_STATUS, &update_status, sizeof(update_status));

			if (OK != err)
			{
				printk("fota_update_complete()  EEPROM write failed\n");
			}
		}
	}
	else
	{
		printk("fota_update_complete()  EEPROM read failed\n");
	}
}

#define ALARM_ACTIVE_PERIODE 600

void main(void)
{
	size_t size = 0;
	int32_t alarm_periode = 0;
	uint8_t alarm_old = alarm_arm;
	uint8_t loud_old = loud_on;

	const struct device * gnss_uart_dev;
	const struct device * button;
	const struct device * gps_en;
	const struct device * rtc_dev;
	const struct device * rtc_int;
	const struct device * pir_int;
	const struct device * i2c2_dev;
	const struct device * pmic_int;
	const struct device * nrf52840_int;

	int adp_err, err;
	bool apn_valid = false;

	union startup_counter_union
	{
		uint32_t value;
		uint8_t buffer[4];
	};

#if 0
	// switch to normal mode to get ICCID
	lte_lc_normal();

	err = iccid_get(msg, sizeof(msg));

	// power of mode again, otherwise lte_lc_link will fail
	lte_lc_power_off();
#endif

	printk("*** Firmware build    " __DATE__ "  " __TIME__ "\n");

	union startup_counter_union startup_counter;
	eeprom_dev = device_get_binding(EEPROM0_DEVICE_NAME);

	if (eeprom_dev)
	{
		printk("EEPROM device %s initialized successfully\n", EEPROM0_DEVICE_NAME);
		err = eeprom_read(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));
		printk("Startup counter is: %d\n", startup_counter.value);
		startup_counter.value++;
		eeprom_write(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));

		// notify if this is the first boot after a firmware upgrade
		fota_update_complete();

		// read APN config from EEPROM
		apn_valid = apn_read(&apn_config, msg);
	}

	uart0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	uart0_ptr = 0;
	uart0_ctr = 0;
	my_uart_init(uart0_dev);

	fota_uart_init(uart_fota_recv, uart_fota_send);

	k_work_init(&beep_service.work, beep_worker);
	k_work_init(&button_debounce_service.work, button_debounce_worker);
	k_work_init(&pir_debounce_service.work, pir_debounce_worker);
	k_work_init(&pmic_service.work, pmic_worker);
	k_work_init(&gnss_uart_service.work, gnss_uart_worker);
	k_work_init(&nrf52840_int_debounce_service.work, nrf52840_int_debounce_worker);


	alarm_loud = pin_init(&alarm_loud_node);
	gps_en = pin_init(&gps_en_node);

	button = pin_int_init(&button_node, &button_cb_data, button_pressed, 0);
	button_debounce_service.device = button;
	button_debounce_service.gpio_node = &button_node;

	rtc_int = pin_int_init(&rtc_node, &rtc_int_cb_data, rtc_int_occured, 0);

	pir_int = pin_int_init(&pir_node, &pir_int_cb_data, pir_int_occured, 0);
	pir_debounce_service.device = pir_int;
	pir_debounce_service.gpio_node = &pir_node;

	pmic_int = pin_int_init(&pmic_node, &pmic_int_cb_data, pmic_int_occured, 0);
	pmic_service.device = pmic_int;
	pmic_service.gpio_node = &pmic_node;

	nrf52840_int = pin_int_init(&nrf52840_int_node, &nrf52840_int_cb_data, nrf52840_int_occured, GPIO_INT_EDGE_BOTH);
	nrf52840_int_debounce_service.device = nrf52840_int;
	nrf52840_int_debounce_service.gpio_node = &nrf52840_int_node;
	nrf52840_int_h();


	fota_dl_init();

	if (button_int_event || gpio_pin_get(button, button_node.gpio_pin))
	{
		button_int_event = false;

		// TODO: create a dedicated task to check for updates
		fota_dl_trigger();
	}

	adp_err = adp536x_init(I2C_BUS_2_DEV_NAME);

	if (adp_err)
	{
		printk("ADP536X failed to initialize, error: %d\n", adp_err);
	}
	else
	{
		printk("ADP536X initialized successfully\n");
	};

	gnss_uart_dev = device_get_binding(UART_DEVICE_NAME);

	if (gnss_uart_dev)
	{
		printk("Uart %s initialized successfully\n", UART_DEVICE_NAME);

		uart_irq_callback_set(gnss_uart_dev, gnss_uart_cb);
		uart_irq_rx_enable(gnss_uart_dev);

		gnss_uart_service.device = gnss_uart_dev;
	}
	else
	{
		printk("Error initialzing uart %s\n", UART_DEVICE_NAME);
	}

	/*uart0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	uart_irq_rx_enable(uart0_dev);*/

	err = shtc3_init(I2C_BUS_2_DEV_NAME);

	if (err)
	{
		printk("SHTC3 sensor failed to initialize, error: %d\n", err);
	}
	else
	{
		printk("SHTC3 sensor initialized successfully\n");

		uint8_t rx_buf[3];
		uint8_t tx_buf[2] = { 0xef, 0xc8 };

		if (i2c_write_read(shtc3_dev, SHTC3_I2C_ADDR, tx_buf, sizeof(tx_buf),
			rx_buf, sizeof(rx_buf)) < 0)
		{
			printk("SHTC3 Failed to read data sample!\n");
		}
		else
		{
			printk("SHTC3 device id: 0x%02x 0x%02x 0x%02x -> 0x%02x\n", rx_buf[0], rx_buf[1], rx_buf[2], sht3xd_compute_crc((rx_buf[0] << 8) + rx_buf[1]));
		}
	}

	rtc_dev = device_get_binding(RTC_I2C_DEV_NAME);

	if (rtc_dev)
	{
		printk("RTC on %s bus initialized successfully\n", RTC_I2C_DEV_NAME);

		uint8_t rx_buf[8];
		uint8_t tx_buf[1] = { 0 };

		if (i2c_write_read(rtc_dev, RTC_I2C_ADDR, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf)) < 0)
		{
			printk("RTC Failed to read data sample!\n");
		}
		else
		{
			for (int i = 0; i < 8; i++)
			{
				printk("RTC reg: 0x%02x -> 0x%02x\n", i, rx_buf[i]);
			}

			rtc_print(rx_buf);
		}


	}
	else
	{
		printk("Error initializing RTC on %s\n", RTC_I2C_DEV_NAME);

	}

	client_id_get(client_id_buf, sizeof(client_id_buf));

	printk("Client id: %s\n", client_id_buf);


	i2c2_dev = device_get_binding(I2C_BUS_2_DEV_NAME);

	if (i2c2_dev)
	{
		iis2dlpc_dev = i2c2_dev;
		iis2dlpc_init_api(iis2dlpc_dev);
	}

	//-bme280_init_api(i2c2_dev);
	shtc3_init_api(shtc3_dev);


	if (ui_buzzer_init() == 0)
	{
		printk("Buzzer initialized in silent mode\n");
	}
	else
	{
		printk("Buzzer initialization failed\n");
	}

	k_sem_give(&thread_sync_sem);
	k_sem_give(&thread_sync_sem);

	while (1)
	{
#if 1
#if CONFIG_NETWORKING
		size = 0;
#endif

		k_msleep(1);

		if (button_int_event || pir_int_event)
		{
			printk(
				"protec3_events={\"alarm\": %u, \"PIR\": %u}\n",
				button_int_event,
				pir_int_event
			);

#if CONFIG_NETWORKING
			size += snprintf(
				msg + size,
				sizeof(msg) - size,
				"protec3_events={\"alarm\": %u, \"PIR\": %u}\n",
				button_int_event,
				pir_int_event
			);
#endif
		}

		if (pir_int_event)
		{
			printk("PIR Sensor Interrupt\n");
			pir_int_event = false;
			alarm_periode = ALARM_ACTIVE_PERIODE;
		}

		if (button_int_event)
		{
			mqtt_pub_btn_press();

			if (alarm_arm)
			{
				if (loud_on)
				{
					alarm_arm = MQTT_REQUEST_OFF;
					loud_on = false;
				}
				else
				{
					loud_on = true;

#if 0
					/***************** SMS ***********************/
					printk("send_sms() ...\n");
					char sms[256];
					memset(sms, 0, sizeof(sms));
					err = send_sms("4917686281431", "Protec3", sms, sizeof(sms));
					printk("send_sms: %i\n\n", err);
					/***************** END SMS *******************/
#endif
				}
			}
			else
			{
				alarm_arm = MQTT_REQUEST_ON;
			}

			button_int_event = false;
		}

		if ((alarm_old != alarm_arm) || (loud_old != loud_on))
		{
			alarm_old = alarm_arm;
			loud_old = loud_on;
			gpio_pin_set(gps_en, gps_en_node.gpio_pin, alarm_arm);
			gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, loud_on);
			i2c_reg_write_byte(rtc_dev, RTC_I2C_ADDR, 0x07, alarm_arm ? 0x80 : 0x00);
			k_work_submit(&beep_service.work);
			tcp_connect_disconnect(alarm_arm);
		}

		if (alarm_arm && (loud_on || alarm_periode))
		{
			uint8_t rx_buf[8];
			uint8_t tx_buf[1] = { 0 };

			if (i2c_write_read(rtc_dev, RTC_I2C_ADDR, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf)) < 0)
			{
				printk("RTC Failed to read data sample!\n");
			}
			else
			{
				for (int i = 7; i < 8; i++)
				{
					printk("RTC reg: 0x%02x -> 0x%02x\n", i, rx_buf[i]);
				}

				rtc_print(rx_buf);
				struct rtc_date_time date_time;

				if (OK == rtc_decode(rx_buf, &date_time))
				{
					printk(
						"rtc={\"y\": %u, \"m\": %u, \"d\": %u, \"hh\": %u, \"mm\": %u, \"ss\": %u}\n",
						date_time.y,
						date_time.m,
						date_time.d,
						date_time.hh,
						date_time.mm,
						date_time.ss
					);

#if CONFIG_NETWORKING
					size += snprintf(
						msg + size,
						sizeof(msg) - size,
						"rtc={\"y\": %u, \"m\": %u, \"d\": %u, \"hh\": %u, \"mm\": %u, \"ss\": %u}\n",
						date_time.y,
						date_time.m,
						date_time.d,
						date_time.hh,
						date_time.mm,
						date_time.ss
					);
#endif
				}
			}

			if (!adp_err)
			{
				err = 0;

				uint8_t
					cs1 = 0,
					cs2 = 0,
					soc = 0,
					vbh = 0,
					vbl = 0,
					pgs = 0;

				if (adp536x_reg_read(0x08, &cs1) > 0) { err += 1; }
				if (adp536x_reg_read(0x09, &cs2) > 0) { err += 2; }
				if (adp536x_reg_read(0x21, &soc) > 0) { err += 4; }
				if (adp536x_reg_read(0x25, &vbh) > 0) { err += 8; }
				if (adp536x_reg_read(0x26, &vbl) > 0) { err += 16; }
				if (adp536x_reg_read(0x2f, &pgs) > 0) { err += 32; }

				if (!err)
				{
					uint16_t vb = (vbh << 5) | (vbl >> 3);

					printk(
						"adp536x={\"cs1\": \"0x%02x\", \"cs2\": \"0x%02x\", \"pgs\": \"0x%02x\", \"voltage\": %04d, \"level\": %d}\n",
						cs1,
						cs2,
						pgs,
						vb,
						soc
					);

#if CONFIG_NETWORKING
					size += snprintf(
						msg + size,
						sizeof(msg) - size,
						"adp536x={\"cs1\": \"0x%02x\", \"cs2\": \"0x%02x\", \"pgs\": \"0x%02x\", \"voltage\": %04d, \"level\": %d}\n",
						cs1,
						cs2,
						pgs,
						vb,
						soc
					);

#if MQTT_SIZE
					snprintf(mqtt, sizeof(mqtt), "%02x", pgs);
					mqtt_pub("/battery/status", mqtt);

					snprintf(mqtt, sizeof(mqtt), "%04d", vb);
					mqtt_pub("/battery/voltage", mqtt);
#endif
#endif
				}
			}

			struct shtc3_t shtc3;
			err = shtc3_read(shtc3_dev, &shtc3);

			if (OK == err)
			{
				printf("shtc3={\"temp\": %.3f, \"humidity\": %.3f}\n", shtc3.temperature, shtc3.humidity);

#if CONFIG_NETWORKING
				size += snprintf(
					msg + size,
					sizeof(msg) - size,
					"shtc3={\"temp\": %.3f, \"humidity\": %.3f}\n",
					shtc3.temperature,
					shtc3.humidity
				);

#if MQTT_SIZE
				snprintf(mqtt, sizeof(mqtt), "%.3f", shtc3.temperature);
				mqtt_pub("/temperature", mqtt);
#endif
#endif
			}

			// IIS2DLPC read XYZ
			struct iis2dlpc_t iis2dlpc;
			err = iis2dlpc_read(iis2dlpc_dev, &iis2dlpc);

			if (OK == err)
			{
				printk(
					"iis2dlpc={\"x\": %.3f, \"y\": %.3f, \"z\": %.3f}\n",
					((float)(iis2dlpc.x)) * 0.001,
					((float)(iis2dlpc.y)) * 0.001,
					((float)(iis2dlpc.z)) * 0.001
				);

#if CONFIG_NETWORKING
				size += snprintf(
					msg + size,
					sizeof(msg) - size,
					"iis2dlpc={\"x\": %.3f, \"y\": %.3f, \"z\": %.3f}\n",
					((float)(iis2dlpc.x)) * 0.001,
					((float)(iis2dlpc.y)) * 0.001,
					((float)(iis2dlpc.z)) * 0.001
				);

#if MQTT_SIZE
				snprintf(
					mqtt,
					sizeof(mqtt),
					"%.3f,%.3f,%.3f",
					((float)(iis2dlpc.x)) * 0.001,
					((float)(iis2dlpc.y)) * 0.001,
					((float)(iis2dlpc.z)) * 0.001
				);

				mqtt_pub("/accelerometer", mqtt);
#endif
#endif
			}



#if CONFIG_NETWORKING
			nmea_out_lock = true;

			if (nmea_out_p - nmea_out)
			{
				size += snprintf(
					msg + size,
					sizeof(msg) - size,
					"%s",
					nmea_out
				);

#if MQTT_SIZE
				snprintf(mqtt, sizeof(mqtt), "%s", nmea_out);
#endif

				nmea_out_p = nmea_out;
			}

			nmea_out_lock = false;

#if MQTT_SIZE
			mqtt_pub("/gnss/raw", mqtt);
#endif

			if (size)
			{
				if (size > (sizeof(msg) - 1))
				{

					LOG_ERR("msg out of range %u %u", size, sizeof(msg));
					size = sizeof(msg) - 1;
				}

				lte_send(msg, size);
			}
#endif
		}
#if CONFIG_NETWORKING
		else if (size)
		{
			if (size > (sizeof(msg) - 1))
			{
				LOG_ERR("msg out of range %u %u", size, sizeof(msg));
				size = sizeof(msg) - 1;
			}

			lte_send(msg, size);
		}
#endif

		int w = 1000;
		int32_t sleep_time = loud_on ? 1 : 60;

		while (
			!button_int_event &&
			(alarm_old == alarm_arm) &&
			!(pir_int_event && !alarm_periode)
			&& w--
			)
		{
			k_msleep(sleep_time);
		}

		if (alarm_periode > sleep_time)
		{
			alarm_periode -= sleep_time;
		}
		else
		{
			alarm_periode = 0;
		}

#else
		k_msleep(1);

		if (pir_int_event)
		{
			printk("PIR Sensor Interrupt\n");
			pir_int_event = false;
		}

		if (button_int_event)
		{
			if (alarm_arm)
			{
				if (loud_on)
				{
					loud_on = false;
					gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 0);

					alarm_arm = false;

					i2c_reg_write_byte(rtc_dev, RTC_I2C_ADDR, 0x07, 0x00);
				}
				else
				{
					loud_on = true;
					gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 1);

					gpio_pin_set(gps_en, gps_en_node.gpio_pin, 0);

				}

			}
			else
			{
				alarm_arm = true;
				gpio_pin_set(gps_en, gps_en_node.gpio_pin, 1);
				i2c_reg_write_byte(rtc_dev, RTC_I2C_ADDR, 0x07, 0x80);
			}

			k_work_submit(&beep_service.work);

			uint8_t rx_buf[8];
			uint8_t tx_buf[1] = { 0 };

			if (i2c_write_read(rtc_dev, RTC_I2C_ADDR, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf)) < 0)
			{
				printk("RTC Failed to read data sample!\n");
			}
			else
			{
				for (int i = 7; i < 8; i++)
				{
					printk("RTC reg: 0x%02x -> 0x%02x\n", i, rx_buf[i]);
				}

				rtc_print(rx_buf);
			}

			if (!adp_err)
			{
				err = 0;

				uint8_t
					cs1 = 0,
					cs2 = 0,
					soc = 0,
					vbh = 0,
					vbl = 0,
					pgs = 0;

				if (adp536x_reg_read(0x08, &cs1) > 0) { err += 1; }
				if (adp536x_reg_read(0x09, &cs2) > 0) { err += 2; }
				if (adp536x_reg_read(0x21, &soc) > 0) { err += 4; }
				if (adp536x_reg_read(0x25, &vbh) > 0) { err += 8; }
				if (adp536x_reg_read(0x26, &vbl) > 0) { err += 16; }
				if (adp536x_reg_read(0x2f, &pgs) > 0) { err += 32; }

				if (!err)
				{
					uint16_t vb = (vbh << 5) | (vbl >> 3);
					printk("adp536x status: 0x%02x, 0x%02x, 0x%02x, %04d mV, %d%%\n", cs1, cs2, pgs, vb, soc);
				}
			}

			button_int_event = false;
		}
#endif
	}
}

static void gray_thread()
{
	const struct gpio_struct leds[3] =
	{
		  PIN_OUTPUT(led0),
		  PIN_OUTPUT(led1),
		  PIN_OUTPUT(led2),
	};

	const struct device * led_dev[3];

	k_sem_take(&thread_sync_sem, K_FOREVER);

	for (int i = 0; i < 3; i++)
	{
		led_dev[i] = pin_init(&leds[i]);
	}

	static uint8_t gray_code[8] = { 0, 1, 3, 2, 6, 7, 5, 4 };
	int position = 0;
	printk("LED drivers initialized successfully\n");

	while (1)
	{
		if (alarm_arm)
		{
			for (int i = 0; i < 3; i++)
			{
				gpio_pin_set(led_dev[i], leds[i].gpio_pin, ((gray_code[position] & (1 << i)) ? 1 : 0));

			}

			position++;

			if (position > 7)
			{
				position = 0;
			}

			k_msleep(1000);
		}
		else
		{
			if (position != 0)
			{
				for (int i = 0; i < 3; i++)
				{
					gpio_pin_set(led_dev[i], leds[i].gpio_pin, 0);
				}

				position = 0;
			}

			k_msleep(100);
		}
	};

}

int set_beep_time(enum BEEP_TYPE type, uint16_t length)
{
	if ((BEEP_TIME_MAX < length) || (BEEP_TYPE_INVALID <= type))
	{
		return FAIL;
	}

	beep_length[type] = length;

	return OK;
}

uint16_t get_beep_time(enum BEEP_TYPE type)
{
	if (BEEP_TYPE_INVALID > type)
	{
		return beep_length[type];
	}
	else
	{
		return 0;
	}
}

K_THREAD_DEFINE(gray_thread_id, 1024, gray_thread, NULL, NULL, NULL, 7, 0, 0);
