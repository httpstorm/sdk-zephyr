/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>
#include <drivers/eeprom.h>

#include <string.h>
#include <errno.h>

#include <net/mqtt.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <modem/lte_lc.h>

//#include <modem/nrf_modem_lib.h>
#include <net/tls_credentials.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/modem_key_mgmt.h>

#include <random/rand32.h>
#include <secure_services.h>

#include <power/reboot.h>

#include "buzzer.h"
#include "eeprom.h"
#include "apn.h"
#include "uart.h"
#include "fota_update.h"
#include "drivers/shtc3.h"
#include "drivers/iis2dlpc_reg.h"

#if defined(CONFIG_LWM2M_CARRIER)
#include <lwm2m_carrier.h>
#endif

#ifndef HW_VERSION
  #define HW_VERSION "1.1"
#endif

#ifndef VERSION
  #ifndef CONFIG_MAJROR
    #define CONFIG_MAJOR 3
  #endif
  #ifndef CONFIG_MINOR
    #define CONFIG_MINOR 7
  #endif
  #if (CONFIG_MAJOR <= 9999) && (CONFIG_MINOR <= 9999)
    #define VERSION version()
    char sversion[]="9999.9999.20150710.045535";

char *version(void)
{
  const char data[] = __DATE__;
  const char tempo[] = __TIME__;
  const char nomes[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char omes[4];
  int ano, mes, dia, hora, min, seg;

  if (strcmp(sversion, "9999.9999.20150710.045535"))
    return sversion;

  if (strlen(data) != 11 || strlen(tempo) != 8)
    return NULL;

  sscanf(data, "%s %d %d", omes, &dia, &ano);
  sscanf(tempo, "%d:%d:%d", &hora, &min, &seg);
  mes= (strstr(nomes, omes) - nomes) / 3 + 1;
  snprintk(sversion, sizeof(sversion), "%d.%d.%04d%02d%02d.%02d%02d%02d", CONFIG_MAJOR, CONFIG_MINOR, ano, mes, dia, hora, min, seg);

  return sversion;
}
  #endif
#endif

uint32_t device_id[2] = { 0 };

#define I2C_BUS_2_DEV_NAME      DT_LABEL(DT_NODELABEL(i2c2))
//#define ADP536X_I2C_DEV_NAME	DT_LABEL(DT_NODELABEL(i2c2))
//#define SHTC3_I2C_DEV_NAME	DT_LABEL(DT_NODELABEL(i2c2))
#define RTC_I2C_DEV_NAME        DT_LABEL(DT_NODELABEL(i2c3))
#define EEPROM0_DEVICE_NAME     DT_LABEL(DT_NODELABEL(eeprom0))
#define UART_DEVICE_NAME        DT_LABEL(DT_NODELABEL(uart1))

#define SHTC3_I2C_ADDR          0x70
#define RTC_I2C_ADDR            0x68
#define EEPROM_I2C_BASE_ADDR    0x50
#define ADP536X_I2C_ADDR	0x46
#define ACCELEROMETER_I2C_ADDR  0x18

#define BEEP_FREQUENCY 2500

#define ICCID_BUF_LEN 40

#define BL_MAC_LIST_COUNT 6

/* Buffers for MQTT client. */
static u8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static u8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE + 1];

/* The mqtt client struct */
static struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* Connected flag */
static bool connected;

/* File descriptor */
static struct pollfd fds;

#if defined(CONFIG_MQTT_LIB_TLS)
static sec_tag_t sec_tag_list[] = { CONFIG_SEC_TAG };
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

struct gpio_struct {
	const char *gpio_dev_name;
	const char *gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

#define PIN_STRUCT(alias, type) {                                       \
        .gpio_dev_name = DT_GPIO_LABEL(DT_ALIAS(alias), gpios),         \
        .gpio_pin_name = DT_LABEL(DT_ALIAS(alias)),                     \
        .gpio_pin = DT_GPIO_PIN(DT_ALIAS(alias), gpios),                \
        .gpio_flags = type | FLAGS_OR_ZERO(DT_ALIAS(alias)),            \
}

#define PIN_OUTPUT(alias) PIN_STRUCT(alias, GPIO_OUTPUT)
#define PIN_INPUT(alias) PIN_STRUCT(alias, GPIO_INPUT)


static const char ca_cert[] = {
	#include "../certs/ca.crt"
};

static const char private_cert[] = {
	#include "../certs/client.crt"
};

static const char private_key[] = {
	#include "../certs/client.key"
};


//const struct gpio_struct bt_int_node = PIN_INPUT(bt_module_int);
//struct device *bt_int;

const struct device *button0;
const struct device *gnss_uart_dev;
const struct device *gnss_en;
const struct device *alarm_loud;
const struct device *shtc3_dev;
const struct device *accel_int1;

static stmdev_ctx_t dev_ctx;


const struct device * eeprom_dev = NULL;
struct eeprom_apn_config apn_config;
const static struct device * uart0_dev;

#define MSG_SIZE 768
char msg[MSG_SIZE];

const struct gpio_struct button0_node = PIN_INPUT(button);
const struct gpio_struct gnss_en_node = PIN_OUTPUT(gps_enable);

const struct gpio_struct alarm_loud_node = PIN_OUTPUT(alarm_loud);
const struct gpio_struct rtc_node = PIN_INPUT(rtc_int);
const struct gpio_struct pir_node = PIN_INPUT(pir_int);
const struct gpio_struct pmic_node = PIN_INPUT(pmic_int);
const struct gpio_struct accel_int1_node = PIN_INPUT(accel_int1);


const struct gpio_struct led_nodes[3] = {
      PIN_OUTPUT(led0),
      PIN_OUTPUT(led1),
      PIN_OUTPUT(led2),
};

const struct device *leds[3];

volatile bool leds_status[3] = { false };

#define IMEI_LEN 15

char imei[IMEI_LEN + 1] = { 0 };


volatile bool button_int_event = false;
volatile bool pir_int_event = false;

volatile bool alarm_status = false;
volatile bool armed_status = false;
volatile bool accel_wakeup_event = false;

static struct gpio_callback button_cb_data;
//static struct gpio_callback rtc_int_cb_data;
static struct gpio_callback pir_int_cb_data;
static struct gpio_callback pmic_int_cb_data;
static struct gpio_callback accel_int1_cb_data;

struct service_info {
    struct k_work work;
    const struct device *device;
    const struct gpio_struct *gpio_node;
    char data[256];
} beep_service, button_debounce_service, pir_debounce_service, pmic_service, gnss_uart_service, mqtt_message_handler_service, accel_int1_service;

struct k_work_delayable shtc3_delayed_work, accelerometer_delayed_work;

static const struct device *i2c2_dev;

static int adp536x_reg_read(u8_t reg, u8_t *buff)
{
	return i2c_reg_read_byte(i2c2_dev, ADP536X_I2C_ADDR, reg, buff);
}

static int adp536x_reg_write(u8_t reg, u8_t val)
{
	return i2c_reg_write_byte(i2c2_dev, ADP536X_I2C_ADDR, reg, val);
}

/*
static int adp536x_reg_write_mask(u8_t reg_addr,
			       u32_t mask,
			       u8_t data)
{
	int err;
	u8_t tmp;

	err = adp536x_reg_read(reg_addr, &tmp);
	if (err) {
		return err;
	}

	tmp &= ~mask;
	tmp |= data;

	return adp536x_reg_write(reg_addr, tmp);
}
*/


//
//
// frequency in Hz - 2000 or 3000 is the best depends on the buzzer
// duration in ms - max 5000
// loud should be true for over 100 db, best suited for alarm
//
//

void beep(u16_t frequency, u16_t duration, bool loud) {
    if (duration > 5000) {
        duration = 5000;
    }

    if (duration <= 0) {
        return;
    }

    beep_service.data[0] = (frequency >> 8) & 0xff;
    beep_service.data[1] = (frequency) & 0xff;

    beep_service.data[2] = (duration >> 8) & 0xff;
    beep_service.data[3] = (duration) & 0xff;

    beep_service.data[4] = (loud) & 0xff;

    beep_service.data[5] = 0;

    k_work_submit(&beep_service.work);
}

void beep_arm(bool status) {
    beep_service.data[5] = 1;
    beep_service.data[6] = status;
    k_work_submit(&beep_service.work);
}

void beep_worker(struct k_work *item)
{
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);

    u16_t frequency = (the_service->data[0] << 8) | the_service->data[1];
    u16_t duration = (the_service->data[2] << 8) | the_service->data[3];
    bool loud = the_service->data[4];

    bool arming = the_service->data[5];

    if (arming) {
        bool status = the_service->data[6];
        gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 0);
        if (status) {
            ui_buzzer_set_frequency(BEEP_FREQUENCY, 100);
            k_msleep(100);
            ui_buzzer_set_frequency(0, 0);
            k_msleep(100);
            ui_buzzer_set_frequency(BEEP_FREQUENCY, 100);
            k_msleep(100);
            ui_buzzer_set_frequency(0, 0);
        }
        else {
            ui_buzzer_set_frequency(BEEP_FREQUENCY, 100);
            k_msleep(500);
            ui_buzzer_set_frequency(0, 0);
        }
    }

    else {
        if (loud) {
            gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 1);
        }
        else {
            gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 0);
        }

        ui_buzzer_set_frequency(frequency, 100);
        k_msleep(duration);
        ui_buzzer_set_frequency(0, 0);

        gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 0);
    }
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, u32_t pins) {
        k_work_submit(&button_debounce_service.work);
}

static const struct device *pin_init(const struct gpio_struct *gpio)
{
	const struct device *gpio_dev;
	int ret;

	gpio_dev = device_get_binding(gpio->gpio_dev_name);
	if (gpio_dev == NULL) {
		printk("Error: couldn't find the device %s\n", gpio->gpio_dev_name);
		return NULL;
	}

	ret = gpio_pin_configure(gpio_dev, gpio->gpio_pin, gpio->gpio_flags);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n", ret, gpio->gpio_pin, gpio->gpio_pin_name);
		return NULL;
	}

        return gpio_dev;
}

static const struct device *pin_int_init(const struct gpio_struct *gpio, struct gpio_callback *cb_data, void (*cb_func)(const struct device *, struct gpio_callback *,u32_t)) {
	const struct device *gpio_dev;
	int ret;

        gpio_dev = pin_init(gpio);
        if (gpio_dev == NULL) {
                return NULL;
        }

        ret = gpio_pin_interrupt_configure(gpio_dev, gpio->gpio_pin, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
                printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio->gpio_pin_name, gpio->gpio_pin);
                return NULL;
        }

        gpio_init_callback(cb_data, cb_func, BIT(gpio->gpio_pin));
        gpio_add_callback(gpio_dev, cb_data);

        printk("Setting up a button at %s pin %d\n", gpio->gpio_pin_name, gpio->gpio_pin);

        return gpio_dev;
}

static int remove_whitespace(char *buf)
{
    size_t i, j = 0, len;

    len = strlen(buf);
    for (i = 0; i < len; i++) {
        if (buf[i] >= 32 && buf[i] <= 126) {
            if (j != i) {
                buf[j] = buf[i];
            }

            j++;
        }
    }

    if (j < len) {
        buf[j] = '\0';
    }

    return 0;
}

static int query_modem(const char *cmd, char *buf, size_t buf_len)
{
    int ret;
    enum at_cmd_state at_state;

    ret = at_cmd_write(cmd, buf, buf_len, &at_state);
    if (ret) {
        snprintk(buf, buf_len, "at_state: %d", at_state);
        //strncpy(buf, "error", buf_len);
        return ret;
    }

    remove_whitespace(buf);
    return 0;
}

static int client_id_get()
{
        int ret;
	//enum at_cmd_state at_state;
        char buf[32] = { 0 };


        ret = query_modem("AT+CGSN", buf, sizeof(buf));
        if (ret)
        {
              //printk("Error when trying to do at_cmd_write: %d, at_state: %d",
              //    err, at_state);
              //printk("ERROR: Failed to read IMEI.\n");
              //goto finish;
        }
        else
        {
              //printk("Modem IMEI read.\n");
              strncpy(imei, buf, IMEI_LEN);
        }

	return 0;
}

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

static int read_ficr_word(uint32_t *result, const volatile uint32_t *addr)
{
	//printk("Read FICR (address 0x%08x):\n", (uint32_t)addr);
	int ret = spm_request_read(result, (uint32_t)addr, sizeof(uint32_t));

	if (ret != 0) {
		printk("Could not read FICR (err: %d)\n", ret);
	}
	return ret;
}


#if defined(CONFIG_BSD_LIBRARY)

/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
	printk("bsdlib recoverable error: %u\n", (unsigned int)err);
}

#endif /* defined(CONFIG_BSD_LIBRARY) */

#if defined(CONFIG_LWM2M_CARRIER)
K_SEM_DEFINE(carrier_registered, 0, 1);

void lwm2m_carrier_event_handler(const lwm2m_carrier_event_t *event)
{
	switch (event->type) {
	case LWM2M_CARRIER_EVENT_BSDLIB_INIT:
		printk("LWM2M_CARRIER_EVENT_BSDLIB_INIT\n");
		break;
	case LWM2M_CARRIER_EVENT_CONNECT:
		printk("LWM2M_CARRIER_EVENT_CONNECT\n");
		break;
	case LWM2M_CARRIER_EVENT_DISCONNECT:
		printk("LWM2M_CARRIER_EVENT_DISCONNECT\n");
		break;
	case LWM2M_CARRIER_EVENT_READY:
		printk("LWM2M_CARRIER_EVENT_READY\n");
		k_sem_give(&carrier_registered);
		break;
	case LWM2M_CARRIER_EVENT_FOTA_START:
		printk("LWM2M_CARRIER_EVENT_FOTA_START\n");
		break;
	case LWM2M_CARRIER_EVENT_REBOOT:
		printk("LWM2M_CARRIER_EVENT_REBOOT\n");
		break;
	}
}
#endif /* defined(CONFIG_LWM2M_CARRIER) */

/**@brief Function to print strings without null-termination
 */
/*
static void data_print(u8_t *prefix, u8_t *data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}
*/
/**@brief Function to publish data on the configured topic
 */
/*
static int data_publish(struct mqtt_client *c, enum mqtt_qos qos,
	u8_t *data, size_t len)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	param.message.payload.data = data;
	param.message.payload.len = len;
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	data_print("Publishing: " topic, data, len);
	printk("to topic: %s len: %u\n",
		CONFIG_MQTT_PUB_TOPIC,
		(unsigned int)strlen(CONFIG_MQTT_PUB_TOPIC));

	return mqtt_publish(c, &param);
}
*/

static int data_publish1(struct mqtt_client *c, enum mqtt_qos qos, u8_t *subtopic,
	u8_t *data)
{
        u8_t topic[64];
	struct mqtt_publish_param param;

        snprintk(topic, sizeof(topic), "/%s%s", imei, subtopic);

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = topic;
	param.message.topic.topic.size = strlen(topic);
	param.message.payload.data = data;
	param.message.payload.len = strlen(data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	//data_print("Publishing: ", data, strlen(data));
	//printk("to topic: %s len: %u\n",
	//	topic,
	//	(unsigned int)strlen(data));

	return mqtt_publish(c, &param);
}

static u8_t last_location_message[128] = { 0 };

volatile bool publish_gnss_data = false;

void gnss_uart_worker(struct k_work *item) {
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
    if (publish_gnss_data) {
        snprintk(last_location_message, sizeof(last_location_message), "%s", the_service->data);
        data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/gnss/raw", last_location_message);
        publish_gnss_data = false;
    }
}

void mqtt_message_handler(struct k_work *item) {
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);

    u8_t *topic, *payload;
    topic = the_service->data;
    payload = strchr(topic, ' ');
    *payload = '\0';
    payload++;
    if (strlen(topic) > IMEI_LEN + 1) {
        topic += IMEI_LEN + 1;
    }

    printk("New message received on topic %s\n", topic);

    if (strcmp(topic, "/led/2") == 0) {
        bool update_status = false;
        if (strcmp(payload, "ON") == 0) {
            leds_status[2] = true;
            gpio_pin_set(leds[2], led_nodes[2].gpio_pin, 1);
            update_status = true;
        }
        else if (strcmp(payload, "OFF") == 0) {
            leds_status[2] = false;
            gpio_pin_set(leds[2], led_nodes[2].gpio_pin, 0);
            update_status = true;
        }
        else if (strcmp(payload, "GET") == 0) {
            update_status = true;
        }
        if (update_status) {
            if (leds_status[2]) {
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/led/2", "ON");
            }
            else {
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/led/2", "OFF");
            }
        }
    }
    else if (strcmp(topic, "/armed") == 0) {
        bool update_status = false;
        if (strcmp(payload, "ON") == 0) {
            armed_status = true;
            update_status = true;
            accel_wakeup_event = false;
            beep_arm(true);
        }
        else if (strcmp(payload, "OFF") == 0) {
            armed_status = false;
            update_status = true;
            if (alarm_status) {
                alarm_status = false;
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/alarm", "OFF");
            }
            beep_arm(false);
        }
        else if (strcmp(payload, "GET") == 0) {
            update_status = true;
        }
        if (update_status) {
            if (armed_status) {
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/armed", "ON");
            }
            else {
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/armed", "OFF");
            }
        }
    }
    else if (strcmp(topic, "/beep/loud") == 0) {
        char *ptr;
        long duration = strtol(payload, &ptr, 10);
        beep(BEEP_FREQUENCY, duration, true);
    }
    else if (strcmp(topic, "/beep/normal") == 0) {
        char *ptr;
        long duration = strtol(payload, &ptr, 10);
        beep(BEEP_FREQUENCY, duration, false);
    }
    else if (strcmp(topic, "/version") == 0) {
        if (strcmp(payload, "GET") == 0) {
            char buffer[64];
            snprintk(buffer, sizeof(buffer), "%s,%s,%s,%s,%08X%08X", __DATE__, __TIME__, VERSION, HW_VERSION, device_id[0], device_id[1]);
            data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/version", buffer);
        }
    }
    else if (strcmp(topic, "/blmaclist") == 0) {
        if (strcmp(payload, "GET") == 0) {
            u8_t mac_list[BL_MAC_LIST_COUNT * 8];
            //int64_t start_time = k_uptime_get();
            int err = eeprom_read(eeprom_dev, EEPROM_ADDRESS_BL_MAC_LIST, mac_list, sizeof(mac_list));
            if (OK == err) {
                //int64_t delta_time = k_uptime_delta(&start_time);
                //printk("eeprom buffer read successfully in %lld seconds and %lld miliseconds \n", delta_time / 1000, delta_time % 1000);
                u8_t buffer[BL_MAC_LIST_COUNT * 8 * 2 + 1] = { 0 };
                for (u8_t i = 0; i < BL_MAC_LIST_COUNT * 8; i++) {
                    snprintk(&buffer[i * 2], 3, "%02X", mac_list[i]);
                }
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/blmaclist/list", buffer);
            }
            else {
                printk("Error reading BL MAC list: %d\n", err);
            }
        }
    }
    else if (strcmp(topic, "/blmaclist/update") == 0) {
        if (strlen(payload) == (BL_MAC_LIST_COUNT * 8 * 2)) {
            u8_t mac_list[BL_MAC_LIST_COUNT * 8];
            for (u8_t i = 0; i < BL_MAC_LIST_COUNT * 8; i++) {
                mac_list[i] = (payload[i * 2] % 32 + 9) % 25 * 16 + (payload[(i * 2) + 1] % 32 + 9) % 25;
            }
            printk("eerom write with return code: %d\n", eeprom_write(eeprom_dev, EEPROM_ADDRESS_BL_MAC_LIST, mac_list, sizeof(mac_list)));
        }
    }
    else if (strcmp(topic, "/fota") == 0) {
          if (strcmp(payload, "UPGRADE") == 0) {
             printk("Starting Firmware Over The Air Update...\n");
             fota_dl_trigger();
          }
    }
    else if (strcmp(topic, "/reboot") == 0) {
        if (strcmp(payload, "COLD") == 0) {
            printk("Performing cold system reboot...\n");
            sys_reboot(SYS_REBOOT_COLD);
        }
        else if (strcmp(payload, "WARM") == 0) {
            printk("Performing warm system reboot...\n");
            sys_reboot(SYS_REBOOT_WARM);
        }
    }
}


void publish_timer_handler(struct k_timer *dummy)
{
     publish_gnss_data = true;
     k_work_submit(&pmic_service.work);
	 k_work_schedule(&shtc3_delayed_work, K_SECONDS(3));
	 k_work_schedule(&accelerometer_delayed_work, K_SECONDS(5));
}

K_TIMER_DEFINE(publish_timer, publish_timer_handler, NULL);


/**@brief Function to subscribe to the configured topic
 */
static int subscribe(void)
{
    int err;
//#define SUBSCRIBE_TOPICS_COUNT 10

        u8_t topics_buf[][64] = {
          //"/led/0", //          LED control		ON OFF GET	replay /status/led/[0-2] ON OFF
          //"/led/1",
          "/led/2",
          "/alarm", //		alarm arm		ON OFF GET	replay /status/alarm ON OFF, store in EEPROM
          "/beep/loud", //	beep loud		time ms		beep for time in ms, loud
          "/beep/normal", //	beep normal		time ms		beep for time in ms, normal
          "/uptime", //		uptime			GET			replay /status/uptime in seconds
          "/status",
          "/armed",
          "/version",
          "/blmaclist/update",
          "/blmaclist",
          "/fota",
          "/reboot"
        };

        u8_t topic_count = sizeof(topics_buf) / sizeof(topics_buf[0]);

        printk("Subscribing to %d topics\n", topic_count);

        for (u8_t i = 0; i < topic_count; i++) {

              struct mqtt_topic subscribe_topic; //s[SUBSCRIBE_TOPICS_COUNT];

              //for (int i = 0; i < SUBSCRIBE_TOPICS_COUNT; i++) {
                  int len = strlen(topics_buf[i]);
                  memmove(&topics_buf[i][1 + IMEI_LEN], topics_buf[i], len + 1);
                  topics_buf[i][0] = '/';
                  strncpy(&topics_buf[i][1], imei, IMEI_LEN);
                  subscribe_topic/*s[i]*/.topic.utf8 = topics_buf[i];
                  subscribe_topic/*s[i]*/.topic.size = strlen(topics_buf[i]);
                  subscribe_topic/*s[i]*/.qos = MQTT_QOS_1_AT_LEAST_ONCE;
              //}

              const struct mqtt_subscription_list subscription_list = {
                      .list = &subscribe_topic,
                      .list_count = 1, //SUBSCRIBE_TOPICS_COUNT,
                      .message_id = 12340 + i
              };

              //printk("Subscribing to topic: %d\n", SUBSCRIBE_TOPICS_COUNT);

              printk("Subscribing to: %s topic\n", topics_buf[i]);
              err = mqtt_subscribe(&client, &subscription_list);

              if (err < 0) {
                  return 0;
              }
        }

        return 0;
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client *c, size_t length)
{
	u8_t *buf = payload_buf;
	u8_t *end = buf + length;

	if (length > sizeof(payload_buf)) {
		return -EMSGSIZE;
	}

	while (buf < end) {
		int ret = mqtt_read_publish_payload(c, buf, end - buf);

		if (ret < 0) {
			int err;

			if (ret != -EAGAIN) {
				return ret;
			}

			printk("mqtt_read_publish_payload: EAGAIN\n");

			err = poll(&fds, 1,
				   CONFIG_MQTT_KEEPALIVE * MSEC_PER_SEC);
			if (err > 0 && (fds.revents & POLLIN) == POLLIN) {
				continue;
			} else {
				return -EIO;
			}
		}

		if (ret == 0) {
			return -EIO;
		}

		buf += ret;
	}

	return 0;
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client *const c,
		      const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}

		connected = true;
		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
		err = subscribe();
                if (err == 0) {
                    printk("MQTT subscription finished without errors!\n");
                }
                else {
                    printk("Failed to subscribe to MQTT topics with error: %d\n", err);
                }

                char buffer[64];

                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/link", "ONLINE");
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/armed", "OFF");
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/alarm", "OFF");
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/led/2", "OFF");
                snprintk(buffer, sizeof(buffer), "%s,%s,%s,%s,%08X%08X", __DATE__, __TIME__, VERSION, HW_VERSION, device_id[0], device_id[1]);
                data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/version", buffer);

                k_timer_start(&publish_timer, K_SECONDS(3), K_SECONDS(60));

		break;

	case MQTT_EVT_DISCONNECT:
		printk("[%s:%d] MQTT client disconnected %d\n", __func__,
		       __LINE__, evt->result);

                k_timer_stop(&publish_timer);
		connected = false;
		break;

	case MQTT_EVT_PUBLISH: {
		const struct mqtt_publish_param *p = &evt->param.publish;

                u8_t topic[p->message.topic.topic.size + 1];
                snprintk(topic, sizeof(topic), "%s", p->message.topic.topic.utf8);

		//printk("[%s:%d] MQTT PUBLISH result=%d len=%d\n", __func__,
		//       __LINE__, evt->result, p->message.payload.len);
		err = publish_get_payload(c, p->message.payload.len);
		if (err >= 0) {
		//		data_print("Received: ", payload_buf, p->message.payload.len);
                //printk("to topic: %s len: %u\n", topic, p->message.payload.len);

                      u8_t payload[p->message.payload.len + 1];
                      memcpy(payload, payload_buf, p->message.payload.len);
                      payload[p->message.payload.len] = '\0';

                      sprintf(mqtt_message_handler_service.data, "%s %s", topic, payload);
                      k_work_submit(&mqtt_message_handler_service.work);

                /* Echo back received data */
                //payload_buf[p->message.payload.len] = '\0';
                //data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/echo",
                //	payload_buf/*, p->message.payload.len*/);
		} else {
			//printk("mqtt_read_publish_payload: Failed! %d\n", err);
			//printk("Disconnecting MQTT client...\n");

			err = mqtt_disconnect(c);
			if (err) {
				printk("Could not disconnect: %d\n", err);
			}
		}
	} break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			//printk("MQTT PUBACK error %d\n", evt->result);
			break;
		}

		//printk("[%s:%d] PUBACK packet id: %u\n", __func__, __LINE__,
		//		evt->param.puback.message_id);
		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0) {
			//printk("MQTT SUBACK error %d\n", evt->result);
			break;
		}

		//printk("[%s:%d] SUBACK packet id: %u\n", __func__, __LINE__,
		//		evt->param.suback.message_id);
		break;

	default:
		//printk("[%s:%d] default: %d\n", __func__, __LINE__,
		//		evt->type);
		break;
	}
}

/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static void broker_init(void)
{
	int err;
	struct addrinfo *result;
	struct addrinfo *addr;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};


	struct nrf_in_addr dns;
	dns.s_addr = 134744072; // Google DNS, 8.8.8.8
	err = nrf_setdnsaddr(NRF_AF_INET, &dns);
	printk("DNS set with return code: %d\n", err);

	err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);
	if (err) {
		printk("ERROR: getaddrinfo failed %d\n", err);

		return;
	}

	addr = result;
	err = -ENOENT;

	/* Look for address of the broker. */
	while (addr != NULL) {
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in)) {
			struct sockaddr_in *broker4 =
				((struct sockaddr_in *)&broker);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr =
				((struct sockaddr_in *)addr->ai_addr)
				->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr,
				  ipv4_addr, sizeof(ipv4_addr));
			printk("MQTT broker ip address found %s\n", ipv4_addr);

			break;
		} else {
			printk("ai_addrlen = %u should be %u or %u\n",
				(unsigned int)addr->ai_addrlen,
				(unsigned int)sizeof(struct sockaddr_in),
				(unsigned int)sizeof(struct sockaddr_in6));
		}

		addr = addr->ai_next;
		break;
	}

	/* Free the address. */
	freeaddrinfo(result);
}

/**@brief Initialize the MQTT client structure
 */
static void client_init(struct mqtt_client *client)
{
        static struct mqtt_utf8 password;
        static struct mqtt_utf8 user_name;
        static struct mqtt_utf8 will_message;
        static struct mqtt_topic will_topic;
        static struct mqtt_utf8 will_topic_topic;

        static u8_t client_id[64];
        static u8_t topic[64];

	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;

        snprintk(client_id, sizeof(client_id), "protec3-%s", imei);
	client->client_id.utf8 = (u8_t *)client_id;
	client->client_id.size = strlen(client_id);

        password.utf8 = CONFIG_MQTT_CLIENT_PASSWORD;
        password.size = strlen(password.utf8);
        client->password = &password;

        user_name.utf8 = (u8_t *)CONFIG_MQTT_CLIENT_USERNAME;
        user_name.size = strlen(user_name.utf8);
        client->user_name = &user_name;

        will_message.utf8 = (u8_t *)"OFFLINE";
        will_message.size = strlen(will_message.utf8);
        client->will_message = &will_message;

        snprintk(topic, sizeof(topic), "/%s/status/link", imei);
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
        printk("Configuring secure MQTT connection...\n");
        struct mqtt_sec_config *tls_config = &client->transport.tls.config;

        client->transport.type = MQTT_TRANSPORT_SECURE;

        tls_config->peer_verify = CONFIG_PEER_VERIFY;
        tls_config->cipher_count = 0;
        tls_config->cipher_list = NULL;
        tls_config->sec_tag_count = ARRAY_SIZE(sec_tag_list);
        tls_config->sec_tag_list = sec_tag_list;
        tls_config->hostname = CONFIG_MQTT_BROKER_HOSTNAME;
#else /* MQTT transport configuration */
        client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

}

/**@brief Initialize the file descriptor structure used by poll.
 */
static int fds_init(struct mqtt_client *c)
{
	if (c->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds.fd = c->transport.tcp.sock;
	} else {
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = c->transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	return 0;
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
void modem_configure(void)
{
#if defined(CONFIG_LTE_LINK_CONTROL)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT)) {
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	} else {
#if defined(CONFIG_LWM2M_CARRIER)
		/* Wait for the LWM2M_CARRIER to configure the modem and
		 * start the connection.
		 */
		printk("Waitng for carrier registration...\n");
		k_sem_take(&carrier_registered, K_FOREVER);
		printk("Registered!\n");
#else /* defined(CONFIG_LWM2M_CARRIER) */
		int err;

		printk("LTE Link Connecting ...\n");
		err = lte_lc_init_and_connect();
		__ASSERT(err == 0, "LTE link could not be established.");
		printk("LTE Link Connected!\n");
#endif /* defined(CONFIG_LWM2M_CARRIER) */
	}
#endif /* defined(CONFIG_LTE_LINK_CONTROL) */
}

/* Initialize AT communications */
int at_comms_init(void)
{
	int err;

	err = at_cmd_init();
	if (err) {
		printk("Failed to initialize AT commands, err %d\n", err);
		return err;
	}

	err = at_notif_init();
	if (err) {
		printk("Failed to initialize AT notifications, err %d\n", err);
		return err;
	}

	return 0;
}

/* Provision certificate to modem */
int cert_provision(void)
{
	int err;
	bool exists;
	uint8_t unused;

	err = modem_key_mgmt_exists(sec_tag_list[0],
				    MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
				    &exists, &unused);
	if (err) {
		printk("Failed to check for server certificates err %d\n", err);
		return err;
	}

	if (exists) {

		/* For the sake of simplicity we delete what is provisioned
		 * with our security tag and reprovision our certificate.
		 */
		err = modem_key_mgmt_delete(sec_tag_list[0],
					    MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN);
		if (err) {
			printk("Failed to delete existing certificate, err %d\n",
			       err);
		}
	}

	err = modem_key_mgmt_exists(sec_tag_list[0],
				    MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
				    &exists, &unused);
	if (err) {
		printk("Failed to check for server certificates err %d\n", err);
		return err;
	}

	if (exists) {

		/* For the sake of simplicity we delete what is provisioned
		 * with our security tag and reprovision our certificate.
		 */
		err = modem_key_mgmt_delete(sec_tag_list[0],
					    MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT);
		if (err) {
			printk("Failed to delete existing certificate, err %d\n",
			       err);
		}
	}


//        if (!exists) {
//              printk("Provisioning server certificate\n");
//
//              /*  Provision certificate to the modem */
//              err = modem_key_mgmt_write(sec_tag_list[0],
//                                         MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
//                                         server_cert, sizeof(server_cert) - 1);
//              if (err) {
//                      printk("Failed to provision server certificate, err %d\n", err);
//                      return err;
//              }
//        }
//        else {
//              printk("Server certificate already provisioned\n");
//        }



	err = modem_key_mgmt_exists(sec_tag_list[0],
				    MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
				    &exists, &unused);
	if (err) {
		printk("Failed to check for private certificates err %d\n", err);
		return err;
	}


//	if (exists) {
//
//		/* For the sake of simplicity we delete what is provisioned
//		 * with our security tag and reprovision our certificate.
//		 */
//		err = modem_key_mgmt_delete(sec_tag_list[0],
//					    MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT);
//		if (err) {
//			printk("Failed to delete existing certificate, err %d\n",
//			       err);
//		}
//	}

        if (!exists) {
              printk("Provisioning private certificate\n");

              /*  Provision certificate to the modem */
              err = modem_key_mgmt_write(sec_tag_list[0],
                                         MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
                                         ca_cert, sizeof(ca_cert) - 1);
              if (err) {
                      printk("Failed to provision private certificate, err %d\n", err);
                      return err;
              }
        }
        else {
              printk("Private certificate already provisioned\n");
        }


	return 0;
}


int cert_provision1(nrf_sec_tag_t sec_tag, enum modem_key_mgmt_cred_type cred_type, const void *buf, size_t len)
{
	int err;
	bool exists;
	uint8_t unused;

	err = modem_key_mgmt_exists(sec_tag,
				    cred_type,
				    &exists, &unused);
	if (err) {
		printk("Failed to check for certificates err %d\n", err);
		return err;
	}

//	if (exists) {
//                /* For the sake of simplicity we delete what is provisioned
//		 * with our security tag and reprovision our certificate.
//		 */
//		err = modem_key_mgmt_delete(sec_tag,
//					    cred_type);
//		if (err) {
//			printk("Failed to delete existing certificate, err %d\n",
//			       err);
//		}
//	}


        if (!exists) {
              printk("Provisioning certificate\n");

              /*  Provision certificate to the modem */
              err = modem_key_mgmt_write(sec_tag,
                                         cred_type,
                                         buf, len - 1);
              if (err) {
                      printk("Failed to provision certificate, err %d\n", err);
                      return err;
              }
        }
        else {
              printk("Certificate already provisioned\n");
        }


	return 0;
}

void button_debounce_worker(struct k_work *item) {
    k_msleep(50);
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
    if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin)) {
        //button_int_event = true;

        data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/button", "ON");
    }
}

void accel_int1_worker(struct k_work *item) {
    if (armed_status) {
        accel_wakeup_event = true;
    }
}

void accel_int1_occured(const struct device *dev, struct gpio_callback *cb, u32_t pins) {
    k_work_submit(&accel_int1_service.work);
}

void pir_int_occured(const struct device *dev, struct gpio_callback *cb, u32_t pins) {
      if (armed_status) {
        k_work_submit(&pir_debounce_service.work);
      }
}

volatile s64_t pir_events[3] = { 0 };

void pir_debounce_worker(struct k_work *item) {
      k_msleep(50);
      struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
      if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin)) {
            pir_events[0] = pir_events[1];
            pir_events[1] = pir_events[2];
            pir_events[2] = k_uptime_get();
            //printk("PIR event at: %lld ms\n", pir_events[2]);
      }
}



static u8_t nmea_message[128] = { 0 };
static u8_t nmea_message_pos = 0;

void gnss_uart_cb(const struct device *x, void *data)
{
        static u8_t uart_buf[1024];

        static u8_t counter = 0;

	uart_irq_update(x);
	int data_length = 0;

	if (uart_irq_rx_ready(x)) {
		data_length = uart_fifo_read(x, uart_buf, sizeof(uart_buf));
		uart_buf[data_length] = 0;
	}
        for (int i = 0; i < data_length; i++) {
            if (uart_buf[i] == '$' || uart_buf[i] == '!') {
                nmea_message_pos = 0;
            }

            if (nmea_message_pos < 127  || uart_buf[i] != '\r') {
                nmea_message[nmea_message_pos++] = uart_buf[i];
            }

            if (uart_buf[i] == '\n') {
                nmea_message[nmea_message_pos - 1] = 0;
                if (nmea_message[0] == '$' && strncmp(nmea_message + 3, "GGA", 3) == 0) {
                        if (counter++ >= 10) {
                            sprintf(gnss_uart_service.data, "%s", nmea_message);
                            k_work_submit(&gnss_uart_service.work);
                            //client_send(nmea_message);
                            counter = 0;
                        }
                }
            }
        }
}

void pmic_int_occured(const struct device *dev, struct gpio_callback *cb, u32_t pins) {
      k_work_submit(&pmic_service.work);
}

void pmic_worker(struct k_work *item)
{
      int err = 0;

      u8_t buffer[64];

      u8_t cs1 = 0, cs2 = 0, soc = 0, vbh = 0, vbl = 0, pgs = 0;
      u16_t vb = 0;

      if (adp536x_reg_read(0x08, &cs1) > 0) { err += 1; }   // CHARGER_STATUS1
      if (adp536x_reg_read(0x09, &cs2) > 0) { err += 2; }   // CHARGER_STATUS2
      if (adp536x_reg_read(0x21, &soc) > 0) { err += 4; }   // BAT_SOC
      if (adp536x_reg_read(0x25, &vbh) > 0) { err += 8; }   // VBAT_READ_H
      if (adp536x_reg_read(0x26, &vbl) > 0) { err += 16; }  // VBAT_READ_L
      if (adp536x_reg_read(0x2f, &pgs) > 0) { err += 32; }  // PGOOD_STATUS

      if (err == 0) {
          vb = (vbh << 5) | (vbl >> 3);

          snprintk(buffer, sizeof(buffer), "%02x%02x%02x%02x", cs1, cs2, soc, pgs);
          data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/battery/status", buffer);

          snprintk(buffer, sizeof(buffer), "%04d", vb);
          data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/battery/voltage", buffer);
      }
      else {
          printk("Error while getting data from PMIC: %d\n", err);
      }

}


void shtc3_worker(struct k_work *item) {
      int err = 0;
      u8_t buffer[64];

      float temp, humi;

      err = SHTC3_GetTempAndHumi(&temp, &humi);
      if (err == 0) {
          snprintk(buffer, sizeof(buffer), "%0.2f", temp);
          data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/temperature", buffer);
          snprintk(buffer, sizeof(buffer), "%0.0f", humi);
          data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/humidity", buffer);
      }
      else {
          printk("Error while getting temperature and humidity: %d\n", err);
      }
}

void accelerometer_delayed_worker(struct k_work *item) {
      //uint8_t reg;
      //int32_t ret;
      int16_t data_raw_acceleration[3];
      float acceleration_mg[3];

      u8_t buffer[64];

      iis2dlpc_all_sources_t all_source;
      /* Read status register */

      u8_t counter = 20;
      while (counter > 0) {
          iis2dlpc_all_sources_get(&dev_ctx, &all_source);
          if (all_source.wake_up_src.sleep_state_ia) {
              break;
          }
          k_sleep(K_SECONDS(1));
          counter--;
      }

      //ret = iis2dlpc_read_reg(&dev_ctx, IIS2DLPC_STATUS, (uint8_t *) &reg, 1);
      //printk("reg data: 0x%02x\n", reg);

      /* Read acceleration data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      iis2dlpc_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = iis2dlpc_from_fs2_lp1_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = iis2dlpc_from_fs2_lp1_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = iis2dlpc_from_fs2_lp1_to_mg(
                             data_raw_acceleration[2]);
      snprintk(buffer, sizeof(buffer), "%4.2f,%4.2f,%4.2f,%d",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], all_source.wake_up_src.sleep_state_ia ? 1 : 0);

      data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/accelerometer", buffer);

}


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

void sensors_init();

union startup_counter_union
{
        uint32_t value;
        uint8_t buffer[4];
};

void main(void)
{
	int err;
        bool apn_valid = false;

        k_work_init(&button_debounce_service.work, button_debounce_worker);
        k_work_init(&beep_service.work, beep_worker);
        k_work_init(&gnss_uart_service.work, gnss_uart_worker);
        k_work_init(&pmic_service.work, pmic_worker);
        k_work_init(&pir_debounce_service.work, pir_debounce_worker);
        k_work_init(&mqtt_message_handler_service.work, mqtt_message_handler);
        k_work_init(&accel_int1_service.work, accel_int1_worker);
		k_work_init_delayable(&shtc3_delayed_work, shtc3_worker);
		k_work_init_delayable(&accelerometer_delayed_work, accelerometer_delayed_worker);

        button0 = pin_init(&button0_node);

        for (int i = 0; i < 3; i++) {
              leds[i] = pin_init(&led_nodes[i]);
        }

        //printk("my_uart_init function starting...\n");
        uart0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
	my_uart_init(uart0_dev);

        fota_uart_init(uart_fota_recv, uart_fota_send);
        fota_dl_init();

	// switch to normal mode to get ICCID
	lte_lc_normal();

	err = iccid_get(msg, sizeof(msg));

	// power of mode again, otherwise lte_lc_link will fail
	lte_lc_power_off();

	union startup_counter_union startup_counter;
	eeprom_dev = device_get_binding(EEPROM0_DEVICE_NAME);

	if (eeprom_dev)
	{
		printk("EEPROM device %s with size %d bytes initialized successfully\n", EEPROM0_DEVICE_NAME, eeprom_get_size(eeprom_dev));
                err = eeprom_read(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));
		printk("Startup counter is: %u\n", startup_counter.value);
		startup_counter.value++;
		eeprom_write(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));

		// notify if this is the first boot after a firmware upgrade
		fota_update_complete();

		// read APN config from EEPROM
		apn_valid = apn_read(&apn_config, msg);
                printk("apn_valid is %s\n", apn_valid ? "true" : "false");
	}

        printk("Version1\n");

        printk("Waiting for user input...\n");

        while (gpio_pin_get(button0, button0_node.gpio_pin) == 0) {
            gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 1);
            k_sleep(K_MSEC(500));
            gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 0);
            k_sleep(K_MSEC(500));
        }

        printk("*** Firmware build    " __DATE__ "  " __TIME__ "\n");

        if ((read_ficr_word(&device_id[0], &NRF_FICR_S->INFO.DEVICEID[0]) == 0) && (read_ficr_word(&device_id[1], &NRF_FICR_S->INFO.DEVICEID[1]) == 0)) {
            printk("*** Unique device Id: %08X%08X\n", device_id[0], device_id[1]);
        }
        else {
            device_id[0] = 0;
            device_id[1] = 0;
        }

        gnss_en = pin_init(&gnss_en_node);

        //bt_int = pin_init(&bt_int_node);

	printk("The MQTT protec3 PoC started\n");

/*
		err = nrf_modem_lib_init();
		if (err) {
			printk("Failed to initialize modem library!");
			return;
		}
*/
        lte_lc_power_off();

        err = client_id_get();
        if (err) {
                printk("ERROR: Failed to read IMEI.\n");
                return;
        }
        printk("Client Id: %s\n", imei);

	/* Initialize AT comms in order to provision the certificate */
	err = at_comms_init();
	if (err) {
		return;
	}

	/* Provision certificates before connecting to the LTE network */
	err = cert_provision1(sec_tag_list[0], MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN, ca_cert, sizeof(ca_cert));
	if (err) {
		return;
	}

        err = cert_provision1(sec_tag_list[0], MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT, private_cert, sizeof(private_cert));
	if (err) {
		return;
	}

        err = cert_provision1(sec_tag_list[0], MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT, private_key, sizeof(private_key));
	if (err) {
		return;
	}

	modem_configure();

        sensors_init();

        alarm_loud = pin_init(&alarm_loud_node);

        button0 = pin_int_init(&button0_node, &button_cb_data, button_pressed);
        button_debounce_service.device = button0;
        button_debounce_service.gpio_node = &button0_node;

        gnss_uart_dev = device_get_binding(UART_DEVICE_NAME);
        if (gnss_uart_dev) {
              printk("Uart %s initialized successfully\n", UART_DEVICE_NAME);

              uart_irq_callback_set(gnss_uart_dev, gnss_uart_cb);
              uart_irq_rx_enable(gnss_uart_dev);

              gnss_uart_service.device = gnss_uart_dev;

              gpio_pin_set(gnss_en, gnss_en_node.gpio_pin, 1);
        }
        else {
              printk("Error initialzing uart %s\n", UART_DEVICE_NAME);
              //error(2);
        }

        if (ui_buzzer_init() == 0) {
              printk("Buzzer initialized in silent mode\n");
        }
        else {
              printk("Buzzer initialization failed\n");
        }

	client_init(&client);

        while (1) {
              err = mqtt_connect(&client);
              if (err != 0) {
                      printk("ERROR: mqtt_connect %d\n", err);
                      //return;
              }
              else {

                      err = fds_init(&client);
                      if (err != 0) {
                              printk("ERROR: fds_init %d\n", err);
                              //return;
                      }
                      else {
                              while (1) {
                                      err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
                                      if (err < 0) {
                                              printk("ERROR: poll %d\n", errno);
                                              break;
                                      }

                                      err = mqtt_live(&client);
                                      if ((err != 0) && (err != -EAGAIN)) {
                                              printk("ERROR: mqtt_live %d\n", err);
                                              break;
                                      }

                                      if ((fds.revents & POLLIN) == POLLIN) {
                                              err = mqtt_input(&client);
                                              if (err != 0) {
                                                      printk("ERROR: mqtt_input %d\n", err);
                                                      break;
                                              }
                                      }

                                      if ((fds.revents & POLLERR) == POLLERR) {
                                              printk("POLLERR\n");
                                              break;
                                      }

                                      if ((fds.revents & POLLNVAL) == POLLNVAL) {
                                              printk("POLLNVAL\n");
                                              break;
                                      }

                              }

                              printk("Disconnecting MQTT client...\n");

                              err = mqtt_disconnect(&client);
                              if (err) {
                                      printk("Could not disconnect MQTT client. Error: %d\n", err);
                              }
                      }
              }
              k_sleep(K_SECONDS(10));
        }
}

/*
static void bt_int_thread() {
    while (1) {
          gpio_pin_set(leds[0], led_nodes[0].gpio_pin, gpio_pin_get(bt_int, bt_int_node.gpio_pin));
          k_sleep(K_MSEC(100));
    };
}

K_THREAD_DEFINE(bt_int_thread, 1024, bt_int_thread, NULL, NULL, NULL, 7, 0, 0);
*/


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
      uint8_t tx_buf[len + 1];
      tx_buf[0] = reg;
      memcpy(&tx_buf[1], bufp, len);

      return i2c_write(*(const struct device **)handle, tx_buf, len + 1, IIS2DLPC_I2C_ADD_L >> 1);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    return i2c_write_read(*(const struct device **)handle, IIS2DLPC_I2C_ADD_L >> 1,
			      &reg, sizeof(reg),
			      bufp, len);
}

//void sensors_thread(void)
void sensors_init()
{
      int err;
      u8_t tmp;
      u16_t tmp16;

      iis2dlpc_reg_t int_route;

      const struct device *pmic_int;

      i2c2_dev = device_get_binding(I2C_BUS_2_DEV_NAME);

      if (i2c2_dev) {
          printk("I2C bus 2 initialized successfully\n");
      }
      else {
          printk("I2C bus 2 failed to initialize\n");
          return;
          //error(1);
      }

      pmic_int = pin_int_init(&pmic_node, &pmic_int_cb_data, pmic_int_occured);
      pmic_service.device = pmic_int;
      pmic_service.gpio_node = &pmic_node;

      /* Set the INTERRUPT_ENABLE1 register */
      adp536x_reg_read(0x34, &tmp);

      err = adp536x_reg_write(0x32, 0x01);
      if (err) {
          printk("Could not enable adp536x INT1: %d\n", err);
      }

      SHTC3_Init(SHTC3_I2C_ADDR, i2c2_dev);

      err = SHTC3_GetId(&tmp16);
      if (err == 0) {
            printk("SHTC3 initialized with id: 0x%02x\n", tmp16);
      }
      else {
            printk("Could not initialize SHTC3 with error: %d\n", err);
      }

      /* Initialize mems driver interface */
      dev_ctx.write_reg = platform_write;
      dev_ctx.read_reg = platform_read;
      dev_ctx.handle = &i2c2_dev;

      /* Check device ID */
      iis2dlpc_device_id_get(&dev_ctx, &tmp);

      if (tmp != IIS2DLPC_ID) {
          printk("Error initializing accelerometer\n");
      }
      else {
          printk("Accelerometer initialized like a boss with id: 0x%02x\n", tmp);

          iis2dlpc_reset_set(&dev_ctx, PROPERTY_ENABLE);

          do {
              iis2dlpc_reset_get(&dev_ctx, &tmp);
          } while (tmp);

          /* Enable Block Data Update */
          iis2dlpc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
          /*Set full scale */
          iis2dlpc_full_scale_set(&dev_ctx, IIS2DLPC_2g);
          /* Configure filtering chain
          * Accelerometer - filter path / bandwidth
          */
          iis2dlpc_filter_path_set(&dev_ctx, IIS2DLPC_LPF_ON_OUT);
          iis2dlpc_filter_bandwidth_set(&dev_ctx, IIS2DLPC_ODR_DIV_4);
          /*Configure power mode */
          iis2dlpc_power_mode_set(&dev_ctx, IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit);

          iis2dlpc_wkup_dur_set(&dev_ctx, 1);
          /* Set sleep duration
           * Duration to go in sleep mode (1 LSb = 512 / ODR)
           */
          iis2dlpc_act_sleep_dur_set(&dev_ctx, 1);
          /*Set Activity wake-up threshold
           * Threshold for wake-up 1 LSB = FS_XL / 64
           */
          iis2dlpc_wkup_threshold_set(&dev_ctx, 3);
          /* Data sent to wake-up interrupt function */
          iis2dlpc_wkup_feed_data_set(&dev_ctx, IIS2DLPC_HP_FEED);
          /* Config activity / inactivity or stationary / motion detection */
          iis2dlpc_act_mode_set(&dev_ctx, IIS2DLPC_DETECT_ACT_INACT);
          /* Enable activity detection interrupt */
          iis2dlpc_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
          int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
          iis2dlpc_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

          /*Set Output Data Rate */
          iis2dlpc_data_rate_set(&dev_ctx, IIS2DLPC_XL_ODR_200Hz);

          accel_int1 = pin_int_init(&accel_int1_node, &accel_int1_cb_data, accel_int1_occured);
          accel_int1_service.device = accel_int1;
          accel_int1_service.gpio_node = &accel_int1_node;
      }

      const struct device *pir_int = pin_int_init(&pir_node, &pir_int_cb_data, pir_int_occured);
      pir_debounce_service.device = pir_int;
      pir_debounce_service.gpio_node = &pir_node;
}


static void gray_thread() {
    while (1) {
        if (armed_status) {
            gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 1);
            k_sleep(K_MSEC(300));
            gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 0);
            k_sleep(K_SECONDS(7));
        }
        else {
            k_sleep(K_SECONDS(1));
        }
    }
}


static void alarm_thread() {
    s64_t alarm_time = 0;

    while (1) {
        if (armed_status) {
            if (!alarm_status) {
                if ((k_uptime_get() - pir_events[0]) < 10000 || accel_wakeup_event) {
                    alarm_status = true;
                    data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/alarm", "ON");
                    alarm_time = k_uptime_get();
                }
            }
            else {
                while(alarm_status) {
                    ui_buzzer_set_frequency(BEEP_FREQUENCY, 100);
                    k_sleep(K_MSEC(1000));
                    ui_buzzer_set_frequency(0, 0);
                    k_sleep(K_MSEC(500));
                    if (k_uptime_get() - alarm_time > 30000) {
                        alarm_status = false;
                        accel_wakeup_event = false;
                        data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status/alarm", "OFF");
                    }
                }
            }
        }

        k_sleep(K_SECONDS(2));
    }
}


//K_THREAD_DEFINE(sensors_thread_id, 4096, sensors_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(gray_thread_id, 1024, gray_thread, NULL, NULL, NULL, 7, 0, 5000);
K_THREAD_DEFINE(alarm_thread_id, 1024, alarm_thread, NULL, NULL, NULL, 7, 0, 10000);
