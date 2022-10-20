#ifndef GUARD_MAIN_H
#define GUARD_MAIN_H

struct gpio_struct
{
	const char * gpio_dev_name;
	const char * gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

#define I2C_BUS_2_DEV_NAME      DT_LABEL(DT_NODELABEL(i2c2))
#define RTC_I2C_DEV_NAME        DT_LABEL(DT_NODELABEL(i2c3))
#define EEPROM0_DEVICE_NAME     DT_LABEL(DT_NODELABEL(eeprom0))
#define UART_DEVICE_NAME        DT_LABEL(DT_NODELABEL(uart1))

#define SHTC3_I2C_ADDR 0x70
#define RTC_I2C_ADDR 0x68
#define EEPROM_I2C_BASE_ADDRESS 0x50

#define ICCID_BUF_LEN 40
#define CGSN_RESP_LEN 19
#define IMEI_LEN 15
#define CLIENT_ID_LEN (IMEI_LEN + 1)

#define FLAGS_OR_ZERO(node)						        \
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),    \
	(DT_GPIO_FLAGS(node, gpios)),			            \
	(0))

#define PIN_STRUCT(alias, type) \
{                                                                   \
	.gpio_dev_name = DT_GPIO_LABEL(DT_ALIAS(alias), gpios),         \
	.gpio_pin_name = DT_LABEL(DT_ALIAS(alias)),                     \
	.gpio_pin = DT_GPIO_PIN(DT_ALIAS(alias), gpios),                \
	.gpio_flags = type | FLAGS_OR_ZERO(DT_ALIAS(alias)),            \
}

#define PIN_OUTPUT(alias) PIN_STRUCT(alias, GPIO_OUTPUT)
#define PIN_INPUT(alias) PIN_STRUCT(alias, GPIO_INPUT)

extern struct k_sem thread_sync_sem;
extern uint8_t client_id_buf[CLIENT_ID_LEN];

int lte_init(void);

#endif /* GUARD_MAIN_H */
