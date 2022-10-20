#ifndef GUARD_GPIO_T_H
#define GUARD_GPIO_T_H

#include <sys/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>


#define OK 0
#define FAIL -1


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
#elif CONFIG_BOARD_PROTEC3_NRF9160_NS
#define LED_R_NODE DT_ALIAS(led0)
#define LED_G_NODE DT_ALIAS(led1)
#define LED_B_NODE DT_ALIAS(led2)
#define LED_SR_NODE DT_ALIAS(led0)
#define LED_SG_NODE DT_ALIAS(led1)
#define LED_SB_NODE DT_ALIAS(led2)
#else
#error "Unsupported board: please use nRF9160dk or Thingy:91"
#endif

#if DT_NODE_HAS_STATUS(LED_R_NODE, okay)
#define LED_R	  DT_GPIO_LABEL(LED_R_NODE, gpios)
#define LED_G	  DT_GPIO_LABEL(LED_G_NODE, gpios)
#define LED_B	  DT_GPIO_LABEL(LED_B_NODE, gpios)
#define LED_SR	  DT_GPIO_LABEL(LED_SR_NODE, gpios)
#define LED_SG	  DT_GPIO_LABEL(LED_SG_NODE, gpios)
#define LED_SB	  DT_GPIO_LABEL(LED_SB_NODE, gpios)
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

static const char * led_names[] = { LED_R, LED_G, LED_B, LED_SR, LED_SG, LED_SB };
static const gpio_pin_t led_pins[] = { PIN_R, PIN_G, PIN_B, PIN_SR, PIN_SG, PIN_SB };
static const gpio_flags_t led_flags[] = { FLAGS_R, FLAGS_G, FLAGS_B, FLAGS_SR, FLAGS_SG, FLAGS_SB };

#define LED_COUNT (sizeof(led_names) / sizeof(*led_names))
#define LED_COUNT_PINS (sizeof(led_pins) / sizeof(*led_pins))
#define LED_COUNT_FLAGS (sizeof(led_flags) / sizeof(*led_flags))
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

struct PIN_obj_t
{
	const struct device * dev;
	gpio_pin_t pin;
};

struct LED_t
{
	struct PIN_obj_t r;
	struct PIN_obj_t g;
	struct PIN_obj_t b;
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

		struct PIN_obj_t list[LED_COUNT];
	};
};

extern struct LEDS_t led;

int led_init(void);
int pin_get(struct PIN_obj_t pin);
int pin_set(struct PIN_obj_t led, int on);
int led_set_rgb(struct LED_t * led, int r, int g, int b);


// BUTTONS
#if CONFIG_BOARD_PROTEC3_NRF9160_NS
#define BTN_0_NODE DT_ALIAS(button)
#else
#define BTN_0_NODE DT_ALIAS(sw0)
#endif

#if DT_NODE_HAS_STATUS(BTN_0_NODE, okay)
#define BTN_0		DT_GPIO_LABEL(BTN_0_NODE, gpios)
#define BTN_0_PIN	DT_GPIO_PIN(BTN_0_NODE, gpios)
#define BTN_0_FLAGS	DT_GPIO_FLAGS(BTN_0_NODE, gpios)
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

extern struct PIN_obj_t btn_0;
extern int btn_0_trigger;

#endif /* GUARD_GPIO_T_H */
