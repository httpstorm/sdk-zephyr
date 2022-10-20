#ifndef GUARD_MQTT_HANDLER_H
#define GUARD_MQTT_HANDLER_H

#include <net/mqtt.h>


#define BEEP_TIME_MAX 10000

#define SUBTOPIC_MAX_LENGTH 32
#define PAYLOAD_MAX_LENGTH 32


enum MQTT_REQUEST
{
	MQTT_REQUEST_INVALID = -1,
	MQTT_REQUEST_OFF = 0,
	MQTT_REQUEST_ON = 1,
	MQTT_REQUEST_GET = 2,

	// mark the end of the enum
	MQTT_REQUEST__END_MARKER,
};

enum BEEP_TYPE
{
	BEEP_TYPE_NORMAL = 0,
	BEEP_TYPE_LOUD = 1,
	BEEP_TYPE_INVALID = 2
};

typedef struct mqtt_work_piece
{
	struct k_work work;
	char subtopic[SUBTOPIC_MAX_LENGTH];
	size_t subtopic_len;
	char payload[SUBTOPIC_MAX_LENGTH];
	size_t payload_len;
	struct mqtt_client * client;
} mqtt_work_piece_t;


extern uint8_t alarm_arm;


int mqtt_publish_h(struct mqtt_client * client, const char * topic, size_t topic_count, const char * payload, size_t count);
void mqtt_work_queue_init(void);

int data_publish1(
	struct mqtt_client * c,
	enum mqtt_qos qos,
	const uint8_t * subtopic,
	/*const */uint8_t * data
);

int set_beep_time(enum BEEP_TYPE type, uint16_t length);
uint16_t get_beep_time(enum BEEP_TYPE type);

int set_beep_time(enum BEEP_TYPE type, uint16_t length);
uint16_t get_beep_time(enum BEEP_TYPE type);

int add_user(const char * mac);
int del_user(const char * mac);

int mqtt_pub_btn_press(void);
int mqtt_pub(const char * topic, /*const */char * msg);

#endif /* GUARD_MQTT_HANDLER_H */
