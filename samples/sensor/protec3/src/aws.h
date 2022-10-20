#ifndef GUARD_AWS_H
#define GUARD_AWS_H

// at least 256
#define MQTT_MESSAGE_BUFFER_SIZE 512

// at least 128
#define MQTT_PAYLOAD_BUFFER_SIZE 512

#define MQTT_CLIENT_USERNAME "protec3.io"
#define MQTT_CLIENT_PASSWORD "kR59Tq72d%.SX}4aze!^1KX_+nJBRy&T"
#define PEER_VERIFY 2
#define MQTT_BROKER_HOSTNAME "protec3.io"
#define MQTT_BROKER_PORT 5883

struct rsrp_data
{
	uint16_t value;
	uint16_t offset;
};

extern struct mqtt_client client;


#endif /* GUARD_AWS_H */
