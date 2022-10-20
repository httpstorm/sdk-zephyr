#ifndef GUARD_APN_H
#define GUARD_APN_H

#include <zephyr.h>
#include <data/json.h>
#include "eeprom.h"

#define APN_MAX_LENGTH 32
#define APN_MAX_IDENTIFIER_LENGTH 9

struct apn_table_entry
{
#if 0
	uint8_t length;
	char identifier[APN_MAX_IDENTIFIER_LENGTH];
	char apn[APN_MAX_LENGTH];
#else
	char identifier[APN_MAX_IDENTIFIER_LENGTH];
	uint8_t select;
#endif
};

// APN config in EEPROM
struct eeprom_apn_config
{
	// 0 custom APN
	// 1-255 predefined APNs stored in the firmware
	uint8_t select;
	char APN[APN_MAX_LENGTH];
};

struct apn_get_req
{
	const char * config;
};

struct apn_set_req
{
	const char * config;
	int select;
	const char * APN;
};

int apn_read(struct eeprom_apn_config * apn_config, const char * iccid);

void apn_get(
	char * raw_buf,
	size_t len,
	struct eeprom_apn_config * apn_config
);

int apn_set(char * raw_buf, size_t len, struct eeprom_apn_config * apn_config);

#endif /* GUARD_APN_H */
