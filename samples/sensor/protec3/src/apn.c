#include "apn.h"
#include "uart.h"
#include <modem/lte_lc.h>
#include <string.h>
#include <stdio.h>

const char * apn_list[] =
{
	// BG, A1
	"iot-test",

	// world, Aeris Global
	"iot.aer.net",

	// world, Sierra Global
	"lp.swir",

	// US, AT&T by Sierra
	"nmrx11.com.attz",
};

// Please group by APN so that apn_list is better sorted
struct apn_table_entry apn_table[] =
{
	// BG, A1, TODO
	{"8935901", 0},

	// world Aeirs Global
	{"891850", 1},

	// US, AT&T by Aeris
	{"890117022", 1},

	// world, Sierra Global
	{"893324", 2},

	// world, Sierra Global (eSIM)
	{"893325", 2},

	// US, AT&T by Sierra
	{"890117032", 3},
};

char out_buf[512];

const size_t apn_list_count = sizeof(apn_list) / sizeof(*apn_list);
const size_t apn_table_count = sizeof(apn_table) / sizeof(*apn_table);

struct json_obj_descr apn_get_req_descr[] =
{
	JSON_OBJ_DESCR_PRIM(struct apn_get_req, config, JSON_TOK_STRING)
};

struct json_obj_descr apn_set_req_descr[] =
{
	JSON_OBJ_DESCR_PRIM(struct apn_set_req, config, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct apn_set_req, select, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct apn_set_req, APN, JSON_TOK_STRING)
};

int apn_read(struct eeprom_apn_config * apn_config, const char * iccid)
{
	// read APN config from EEPROM
	memset(apn_config, 0, sizeof(*apn_config));
	int err = eeprom_read(eeprom_dev, EEPROM_ADDRESS_APN, apn_config, sizeof(*apn_config));

	if (OK != err)
	{
		printk("apn_read()  EEPROM read failed\n");
		return false;
	}

	bool apn_valid = false;
	uint32_t i = 0;

	for (; i < sizeof(apn_config->APN); i++)
	{
		if (!apn_config->APN[i])
		{
			apn_valid = true;
			break;
		}
	}

	for (; i < sizeof(apn_config->APN); i++)
	{
		if (apn_config->APN[i])
		{
			apn_valid = false;
			break;
		}
	}

	if (apn_valid)
	{
		if (!apn_config->APN[0])
		{
			if (apn_config->select && (apn_config->select < apn_list_count))
			{
				size_t count = strlen(apn_list[apn_config->select]);

				if (count && (count < APN_MAX_LENGTH))
				{
					memset(apn_config->APN, 0, sizeof(apn_config->APN));
					memcpy(apn_config->APN, apn_list[apn_config->select], count);

					// update the APN config in EEPROM
					err = eeprom_write(eeprom_dev, EEPROM_ADDRESS_APN, apn_config, sizeof(*apn_config));

					if (OK != err)
					{
						printk("apn_read()  EEPROM write updated APN failed\n");
					}
				}
			}
		}

		if (apn_config->APN[0])
		{
			printk("APN[%u] %s\n", apn_config->select, apn_config->APN);

			// overwrite the APN init parameters in lte_lc.c
			// static char cgdcont[144]
			// depricated API
			//- lte_lc_pdp_context_set(LTE_LC_PDP_TYPE_IP, apn_config->APN, false, false, false);
			printk("depricated API: lte_lc_pdp_context_set(LTE_LC_PDP_TYPE_IP, ...)\n");
		}
		else
		{
			printk("APN is blank\n");
			apn_valid = false;
		}
	}
	else
	{
		printk("APN not valid, setting to blank\n");
		memset(apn_config, 0, sizeof(*apn_config));
		err = eeprom_write(eeprom_dev, EEPROM_ADDRESS_APN, apn_config, sizeof(*apn_config));

		if (OK != err)
		{
			printk("apn_read()  EEPROM write blank APN failed\n");
		}
	}

	if (!apn_valid)
	{
		// fall to back to use apn_table
		for (i = 0; i < apn_table_count; ++i)
		{
			size_t len = 0;
			bool found = true;

			while (len < APN_MAX_IDENTIFIER_LENGTH)
			{
				if ('\0' == apn_table[i].identifier[len])
				{
					// we have reached end of identifier
					break;
				}

				if (apn_table[i].identifier[len] != iccid[len])
				{
					found = false;
					break;
				}

				len += 1;
			}

			if (found)
			{
				// depricated API
				//- lte_lc_pdp_context_set(
				//- 	LTE_LC_PDP_TYPE_IP,
				//- 	apn_list[apn_table[i].select],
				//- 	false,
				//- 	false,
				//- 	false
				//- );
				printk("depricated API: lte_lc_pdp_context_set(LTE_LC_PDP_TYPE_IP, ...)\n");

				printk(
					"Used APN \"%s\" for ICCID %s\n",
					apn_list[apn_table[i].select],
					iccid
				);

				break;
			}
		}
	}

	return apn_valid;
}

void apn_get(
	char * raw_buf,
	size_t len,
	struct eeprom_apn_config * apn_config
)
{
	struct apn_get_req req;

	int res = json_obj_parse(
		raw_buf,
		len,
		apn_get_req_descr,
		1,
		&req
	);

	if (1 == res)
	{
		if (0 == strcmp("eeprom", req.config))
		{
			snprintf(
				out_buf,
				sizeof(out_buf),
				"apn_eeprom={\"select\": %d, \"APN\": \"%s\"}\n",
				apn_config->select,
				apn_config->APN
			);

			uart_fota_send(out_buf, strlen(out_buf));
		}
		else if (0 == strcmp("list", req.config))
		{
			int i;

			size_t l = snprintf(
				out_buf,
				sizeof(out_buf),
				"apn_list={\"list\": ["
			);

			// len now points to the terminating NULLВ byte
			for (i = 0; i < apn_list_count; ++i)
			{
				l += snprintf(
					out_buf + l,
					sizeof(out_buf) - l,
					"\"%s\"",
					apn_list[i]
				);

				if (i != apn_list_count - 1)
				{
					l += snprintf(out_buf + l, sizeof(out_buf) - l, ", ");
				}
			}

			l += snprintf(out_buf + l, sizeof(out_buf) - l, "]}\n");
			uart_fota_send(out_buf, l);
		}
	}
}

int apn_set(char * raw_buf, size_t len, struct eeprom_apn_config * apn_config)
{
	struct apn_set_req req;

	int res = json_obj_parse(
		raw_buf,
		len,
		apn_set_req_descr,
		3,
		&req
	);

	if ((7 == res) && (0 == strcmp("eeprom", req.config)))
	{
		size_t apn_length = strlen(req.APN);

		// NULL terminator
		if (apn_length >= APN_MAX_LENGTH)
		{
			printk("apn_error={\"status\":\"APN too long\"}\n");
			return FAIL;
		}

		apn_config->select = req.select;
		memset(apn_config->APN, 0, sizeof(apn_config->APN));
		memcpy(apn_config->APN, req.APN, MIN(sizeof(apn_config->APN), apn_length));

		// update the APN config in EEPROM
		int status = eeprom_write(eeprom_dev, EEPROM_ADDRESS_APN, apn_config, sizeof(*apn_config));

		if (status)
		{
			printk("apn_error={\"status\":\"eeprom_write failed\"}\n");
		}
		else
		{
			// depricated API
			//- lte_lc_pdp_context_set(LTE_LC_PDP_TYPE_IP, apn_config->APN, false, false, false);
			printk("depricated API: lte_lc_pdp_context_set(LTE_LC_PDP_TYPE_IP, ...)\n");
		}

		return status;

	}

	return FAIL;
}
