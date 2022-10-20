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
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/at_cmd.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <power/reboot.h>

#include <adp536x-mod.h>
#include <buzzer.h>

/***************** FOTA ****************/
#include <logging/log.h>
#include <net/fota_download.h>
#include <dfu/mcuboot.h>
#include <net/download_client.h>
#include <dfu/dfu_target.h>
#include <pm_config.h>
#include <mbedtls/sha1.h>
#include <data/json.h>
#include "fota_update.h"
/***************** END FOTA ***********/


#define OK 0
#define FAIL -1


/************************ FOTA ***********************/
LOG_MODULE_REGISTER(fota_update, CONFIG_FOTA_DOWNLOAD_LOG_LEVEL);

struct k_work	fota_work;
static struct download_client udc;
static fota_download_callback_t callback;
//static bl_sha256_ctx_t ctx256;
static mbedtls_sha1_context ctx1;
static mbedtls_sha1_context ctx_packet;
static uint8_t sha1_expected[20];
static fota_finish_t ff;

struct upload_data
{
	int offset;
	int count;
	const char * checksum;
};

struct json_obj_descr upload_descr[] =
{
	JSON_OBJ_DESCR_PRIM(struct upload_data, offset, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct upload_data, count, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct upload_data, checksum, JSON_TOK_STRING)
};

/**@brief Start transfer of the file. */
static void app_dfu_transfer_start(struct k_work * unused)
{
	int retval;
	int sec_tag;
	char * apn = NULL;

#ifndef CONFIG_USE_HTTPS
	sec_tag = -1;
#else
	sec_tag = TLS_SEC_TAG;
#endif

	retval = fota_download_start(
		CONFIG_UPDATE_HOST,
		CONFIG_UPDATE_FILE,
		sec_tag,
		apn,
		0
	);

	if (retval != 0)
	{
		/* Re-enable button callback */
		printk("fota_download_start() failed, err %d  %s\n", -retval, strerror(-retval));
	}

}

static void fota_dl_handler(const struct fota_download_evt * evt)
{
	volatile int stop = 1;

	switch (evt->id)
	{
	case FOTA_DOWNLOAD_EVT_ERROR:
		printk("Received error from fota_download\n");
#if 0
		printk("Restartng...\n");
		sys_reboot(SYS_REBOOT_WARM);

		while (stop);
#endif

		break;

	case FOTA_DOWNLOAD_EVT_FINISHED:
		printk("FOTA download succeeded\n\r");
		printk("Restartng...\n");
		//?boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
		sys_reboot(SYS_REBOOT_WARM);

		while (stop);

		break;

	default:
		break;
	}
}

void fota_dl_init(void)
{
	k_work_init(&fota_work, app_dfu_transfer_start);
	int err = fota_download_init(fota_dl_handler);

	if (0 != err)
	{
		printf("fota init failed %i\n\r", err);
		return;
	}

	//printk("FIRMWARE VERSION 2\n");
}

void fota_dl_trigger(void)
{
	printk("FOTA\n");
	//k_sleep(K_MSEC(1000));
	k_work_submit(&fota_work);

	while (1)
	{
		// let fota_work run
		k_sleep(K_MSEC(1000));
	}
}

static void send_evt(enum fota_download_evt_id id)
{
	__ASSERT(id != FOTA_DOWNLOAD_EVT_PROGRESS, "use send_progress");
	__ASSERT(id != FOTA_DOWNLOAD_EVT_ERROR, "use send_error_evt");

	const struct fota_download_evt evt =
	{
		.id = id
	};

	callback(&evt);
}

static void send_error_evt(enum fota_download_error_cause cause)
{
	__ASSERT(cause != FOTA_DOWNLOAD_ERROR_CAUSE_NO_ERROR, "use a valid error cause");

	const struct fota_download_evt evt =
	{
		.id = FOTA_DOWNLOAD_EVT_ERROR,
		.cause = cause
	};

	callback(&evt);
}

static void send_progress(int progress)
{
#ifdef CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT
	const struct fota_download_evt evt =
	{
		.id = FOTA_DOWNLOAD_EVT_PROGRESS,
		.progress = progress
	};

	callback(&evt);
#endif
}

static void dfu_target_callback_handler(enum dfu_target_evt_id evt)
{
	switch (evt)
	{
	case DFU_TARGET_EVT_TIMEOUT:
		send_evt(FOTA_DOWNLOAD_EVT_ERASE_PENDING);
		break;

	case DFU_TARGET_EVT_ERASE_DONE:
		send_evt(FOTA_DOWNLOAD_EVT_ERASE_DONE);
		break;

	default:
		send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);
	}
}

static int download_client_callback(const struct download_client_evt * event)
{
	static bool first_fragment = true;
	static size_t file_size;
	size_t offset;
	int err;

	if (event == NULL)
	{
		return -EINVAL;
	}

	switch (event->id)
	{
	case DOWNLOAD_CLIENT_EVT_FRAGMENT:
	{
		if (first_fragment)
		{
			err = download_client_file_size_get(&udc, &file_size);

			if (err != 0)
			{
				LOG_DBG("download_client_file_size_get err: %d", err);
				send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);
				return err;
			}

			first_fragment = false;
			int img_type = dfu_target_img_type(event->fragment.buf, event->fragment.len);
			err = dfu_target_init(img_type, file_size, dfu_target_callback_handler);

			if ((err < 0) && (err != -EBUSY))
			{
				LOG_ERR("dfu_target_init error %d", err);
				send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);
				int res = dfu_target_reset();

				if (res != 0)
				{
					LOG_ERR("Unable to reset DFU target");
				}

				first_fragment = true;

				return err;
			}
		}

		err = dfu_target_write(event->fragment.buf, event->fragment.len);

		if (err != 0)
		{
			LOG_ERR("dfu_target_write error %d", err);
			int res = dfu_target_done(false);

			if (res != 0)
			{
				LOG_ERR("Unable to free DFU target resources");
			}

			first_fragment = true;
			send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_INVALID_UPDATE);

			return err;
		}

		if (IS_ENABLED(CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT) && !first_fragment)
		{
			err = dfu_target_offset_get(&offset);

			if (err != 0)
			{
				LOG_DBG("unable to get dfu target offset err: %d", err);
				send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);

				return err;
			}

			if (file_size == 0)
			{
				LOG_DBG("invalid file size: %d", file_size);
				send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);

				return err;
			}

			send_progress((offset * 100) / file_size);
			LOG_DBG("Progress: %d/%d%%", offset, file_size);
		}

		break;
	}

	case DOWNLOAD_CLIENT_EVT_DONE:
		err = dfu_target_done(true);

		if (err != 0)
		{
			LOG_ERR("dfu_target_done error: %d", err);
			send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);

			return err;
		}

		send_evt(FOTA_DOWNLOAD_EVT_FINISHED);
		first_fragment = true;

		break;

	case DOWNLOAD_CLIENT_EVT_ERROR:
	{
		/* In case of socket errors we can return 0 to retry/continue,
		 * or non-zero to stop
		 */
		LOG_ERR("Download client error");
		err = dfu_target_done(false);

		if (err == -EACCES)
		{
			LOG_DBG("No DFU target was initialized");
		}
		else if (err != 0)
		{
			LOG_ERR("Unable to deinitialze resources used by dfu_target.");
		}

		first_fragment = true;
		send_error_evt(FOTA_DOWNLOAD_ERROR_CAUSE_DOWNLOAD_FAILED);

		/* Return non-zero to tell download_client to stop */
		return event->error;
	}

	default:
		break;
	}

	return 0;
}



static int fragment_evt_send(const struct download_client * client)
{
	__ASSERT(client->offset <= CONFIG_DOWNLOAD_CLIENT_BUF_SIZE, "Buffer overflow!");

	const struct download_client_evt evt =
	{
		.id = DOWNLOAD_CLIENT_EVT_FRAGMENT,
		.fragment =
		{
			.buf = client->buf,
			.len = client->offset,
		}
	};

	return client->callback(&evt);
}

static int error_evt_send(const struct download_client * dl, int error)
{
	/* Error will be sent as negative. */
	__ASSERT_NO_MSG(error > 0);

	const struct download_client_evt evt =
	{
		.id = DOWNLOAD_CLIENT_EVT_ERROR,
		.error = -error
	};

	return dl->callback(&evt);
}

void uart_download_thread(void * client, void * recv_func, void * send_func)
{
	int64_t time;
	int rc = 0;
	int zero_counter;
	size_t len, req_len;
	char send_buf[128];
	uint8_t check[20];
	uint8_t sha1_computed[20];
	int json_mode;
	struct download_client * const dl = client;
	fota_recv_t fota_recv = recv_func;
	fota_send_t fota_send = send_func;

restart_and_suspend:
	k_thread_suspend(dl->tid);
	mbedtls_sha1_free(&ctx1);
	mbedtls_sha1_init(&ctx1);
	mbedtls_sha1_starts_ret(&ctx1);

	//fota_recv(dl->buf, sizeof(dl->buf), 0, 0);

	time = k_uptime_get();
	json_mode = 1;
	dl->offset = 0;
	dl->progress = 0;
	zero_counter = 0;
	req_len = 128;
	fota_send("update_status={\"status\": \"download-image\"}\n", 43);

	// update_download={"count": 128, "offset": 0, "size": 196358, "progress": 72}
	sprintf(
		send_buf,
		"update_download={\"count\": %u, \"offset\": %u, \"size\": %u, \"progress\": %u}\n",
		MIN(128, (size_t)dl->file_size),
		dl->progress,
		dl->file_size,
		(dl->progress * 100) / dl->file_size
	);

	fota_send(send_buf, strlen(send_buf));

	while (true)
	{
#if 0
		__ASSERT(dl->offset < sizeof(dl->buf), "Buffer overflow");

		if (sizeof(dl->buf) - dl->offset == 0)
		{
			LOG_ERR("Buffer not sufficient (> %d)", sizeof(dl->buf));
			error_evt_send(dl, E2BIG);

			break;
		}

		LOG_DBG(
			"Receiving up to %d bytes at %p...",
			sizeof(dl->buf) - dl->offset,
			dl->buf + dl->offset
		);

#endif

		/*
		len = uart_fifo_read(
			(const struct device*)dl->host,
			dl->buf,
			MIN(sizeof(dl->buf), dl->file_size - dl->progress)
		);
		*/
		len = fota_recv(dl->buf + dl->offset, MIN(sizeof(dl->buf), req_len - dl->offset), json_mode, 0);

		/*len = recv(dl->fd, dl->buf + dl->offset, sizeof(dl->buf) - dl->offset, 0);*/

		if (len == -1)
		{
			/* We just had an unexpected socket error or closure */

			/* If there is a partial data payload in our buffer,
			 * and it has been accounted in our progress, we have
			 * to hand it to the application before discarding it.
			 */

			LOG_ERR("Error in fgets");

			/* Notify the application of the error via en event.
			 * Attempt to reconnect and resume the download
			 * if the application returns Zero via the event.
			 */
			rc = error_evt_send(dl, ECONNRESET);

			if (rc)
			{
				/* Restart and suspend */
				break;
			}

			dl->offset = 0;
			zero_counter = 0;
			continue;
		}

		if (0 == len)
		{
			// more than 1000 ms passed?
			if (k_uptime_get() - time > 1000)
			{
				++zero_counter;

				if (10 <= zero_counter)
				{
					// more than 10 s passed, assume connection is down
					LOG_WRN("No data for 10 s!\n");
					error_evt_send(dl, ECONNRESET);
					break;
				}

				//В assume something went wrong and we must resend
				json_mode = 1;
				req_len = 128;
				dl->offset = 0;
				time = k_uptime_get();

				sprintf(
					send_buf,
					"update_download={\"count\": %u, \"offset\": %u, \"size\": %u, \"progress\": %u}\n",
					MIN(128, (size_t)dl->file_size - dl->progress),
					dl->progress,
					dl->file_size,
					(dl->progress * 100) / dl->file_size
				);

				fota_send(send_buf, strlen(send_buf));
			}

			continue;
		}

		if (json_mode)
		{
			struct upload_data upload;
			size_t json_len;
			int i;
			const char upload_cmd[] = "update_upload=";

			char * end = strstr(dl->buf, "}");

			if (!end)
			{
				continue;
			}

			// line break
			end[1] = 0;

			// TODO what if we have two character line break
			json_len = strlen(dl->buf);

			int res = memcmp(upload_cmd, dl->buf, sizeof(upload_cmd) - 1);

			if (0 == res)
			{
				res = json_obj_parse(
					dl->buf + sizeof(upload_cmd) - 1,
					len,
					upload_descr,
					3,
					&upload
				);

				if (7 == res)
				{
					if (upload.offset != dl->progress)
					{
						// not the packet we expected;
						continue;
					}

					for (i = 0; 20 != i; ++i)
					{
						check[i] = strtou8(&upload.checksum[2 * i]);
					}

					req_len = upload.count;

					if (json_len + 1 < len)
					{
						// we have already received payload
						memmove(dl->buf, dl->buf + json_len + 1, len - json_len - 1);
						dl->offset = len - json_len - 1;
						req_len -= dl->offset;
					}

					json_mode = 0;
					mbedtls_sha1_free(&ctx_packet);
					mbedtls_sha1_init(&ctx_packet);
					mbedtls_sha1_starts_ret(&ctx_packet);
				}
			}
		}
		else
		{
			dl->offset += len;

			if (dl->offset < req_len)
			{
				continue;
			}

			//LOG_DBG("Read %d bytes from uart", len);

			//dl->offset = len;

			mbedtls_sha1_update_ret(&ctx_packet, dl->buf, dl->offset);
			rc = mbedtls_sha1_finish_ret(&ctx_packet, sha1_computed);

			if (rc || memcmp(check, sha1_computed, 20))
			{
				// checksum does not match
				k_msleep(1000);
				time = k_uptime_get();
				json_mode = 1;
				req_len = 128;
				zero_counter = 0;
				dl->offset = 0;

				LOG_WRN("SHA1 of chunk failed.");

				// flush input buffer
				fota_recv(NULL, 0, 0, 1);

				sprintf(
					send_buf,
					"update_download={\"count\": %u, \"offset\": %u, \"size\": %u, \"progress\": %u}\n",
					MIN(128, (size_t)dl->file_size - dl->progress),
					dl->progress,
					dl->file_size,
					(dl->progress * 100) / dl->file_size
				);

				fota_send(send_buf, strlen(send_buf));

				continue;
			}

			dl->progress += dl->offset;

			/* Send fragment to application.
			 * If the application callback returns non-zero, stop.
			 */
			mbedtls_sha1_update_ret(&ctx1, dl->buf, dl->offset);

			if (dl->progress == dl->file_size)
			{
				rc = mbedtls_sha1_finish_ret(&ctx1, sha1_computed);

				if (0 == rc)
				{
					rc = memcmp(sha1_computed, sha1_expected, 20);
				}

				if (0 != rc)
				{
					// SHA1 failed
					fota_send("update_status={\"status\": \"checksum-invalid\"}\n", 45);
					error_evt_send(dl, EINVAL);

					break;
				}
			}

			rc = fragment_evt_send(dl);

			if (rc)
			{
				/* Restart and suspend */
				LOG_INF("Fragment refused, download stopped.");

				break;
			}

			if (dl->progress == dl->file_size)
			{
				LOG_INF("Download complete");

				const struct download_client_evt evt =
				{
					.id = DOWNLOAD_CLIENT_EVT_DONE,
				};

				fota_send("update_status={\"status\": \"checksum-valid\"}\n", 43);
				fota_send("update_status={\"status\": \"restarting\"}\n", 39);
				fota_update_notify();
				dl->callback(&evt);

				/* Restart and suspend */
				break;
			}
			else
			{
				// flush input
				fota_recv(NULL, 0, 0, 1);

				time = k_uptime_get();
				json_mode = 1;
				req_len = 128;
				zero_counter = 0;
				dl->offset = 0;

				sprintf(
					send_buf,
					"update_download={\"count\": %u, \"offset\": %u, \"size\": %u, \"progress\": %u}\n",
					MIN(128, (size_t)dl->file_size - dl->progress),
					dl->progress,
					dl->file_size,
					(dl->progress * 100) / dl->file_size
				);

				fota_send(send_buf, strlen(send_buf));
			}
		}
	}

	ff();

	/* Do not let the thread return, since it can't be restarted */
	goto restart_and_suspend;
}

void fota_uart_init(fota_recv_t fota_recv, fota_send_t fota_send)
{
	callback = fota_dl_handler;
	udc.fd = -1;
	udc.callback = download_client_callback;

	/* The thread is spawned now, but it will suspend itself;
	 * it is resumed when the download is started via the API.
	 */
	udc.tid = k_thread_create(
		&udc.thread,
		udc.thread_stack,
		K_THREAD_STACK_SIZEOF(udc.thread_stack),
		uart_download_thread,
		&udc,
		fota_recv,
		fota_send,
		0, /* prio */
		0,
		K_NO_WAIT
	);
}

void fota_uart_start(size_t file_size, uint8_t sha1[20], fota_finish_t fota_finish)
{
	udc.file_size = file_size;
	memcpy(sha1_expected, sha1, 20);
	ff = fota_finish;
	k_thread_resume(udc.tid);
}
/*************************** END FOTA ************************/
