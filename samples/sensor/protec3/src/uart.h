#ifndef GUARD_UART_H
#define GUARD_UART_H

#include <zephyr.h>
#include "fota_update.h"



enum uart_modes
{
	CMD_MODE,
	NEWLINE_MODE,
	RAW_MODE
};

extern struct k_work uart_work;

void uart_worker(struct k_work * work);
void my_uart_isr(const struct device * dev, void * user_data);
void my_uart_init(const struct device * uart_dev);
void uart_set_mode(enum uart_modes mode);

void fota_uart_init(fota_recv_t fota_recv, fota_send_t fota_send);
int uart_fota_send(const uint8_t * buf, size_t len);
int uart_fota_recv(uint8_t * buf, size_t len, int look_for_newline, int flush);

void button_int(void);

#endif /*GUARD_UART_H*/
