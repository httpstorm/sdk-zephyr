#ifndef GUARD_FOTA_UPDATE_H
#define GUARD_FOTA_UPDATE_H

// receive len bytes into buf; if look_for_newline is set, only if a newline was received
// if flush is set flush buffer instead
typedef int (*fota_recv_t)(uint8_t * buf, size_t len, int look_for_newline, int flush);

// send len bytes from buf
typedef int (*fota_send_t)(const uint8_t * buf, size_t len);

//clean up duties in case of error, e. g. disconnect
typedef void (*fota_finish_t)(void);

void fota_uart_start(size_t file_size, uint8_t sha1[20], fota_finish_t fota_finish);
void fota_uart_init(fota_recv_t fota_recv, fota_send_t fota_send);
void fota_update_notify(void);

void fota_dl_init(void);
void fota_dl_trigger(void);

uint8_t strtou8(const char * s);

#endif /* GUARD_FOTA_UPDATE_H */
