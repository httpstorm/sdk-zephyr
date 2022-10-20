/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
/**@file
 *
 * @brief   Buzzer control for the User Interface module. The module uses PWM to
 *	    control the buzzer output frequency.
 */

#ifndef UI_BUZZER_H__
#define UI_BUZZER_H__

#include <zephyr.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int64_t s64_t;

/**@brief Initialize buzzer in the user interface module. */
int ui_buzzer_init(void);
int ui_buzzer_set_frequency(u32_t frequency, u8_t intensity);

#ifdef __cplusplus
}
#endif

#endif /* UI_BUZZER_H__ */
