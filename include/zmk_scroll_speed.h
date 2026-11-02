/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 *
 * Runtime control API for the per-host scroll speed input processor
 * (compatible "zmk,input-processor-scroll-speed"). The scroll speed is kept
 * separately for USB and for each BLE profile, so a host whose apps scroll a
 * long way per wheel notch can be slowed down without affecting the others.
 * Changes are clamped and (when CONFIG_SETTINGS is enabled) persisted to flash.
 */

#ifndef ZMK_DRIVER_AZOTEQ_IQS5XX_SCROLL_SPEED_H_
#define ZMK_DRIVER_AZOTEQ_IQS5XX_SCROLL_SPEED_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Adjust the scroll speed of the current host (USB or active BLE profile) in percent.
void zmk_scroll_speed_adjust(int delta_pct);

// Restore the scroll speed of the current host to 100%.
void zmk_scroll_speed_reset_current(void);

// Format a single-line summary of the per-host scroll speeds.
int zmk_scroll_speed_format_status(char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif // ZMK_DRIVER_AZOTEQ_IQS5XX_SCROLL_SPEED_H_
