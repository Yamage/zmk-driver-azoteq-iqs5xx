/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 *
 * Runtime control API for the IQS5xx trackpad inertia/glide feature. Intended to
 * be called from a ZMK behavior so the user can toggle and tune inertia from the
 * keymap. All changes are clamped and (when CONFIG_SETTINGS is enabled) persisted
 * to flash so they survive a reboot. Safe to call before the trackpad is ready
 * (the calls become no-ops).
 */

#ifndef ZMK_DRIVER_AZOTEQ_IQS5XX_INERTIA_H_
#define ZMK_DRIVER_AZOTEQ_IQS5XX_INERTIA_H_

#ifdef __cplusplus
extern "C" {
#endif

// Toggle the inertia glide on/off (master enable; honours the per-type
// inertia-cursor / inertia-scroll devicetree flags when on).
void iqs5xx_inertia_toggle(void);

// Adjust the friction (velocity retained per tick, out of 256). Positive delta =
// longer glide. Clamped to a sensible range.
void iqs5xx_inertia_adjust_friction(int delta);

// Adjust the minimum release speed (counts/report) needed to start a glide.
// Positive delta = needs a stronger flick. Clamped to a sensible range.
void iqs5xx_inertia_adjust_min_speed(int delta);

// Restore enabled state, friction and min-speed to the devicetree defaults.
void iqs5xx_inertia_reset(void);

// Stop an ongoing glide immediately (no-op when not gliding).
void iqs5xx_inertia_stop(void);

// Adjust cursor rotation in degrees (positive = clockwise on screen).
void iqs5xx_adjust_rotation(int delta_deg);

// Adjust base cursor speed scale in percent (100 = 1.0x).
void iqs5xx_adjust_cursor_scale(int delta_pct);

// Adjust horizontal (X-axis) cursor scale in percent relative to Y (100 = 1:1, 175 = 1.75x).
void iqs5xx_adjust_x_scale(int delta_pct);

// Format a single-line summary of the current trackpad runtime settings.
int iqs5xx_format_status(char *buf, size_t size);

// True when ZMK's active output endpoint is BLE (implemented in behavior_inertia.c).
bool iqs5xx_is_ble_transport(void);

#ifdef __cplusplus
}
#endif

#endif // ZMK_DRIVER_AZOTEQ_IQS5XX_INERTIA_H_
