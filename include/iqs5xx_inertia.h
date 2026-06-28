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

#ifdef __cplusplus
}
#endif

#endif // ZMK_DRIVER_AZOTEQ_IQS5XX_INERTIA_H_
