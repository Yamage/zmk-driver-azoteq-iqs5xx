/*
 * SPDX-License-Identifier: MIT
 *
 * Crash guard: records fatal errors and work queue hangs in RAM that survives a
 * soft reset, reboots automatically instead of halting, and persists the last
 * record to settings so it can be reported later (e.g. typed by a diagnostic key).
 */

#ifndef ZMK_DRIVER_AZOTEQ_IQS5XX_CRASH_GUARD_H_
#define ZMK_DRIVER_AZOTEQ_IQS5XX_CRASH_GUARD_H_

#include <stddef.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Watch a work queue with the task watchdog: a feed item is scheduled on the
 * queue periodically, and if the queue does not run it within
 * CONFIG_ZMK_CRASH_GUARD_WATCHDOG_TIMEOUT_MS the hang is recorded and the
 * system reboots. Call from init code; `label` must be a static string.
 * Watches registered before the crash guard starts are armed when it starts.
 */
int zmk_crash_guard_watch_work_q(struct k_work_q *queue, const char *label);

/*
 * Write a human-readable summary of the last boot cause and the last recorded
 * crash/hang (one or more '\n'-terminated lines). Returns the snprintf-style
 * length that the full text needs.
 */
int zmk_crash_guard_format(char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif // ZMK_DRIVER_AZOTEQ_IQS5XX_CRASH_GUARD_H_
