/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include "iqs5xx.h"
#include <iqs5xx_inertia.h>

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_WATCHDOG)
#include <zmk_crash_guard.h>
#endif

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

// Bounds for the runtime-tunable inertia and cursor parameters.
#define IQS5XX_FRICTION_MIN 160
#define IQS5XX_FRICTION_MAX 252
#define IQS5XX_MIN_SPEED_MIN 0
#define IQS5XX_MIN_SPEED_MAX 60
#define IQS5XX_ROTATION_DEG_MIN (-90)
#define IQS5XX_ROTATION_DEG_MAX 90
#define IQS5XX_CURSOR_SCALE_PCT_MIN 40
#define IQS5XX_CURSOR_SCALE_PCT_MAX 250
#define IQS5XX_CURSOR_SCALE_PCT_DEFAULT 100
#define IQS5XX_X_SCALE_PCT_MIN 50
#define IQS5XX_X_SCALE_PCT_MAX 300

// Single trackpad instance, captured at init so the keymap behavior and the
// settings handler can reach the runtime inertia state.
static struct iqs5xx_data *iqs5xx_inertia_data;
static const struct iqs5xx_config *iqs5xx_inertia_config;

/*
 * Dedicated workqueue for servicing trackpad reports. Keeping report processing
 * off the shared system workqueue (which also handles BLE, split comms, etc.)
 * avoids head-of-line blocking and reduces the report-to-report jitter that
 * makes the cursor look low-framerate. Shared across all instances (there is
 * only ever one trackpad).
 */
K_THREAD_STACK_DEFINE(iqs5xx_work_q_stack, CONFIG_INPUT_AZOTEQ_IQS5XX_WORKQUEUE_STACK_SIZE);
static struct k_work_q iqs5xx_work_q;
static bool iqs5xx_work_q_started;

// Default weak stub when ZMK_BEHAVIOR_INERTIA is not compiled in.
__weak bool iqs5xx_is_ble_transport(void) { return false; }

static void iqs5xx_emit_cursor_now(struct iqs5xx_data *data) {
    if (data->ble_pending_x == 0 && data->ble_pending_y == 0) {
        return;
    }
    int16_t out_x = CLAMP(data->ble_pending_x, INT16_MIN, INT16_MAX);
    int16_t out_y = CLAMP(data->ble_pending_y, INT16_MIN, INT16_MAX);
    data->ble_pending_x = 0;
    data->ble_pending_y = 0;
    data->last_ble_emit_ms = k_uptime_get();
    input_report_rel(data->dev, INPUT_REL_X, out_x, false, K_FOREVER);
    input_report_rel(data->dev, INPUT_REL_Y, out_y, true, K_FOREVER);
}

static void iqs5xx_ble_flush_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, ble_flush_work);
    iqs5xx_emit_cursor_now(data);
}

static void iqs5xx_report_cursor(struct iqs5xx_data *data, int32_t dx, int32_t dy) {
    if (dx == 0 && dy == 0) {
        return;
    }
    const struct iqs5xx_config *config = data->dev->config;
    data->ble_pending_x += dx;
    data->ble_pending_y += dy;

    if (config->ble_min_report_interval_ms == 0 || !iqs5xx_is_ble_transport()) {
        k_work_cancel_delayable(&data->ble_flush_work);
        iqs5xx_emit_cursor_now(data);
        return;
    }

    int64_t elapsed = k_uptime_get() - data->last_ble_emit_ms;
    int64_t min_ms = config->ble_min_report_interval_ms;
    if (elapsed >= min_ms) {
        k_work_cancel_delayable(&data->ble_flush_work);
        iqs5xx_emit_cursor_now(data);
    } else {
        k_work_schedule_for_queue(&iqs5xx_work_q, &data->ble_flush_work,
                                  K_MSEC(min_ms - elapsed));
    }
}

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    // TODO: This loop should only deactivate one button.
    // Log a warning when that is not the case.
    for (int i = 0; i < 3; i++) {
        LOG_INF("Releasing synthetic button");
        if (data->buttons_pressed & BIT(i)) {
            input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
            // Turn off the bit.
            data->buttons_pressed &= ~BIT(i);
        }
    }
}

/*
 * Inertia / glide handler.
 *
 * After the finger is lifted, this replays the last measured velocity as a
 * stream of synthetic relative reports, decaying it by a friction factor every
 * tick until it dies out (trackball-like spin). The RDY interrupt does not fire
 * once the finger is gone, so this self-reschedules on the trackpad workqueue.
 * A fresh touch cancels it from iqs5xx_work_handler() for an instant stop.
 */
static void iqs5xx_inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, inertia_work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    // vel is per chip report; scale it to per glide tick so the glide starts at the release speed.
    int32_t report_ms =
        config->active_report_rate ? config->active_report_rate : IQS5XX_DEFAULT_REPORT_MS;
    int32_t step_x = ((data->vel_x * data->glide_gain_q8) >> 8) * config->inertia_tick_ms / report_ms;
    int32_t step_y = ((data->vel_y * data->glide_gain_q8) >> 8) * config->inertia_tick_ms / report_ms;

    if (data->glide_mode == IQS5XX_GLIDE_CURSOR) {
        int32_t step_x_scaled = step_x * (int32_t)data->x_scale_pct_rt / 100;
        data->glide_x_acc += step_x_scaled;
        data->glide_y_acc += step_y;
        int32_t out_x = data->glide_x_acc >> 8;
        int32_t out_y = data->glide_y_acc >> 8;
        data->glide_x_acc -= out_x << 8;
        data->glide_y_acc -= out_y << 8;
        iqs5xx_report_cursor(data, out_x, out_y);
    } else if (data->glide_mode == IQS5XX_GLIDE_SCROLL) {
        const int16_t scroll_div = 32;
        data->glide_x_acc += step_x;
        data->glide_y_acc += step_y;
        int32_t scroll_step_x = data->glide_x_acc >> 8;
        int32_t scroll_step_y = data->glide_y_acc >> 8;
        data->glide_x_acc -= scroll_step_x << 8;
        data->glide_y_acc -= scroll_step_y << 8;
        if (scroll_step_x != 0) {
            data->scroll_x_acc += scroll_step_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
        }
        if (scroll_step_y != 0) {
            data->scroll_y_acc += scroll_step_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_y_acc %= scroll_div;
            }
        }
    } else {
        return;
    }

    // Decay the velocity by the friction factor (retained/256 per tick).
    data->vel_x = (data->vel_x * (int32_t)data->inertia_friction_rt) >> 8;
    data->vel_y = (data->vel_y * (int32_t)data->inertia_friction_rt) >> 8;

    // Stop once the speed falls below ~1 count/tick on both axes.
    if (abs(data->vel_x) < (1 << 8) && abs(data->vel_y) < (1 << 8)) {
        data->glide_mode = IQS5XX_GLIDE_NONE;
        data->vel_x = 0;
        data->vel_y = 0;
        data->glide_x_acc = 0;
        data->glide_y_acc = 0;
        return;
    }

    k_work_schedule_for_queue(&iqs5xx_work_q, &data->inertia_work,
                              K_MSEC(config->inertia_tick_ms));
}

static void iqs5xx_glide_clear(struct iqs5xx_data *data) {
    k_work_cancel_delayable(&data->inertia_work);
    k_work_cancel_delayable(&data->ble_flush_work);
    iqs5xx_emit_cursor_now(data);
    data->glide_mode = IQS5XX_GLIDE_NONE;
    data->vel_x = 0;
    data->vel_y = 0;
    data->prev_vel_x = 0;
    data->prev_vel_y = 0;
    data->glide_x_acc = 0;
    data->glide_y_acc = 0;
}

static void iqs5xx_inertia_stop_work_handler(struct k_work *work) {
    iqs5xx_glide_clear(CONTAINER_OF(work, struct iqs5xx_data, inertia_stop_work));
}

// Approximate speed (Q8 counts/report) from a velocity vector: max + min/2.
static int32_t iqs5xx_vec_speed_q8(int32_t vx, int32_t vy) {
    int32_t ax = abs(vx);
    int32_t ay = abs(vy);
    return MAX(ax, ay) + MIN(ax, ay) / 2;
}

static int32_t iqs5xx_speed_q8(const struct iqs5xx_data *data) {
    return iqs5xx_vec_speed_q8(data->vel_x, data->vel_y);
}

static int32_t iqs5xx_accel_gain_q8(const struct iqs5xx_data *data,
                                    const struct iqs5xx_config *config, int32_t speed_q8) {
    int32_t lo = (int32_t)config->accel_speed_low << 8;
    int32_t hi = (int32_t)config->accel_speed_high << 8;
    int32_t gmin = (int32_t)config->accel_gain_min * 256 / 100;
    int32_t gmax = (int32_t)config->accel_gain_max * 256 / 100;
    int32_t gain;
    if (speed_q8 >= hi) {
        gain = gmax;
    } else if (speed_q8 <= lo) {
        gain = gmin;
    } else {
        gain = gmin + (gmax - gmin) * ((speed_q8 - lo) >> 4) / ((hi - lo) >> 4);
    }
    return gain * (int32_t)data->cursor_scale_pct_rt / 100;
}

static void iqs5xx_update_rotation_trig(struct iqs5xx_data *data) {
    if (data->rotation_deg_rt != 0) {
        float rad = (float)data->rotation_deg_rt * (3.14159265f / 180.0f);
        data->rot_cos_q14 = (int32_t)roundf(cosf(rad) * 16384.0f);
        data->rot_sin_q14 = (int32_t)roundf(sinf(rad) * 16384.0f);
    } else {
        data->rot_cos_q14 = 16384;
        data->rot_sin_q14 = 0;
    }
    data->rot_x_acc = 0;
    data->rot_y_acc = 0;
}

/* ---- Runtime inertia & cursor control + flash persistence ---- */

#if IS_ENABLED(CONFIG_SETTINGS)
struct iqs5xx_inertia_persist_v1 {
    uint8_t enabled;
    uint16_t friction;
    uint16_t min_speed;
};

struct iqs5xx_inertia_persist_v2 {
    uint8_t enabled;
    uint16_t friction;
    uint16_t min_speed;
    int16_t rotation_deg;
    uint16_t cursor_scale_pct;
};

struct iqs5xx_inertia_persist {
    uint8_t enabled;
    uint16_t friction;
    uint16_t min_speed;
    int16_t rotation_deg;
    uint16_t cursor_scale_pct;
    uint16_t x_scale_pct;
};

static void iqs5xx_inertia_save_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    struct iqs5xx_inertia_persist p = {
        .enabled = data->inertia_enabled ? 1 : 0,
        .friction = data->inertia_friction_rt,
        .min_speed = data->inertia_min_speed_rt,
        .rotation_deg = data->rotation_deg_rt,
        .cursor_scale_pct = data->cursor_scale_pct_rt,
        .x_scale_pct = data->x_scale_pct_rt,
    };
    int rc = settings_save_one("iqs5xx_inertia/state", &p, sizeof(p));
    if (rc) {
        LOG_WRN("Failed to save inertia settings: %d", rc);
    }
}

static int iqs5xx_inertia_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                       void *cb_arg) {
    if (!settings_name_steq(name, "state", NULL)) {
        return -ENOENT;
    }
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return -EINVAL;
    }
    if (len == sizeof(struct iqs5xx_inertia_persist)) {
        struct iqs5xx_inertia_persist p;
        int rc = read_cb(cb_arg, &p, sizeof(p));
        if (rc < 0) {
            return rc;
        }
        data->inertia_enabled = p.enabled != 0;
        data->inertia_friction_rt = CLAMP(p.friction, IQS5XX_FRICTION_MIN, IQS5XX_FRICTION_MAX);
        data->inertia_min_speed_rt = CLAMP(p.min_speed, IQS5XX_MIN_SPEED_MIN, IQS5XX_MIN_SPEED_MAX);
        data->rotation_deg_rt =
            CLAMP(p.rotation_deg, IQS5XX_ROTATION_DEG_MIN, IQS5XX_ROTATION_DEG_MAX);
        data->cursor_scale_pct_rt =
            CLAMP(p.cursor_scale_pct, IQS5XX_CURSOR_SCALE_PCT_MIN, IQS5XX_CURSOR_SCALE_PCT_MAX);
        data->x_scale_pct_rt =
            CLAMP(p.x_scale_pct, IQS5XX_X_SCALE_PCT_MIN, IQS5XX_X_SCALE_PCT_MAX);
        iqs5xx_update_rotation_trig(data);
        return 0;
    }
    if (len == sizeof(struct iqs5xx_inertia_persist_v2)) {
        struct iqs5xx_inertia_persist_v2 p2;
        int rc = read_cb(cb_arg, &p2, sizeof(p2));
        if (rc < 0) {
            return rc;
        }
        data->inertia_enabled = p2.enabled != 0;
        data->inertia_friction_rt = CLAMP(p2.friction, IQS5XX_FRICTION_MIN, IQS5XX_FRICTION_MAX);
        data->inertia_min_speed_rt =
            CLAMP(p2.min_speed, IQS5XX_MIN_SPEED_MIN, IQS5XX_MIN_SPEED_MAX);
        data->rotation_deg_rt =
            CLAMP(p2.rotation_deg, IQS5XX_ROTATION_DEG_MIN, IQS5XX_ROTATION_DEG_MAX);
        data->cursor_scale_pct_rt =
            CLAMP(p2.cursor_scale_pct, IQS5XX_CURSOR_SCALE_PCT_MIN, IQS5XX_CURSOR_SCALE_PCT_MAX);
        iqs5xx_update_rotation_trig(data);
        return 0;
    }
    if (len == sizeof(struct iqs5xx_inertia_persist_v1)) {
        struct iqs5xx_inertia_persist_v1 p1;
        int rc = read_cb(cb_arg, &p1, sizeof(p1));
        if (rc < 0) {
            return rc;
        }
        data->inertia_enabled = p1.enabled != 0;
        data->inertia_friction_rt = CLAMP(p1.friction, IQS5XX_FRICTION_MIN, IQS5XX_FRICTION_MAX);
        data->inertia_min_speed_rt =
            CLAMP(p1.min_speed, IQS5XX_MIN_SPEED_MIN, IQS5XX_MIN_SPEED_MAX);
        return 0;
    }
    return -EINVAL;
}

SETTINGS_STATIC_HANDLER_DEFINE(iqs5xx_inertia, "iqs5xx_inertia", NULL,
                               iqs5xx_inertia_settings_set, NULL, NULL);

static int iqs5xx_inertia_settings_load(void) {
    return settings_load_subtree("iqs5xx_inertia");
}
SYS_INIT(iqs5xx_inertia_settings_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif // CONFIG_SETTINGS

static void iqs5xx_inertia_schedule_save(struct iqs5xx_data *data) {
#if IS_ENABLED(CONFIG_SETTINGS)
    // Run the flash write on the system workqueue, NOT the dedicated trackpad
    // report queue (iqs5xx_work_q). settings_save_one() blocks while the nRF
    // flash write is synchronised with the BLE radio and is stack-heavy; doing
    // it on the single-threaded report queue stalls (or overflows) report
    // processing and freezes the cursor a couple of seconds after an inertia
    // control is used.
    k_work_schedule(&data->inertia_save_work, K_SECONDS(2));
#else
    ARG_UNUSED(data);
#endif
}

void iqs5xx_inertia_toggle(void) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    data->inertia_enabled = !data->inertia_enabled;
    LOG_INF("Inertia %s", data->inertia_enabled ? "on" : "off");
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_inertia_adjust_friction(int delta) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    int v = CLAMP((int)data->inertia_friction_rt + delta, IQS5XX_FRICTION_MIN, IQS5XX_FRICTION_MAX);
    data->inertia_friction_rt = (uint16_t)v;
    LOG_INF("Inertia friction %d", v);
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_inertia_adjust_min_speed(int delta) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    int v =
        CLAMP((int)data->inertia_min_speed_rt + delta, IQS5XX_MIN_SPEED_MIN, IQS5XX_MIN_SPEED_MAX);
    data->inertia_min_speed_rt = (uint16_t)v;
    LOG_INF("Inertia min-speed %d", v);
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_inertia_stop(void) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    k_work_submit_to_queue(&iqs5xx_work_q, &data->inertia_stop_work);
}

void iqs5xx_inertia_reset(void) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    const struct iqs5xx_config *config = iqs5xx_inertia_config;
    if (data == NULL || config == NULL) {
        return;
    }
    data->inertia_enabled = config->inertia_cursor || config->inertia_scroll;
    data->inertia_friction_rt = config->inertia_friction;
    data->inertia_min_speed_rt = config->inertia_min_speed;
    data->rotation_deg_rt = config->rotation_deg;
    data->cursor_scale_pct_rt = IQS5XX_CURSOR_SCALE_PCT_DEFAULT;
    data->x_scale_pct_rt = CLAMP(config->x_scale_pct, IQS5XX_X_SCALE_PCT_MIN, IQS5XX_X_SCALE_PCT_MAX);
    iqs5xx_update_rotation_trig(data);
    LOG_INF("Trackpad runtime settings reset to defaults");
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_adjust_rotation(int delta_deg) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    int v =
        CLAMP((int)data->rotation_deg_rt + delta_deg, IQS5XX_ROTATION_DEG_MIN, IQS5XX_ROTATION_DEG_MAX);
    data->rotation_deg_rt = (int16_t)v;
    iqs5xx_update_rotation_trig(data);
    LOG_INF("Trackpad rotation %d deg", v);
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_adjust_cursor_scale(int delta_pct) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    int v = CLAMP((int)data->cursor_scale_pct_rt + delta_pct, IQS5XX_CURSOR_SCALE_PCT_MIN,
                  IQS5XX_CURSOR_SCALE_PCT_MAX);
    data->cursor_scale_pct_rt = (uint16_t)v;
    LOG_INF("Trackpad cursor scale %d%%", v);
    iqs5xx_inertia_schedule_save(data);
}

void iqs5xx_adjust_x_scale(int delta_pct) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return;
    }
    int v = CLAMP((int)data->x_scale_pct_rt + delta_pct, IQS5XX_X_SCALE_PCT_MIN,
                  IQS5XX_X_SCALE_PCT_MAX);
    data->x_scale_pct_rt = (uint16_t)v;
    LOG_INF("Trackpad X scale %d%%", v);
    iqs5xx_inertia_schedule_save(data);
}

int iqs5xx_format_status(char *buf, size_t size) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    if (data == NULL) {
        return 0;
    }
    return snprintf(buf, size,
                    "[Trackpad] Rot:%ddeg | Spd:%u%% | X:%u%% | Inertia:%s (Fric:%u MSpd:%u)\n",
                    (int)data->rotation_deg_rt, (unsigned)data->cursor_scale_pct_rt,
                    (unsigned)data->x_scale_pct_rt, data->inertia_enabled ? "ON" : "OFF",
                    (unsigned)data->inertia_friction_rt, (unsigned)data->inertia_min_speed_rt);
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    int ret;

    /*
     * Read the whole contiguous data block (0x000D..0x0015) in a single I2C
     * transaction instead of one read per register. Fewer transactions keep
     * the per-report bus time well inside the RDY comm window, which avoids
     * dropped reports (RR_MISSED) and makes the cursor refresh smoothly.
     *
     * Block layout (offset from 0x000D = IQS5XX_GESTURE_EVENTS_0):
     *   [0] gesture events 0   [1] gesture events 1
     *   [2] system info 0      [3] system info 1
     *   [4] number of fingers
     *   [5..6] relative X (big-endian, int16)
     *   [7..8] relative Y (big-endian, int16)
     */
    uint8_t block[9];
    uint8_t block_reg[2] = {IQS5XX_GESTURE_EVENTS_0 >> 8, IQS5XX_GESTURE_EVENTS_0 & 0xFF};
    ret = i2c_write_read_dt(&config->i2c, block_reg, sizeof(block_reg), block, sizeof(block));
    if (ret < 0) {
        LOG_ERR("Failed to read data block: %d", ret);
        goto end_comm;
    }

    uint8_t gesture_events_0 = block[0];
    uint8_t gesture_events_1 = block[1];
    uint8_t sys_info_0 = block[2];
    uint8_t sys_info_1 = block[3];
    int16_t rel_x = (int16_t)((block[5] << 8) | block[6]);
    int16_t rel_y = (int16_t)((block[7] << 8) | block[8]);

    // Handle reset indication.
    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        LOG_INF("Device reset detected");
        // Acknowledge reset.
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        goto end_comm;
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;
    bool scroll = (gesture_events_1 & IQS5XX_SCROLL) != 0;
    if (!scroll) {
        // Clear accumulators if we're not actively scrolling.
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    // Touch/release edge detection for the inertia glide.
    uint8_t fingers = block[4];
    if (fingers > 0 && data->last_fingers == 0) {
        // A new touch instantly stops any ongoing glide.
        iqs5xx_glide_clear(data);
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
        data->accel_x_acc = 0;
        data->accel_y_acc = 0;
        data->rot_x_acc = 0;
        data->rot_y_acc = 0;
        data->touch_reports = 1;
        data->move_reports = 0;
        data->idle_reports = 0;

        data->drag_ignore_tap = false;
        if (data->tap_drag_armed &&
            k_uptime_get() - data->last_tap_ms <= IQS5XX_TAP_DRAG_WINDOW_MS) {
            // The first tap's click may still be held for its 100 ms; end it so the drag is a new press.
            if (data->buttons_pressed & LEFT_BUTTON_BIT) {
                input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
                data->buttons_pressed &= ~LEFT_BUTTON_BIT;
            }
            input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
            data->drag_active = true;
        }
        data->tap_drag_armed = false;
    } else if (fingers > 0) {
        if (data->touch_reports < UINT8_MAX) {
            data->touch_reports++;
        }
    } else if (fingers == 0 && data->last_fingers > 0 && data->drag_active) {
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->drag_active = false;
        data->drag_ignore_tap = true;
        // Do not fling the cursor when a drag ends.
        data->glide_mode = IQS5XX_GLIDE_NONE;
        data->vel_x = 0;
        data->vel_y = 0;
        data->prev_vel_x = 0;
        data->prev_vel_y = 0;
    } else if (fingers == 0 && data->last_fingers > 0) {
        /*
         * If the finger was already leaving the pad on the final movement report,
         * that last delta is often smaller than the peak flick delta 1 report earlier.
         * When the thumb did not pause before lifting (idle_reports <= 1), use the
         * faster of the last two velocity samples so a quick flick is not lost.
         */
        if (data->idle_reports <= 1 &&
            iqs5xx_vec_speed_q8(data->prev_vel_x, data->prev_vel_y) > iqs5xx_speed_q8(data)) {
            data->vel_x = data->prev_vel_x;
            data->vel_y = data->prev_vel_y;
        }
        /*
         * Short-flick rescue: if the thumb brushed across the pad so quickly that
         * the chip only detected movement on the lift-off frame itself (fingers == 0
         * with tp_movement && touch_reports <= 6), seed velocity from this frame's
         * delta instead of dropping it.
         */
        if (data->move_reports == 0 && tp_movement && data->touch_reports <= 6 &&
            (rel_x != 0 || rel_y != 0) && config->inertia_cursor) {
            int32_t rx = rel_x;
            int32_t ry = rel_y;
            if (data->rotation_deg_rt != 0) {
                rx = (rel_x * data->rot_cos_q14 - rel_y * data->rot_sin_q14) / (1 << 14);
                ry = (rel_x * data->rot_sin_q14 + rel_y * data->rot_cos_q14) / (1 << 14);
            }
            data->vel_x = rx << 8;
            data->vel_y = ry << 8;
            data->glide_mode = IQS5XX_GLIDE_CURSOR;
            int32_t gain_q8 = iqs5xx_accel_gain_q8(data, config, iqs5xx_speed_q8(data));
            int32_t gain_x_q8 = gain_q8 * (int32_t)data->x_scale_pct_rt / 100;
            iqs5xx_report_cursor(data, (rx * gain_x_q8) / 256, (ry * gain_q8) / 256);
        }

        // The finger was lifted: start gliding if the release was fast enough.
        bool enabled = data->inertia_enabled &&
                       ((data->glide_mode == IQS5XX_GLIDE_CURSOR && config->inertia_cursor) ||
                        (data->glide_mode == IQS5XX_GLIDE_SCROLL && config->inertia_scroll));
        int32_t min_q8 = (int32_t)data->inertia_min_speed_rt << 8;
        if (enabled && iqs5xx_speed_q8(data) >= min_q8) {
            data->glide_x_acc = 0;
            data->glide_y_acc = 0;
            data->glide_gain_q8 =
                data->glide_mode == IQS5XX_GLIDE_CURSOR
                    ? iqs5xx_accel_gain_q8(data, config, iqs5xx_speed_q8(data))
                    : 256;
            k_work_schedule_for_queue(&iqs5xx_work_q, &data->inertia_work,
                                      K_MSEC(config->inertia_tick_ms));
        } else {
            data->glide_mode = IQS5XX_GLIDE_NONE;
            data->vel_x = 0;
            data->vel_y = 0;
        }
        data->prev_vel_x = 0;
        data->prev_vel_y = 0;
    }
    data->last_fingers = fingers;

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = !data->drag_ignore_tap;
        button_code = INPUT_BTN_0;
    } else if (gesture_events_1 & IQS5XX_TWO_FINGER_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_1;
    }

    bool hold_became_active = (gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released = !(gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && data->active_hold;

    // Handle movement and gestures.
    //
    // Each one of these branches needs to send the last report it makes as
    // sync to ensure that the input subsystem processes things in order.
    if (hold_became_active) {
        LOG_INF("Hold became active");
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
        data->active_hold = true;
        // Suppress inertia during a press-and-hold drag.
        data->glide_mode = IQS5XX_GLIDE_NONE;
        data->vel_x = 0;
        data->vel_y = 0;
    } else if (hold_released) {
        LOG_INF("Hold became inactive");
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->active_hold = false;
        // Do not fling the cursor when a drag ends.
        data->glide_mode = IQS5XX_GLIDE_NONE;
        data->vel_x = 0;
        data->vel_y = 0;
    } else if (button_pressed) {
        // Cancel any pending release.
        k_work_cancel_delayable(&data->button_release_work);

        // ZMK counts presses per button; a second press without a release leaves it stuck down.
        uint8_t button_bit = BIT(button_code - INPUT_BTN_0);
        if (data->buttons_pressed & button_bit) {
            input_report_key(dev, button_code, 0, true, K_FOREVER);
        }

        // Press the button immediately.
        input_report_key(dev, button_code, 1, true, K_FOREVER);
        data->buttons_pressed |= button_bit;

        // Same queue as this handler, so the release can never run concurrently with a new press.
        k_work_schedule_for_queue(&iqs5xx_work_q, &data->button_release_work, K_MSEC(100));

        if (button_code == INPUT_BTN_0 && config->tap_and_drag) {
            data->tap_drag_armed = true;
            data->last_tap_ms = k_uptime_get();
        }
    } else if (scroll) {
        // TODO: Expose this divisor.
        int16_t scroll_div = 32;

        // Only one scrolling direction is valid at a time.
        // End the communication right after reporting the movement.
        if (rel_x != 0) {
            // By default the x axis is already "natural".
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
            // Track velocity for scroll inertia (sign already normalised above).
            data->vel_x = (data->vel_x * 3 + ((int32_t)rel_x << 8)) >> 2;
            data->vel_y = (data->vel_y * 3) >> 2;
            data->glide_mode = IQS5XX_GLIDE_SCROLL;
            goto end_comm;
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_y_acc %= scroll_div;
            }

            data->vel_y = (data->vel_y * 3 + ((int32_t)rel_y << 8)) >> 2;
            data->vel_x = (data->vel_x * 3) >> 2;
            data->glide_mode = IQS5XX_GLIDE_SCROLL;
            goto end_comm;
        }
    } else if (tp_movement && fingers > 0) {
        // Skip the lift report (fingers == 0): a thumb rolling off can move the centroid there.
        if (data->rotation_deg_rt != 0) {
            data->rot_x_acc += rel_x * data->rot_cos_q14 - rel_y * data->rot_sin_q14;
            data->rot_y_acc += rel_x * data->rot_sin_q14 + rel_y * data->rot_cos_q14;
            rel_x = data->rot_x_acc / (1 << 14);
            rel_y = data->rot_y_acc / (1 << 14);
            data->rot_x_acc -= rel_x * (1 << 14);
            data->rot_y_acc -= rel_y * (1 << 14);
        }
        /*
         * Velocity (raw counts) drives both the acceleration gain and the glide.
         * On the first 1-2 movement reports of a fresh touch, seed velocity directly
         * from the instantaneous delta so a quick, trackball-like flick reaches full
         * speed immediately instead of losing 56%-75% of its speed to the 4-frame EMA
         * warm-up. From the 3rd report onward, smooth with a 50/50 EMA.
         */
        data->prev_vel_x = data->vel_x;
        data->prev_vel_y = data->vel_y;
        if (data->move_reports < 2) {
            data->vel_x = (int32_t)rel_x << 8;
            data->vel_y = (int32_t)rel_y << 8;
        } else {
            data->vel_x = (data->vel_x + ((int32_t)rel_x << 8)) >> 1;
            data->vel_y = (data->vel_y + ((int32_t)rel_y << 8)) >> 1;
        }
        if (data->move_reports < UINT8_MAX) {
            data->move_reports++;
        }
        data->idle_reports = 0;
        data->glide_mode = IQS5XX_GLIDE_CURSOR;

        int32_t gain_q8 = iqs5xx_accel_gain_q8(data, config, iqs5xx_speed_q8(data));
        int32_t gain_x_q8 = gain_q8 * (int32_t)data->x_scale_pct_rt / 100;
        data->accel_x_acc += rel_x * gain_x_q8;
        data->accel_y_acc += rel_y * gain_q8;
        int32_t out_x = data->accel_x_acc / 256;
        int32_t out_y = data->accel_y_acc / 256;
        data->accel_x_acc -= out_x * 256;
        data->accel_y_acc -= out_y * 256;
        iqs5xx_report_cursor(data, out_x, out_y);
    } else if (!tp_movement && fingers > 0) {
        /*
         * Finger is held stationary on the pad:
         * - Wait 1 report (5 ms) before decaying so a single zero-delta report right
         *   as the thumb lifts off after a quick flick does not halve the release speed.
         * - Once stationary for >= 2 reports (>= 10 ms), decay velocity rapidly so
         *   stopping the thumb before lifting never triggers an unintended glide.
         */
        if (data->idle_reports < UINT8_MAX) {
            data->idle_reports++;
        }
        if (data->idle_reports >= 2) {
            data->vel_x >>= 1;
            data->vel_y >>= 1;
            data->prev_vel_x = data->vel_x;
            data->prev_vel_y = data->vel_y;
            if (abs(data->vel_x) < (1 << 8)) {
                data->vel_x = 0;
                data->prev_vel_x = 0;
            }
            if (abs(data->vel_y) < (1 << 8)) {
                data->vel_y = 0;
                data->prev_vel_y = 0;
            }
        }
    }

end_comm:
    // End communication window.
    iqs5xx_end_comm_window(dev);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit_to_queue(&iqs5xx_work_q, &data->work);
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;

    // Enable event mode and trackpad events.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1,
                            IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    /*
     * Configure report rates (0x057A..0x0583) and mode timeouts (0x0584..0x0587)
     * in a single 14-byte contiguous I2C burst write so the bus time stays minimal:
     * - Active rate: config->active_report_rate (default 8 ms = 125 Hz)
     * - Idle Touch rate: 25 ms (40 Hz) while finger rests without moving
     * - Idle rate: 30 ms (33 Hz) just after release for instant wake
     * - LP1 rate: 50 ms (20 Hz), LP2 rate: 100 ms (10 Hz) for deep idle
     * - Timeouts (in 500 ms units): Active->Idle after 1 s (2),
     *   IdleTouch->Idle after 2 s (4), Idle->LP1 after 2 s (4), LP1->LP2 after 10 s (20).
     */
    uint16_t act_rate =
        config->active_report_rate ? config->active_report_rate : IQS5XX_DEFAULT_REPORT_MS;
    uint8_t power_Profile[16] = {
        IQS5XX_ACTIVE_REPORT_RATE >> 8,
        IQS5XX_ACTIVE_REPORT_RATE & 0xFF,
        (uint8_t)(act_rate >> 8),
        (uint8_t)(act_rate & 0xFF),
        0, 25,  // IDLE_TOUCH_REPORT_RATE = 25 ms
        0, 30,  // IDLE_REPORT_RATE = 30 ms
        0, 50,  // LP1_REPORT_RATE = 50 ms
        0, 100, // LP2_REPORT_RATE = 100 ms
        2,      // ACTIVE_TIMEOUT = 1 s
        4,      // IDLE_TOUCH_TIMEOUT = 2 s
        4,      // IDLE_TIMEOUT = 2 s
        20,     // LP1_TIMEOUT = 10 s
    };
    ret = i2c_write_dt(&config->i2c, power_Profile, sizeof(power_Profile));
    if (ret < 0) {
        LOG_ERR("Failed to set report rates and power timeouts: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_BOTTOM_BETA, config->bottom_beta);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom beta: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_STATIONARY_THRESH, config->stationary_threshold);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom stationary threshold: %d", ret);
        return ret;
    }

    // Optionally raise the trackpad resolution for finer, smoother tracking.
    if (config->x_resolution != 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_X_RESOLUTION, config->x_resolution);
        if (ret < 0) {
            LOG_ERR("Failed to set X resolution: %d", ret);
            return ret;
        }
    }

    if (config->y_resolution != 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_Y_RESOLUTION, config->y_resolution);
        if (ret < 0) {
            LOG_ERR("Failed to set Y resolution: %d", ret);
            return ret;
        }
    }

    // TODO: Expose these through dts bindings.
    // Set filter settings with:
    // - IIR filter enabled
    // - MAV filter enabled
    // - IIR select disabled (dynamic IIR)
    // - ALP count filter enabled
    ret = iqs5xx_write_reg8(dev, IQS5XX_FILTER_SETTINGS,
                            IQS5XX_IIR_FILTER | IQS5XX_MAV_FILTER | IQS5XX_ALP_COUNT_FILTER);
    if (ret < 0) {
        LOG_ERR("Failed to configure filter settings: %d", ret);
        return ret;
    }

    uint8_t single_finger_gestures = 0;
    single_finger_gestures |= config->one_finger_tap ? IQS5XX_SINGLE_TAP : 0;
    single_finger_gestures |= config->press_and_hold ? IQS5XX_PRESS_AND_HOLD : 0;
    // Configure single finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, single_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure single finger gestures: %d", ret);
        return ret;
    }

    // Configure the hold time for the press and hold gesture.
    ret = iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, config->press_and_hold_time);
    if (ret < 0) {
        LOG_ERR("Failed to configure the hold time: %d", ret);
        return ret;
    }

    if (config->tap_distance != 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_TAP_DISTANCE, config->tap_distance);
        if (ret < 0) {
            LOG_ERR("Failed to configure the tap distance: %d", ret);
            return ret;
        }
    }

    if (config->touch_threshold_increase != 0) {
        struct iqs5xx_data *data = dev->data;
        // Read the module's own set/clear multipliers once, so a setup retry does not add twice.
        if (!data->touch_mult_read) {
            uint8_t reg[2] = {IQS5XX_GLOBAL_TOUCH_SET >> 8, IQS5XX_GLOBAL_TOUCH_SET & 0xFF};
            ret = i2c_write_read_dt(&config->i2c, reg, sizeof(reg), data->touch_mult_base,
                                    sizeof(data->touch_mult_base));
            if (ret < 0) {
                LOG_ERR("Failed to read the touch multipliers: %d", ret);
                return ret;
            }
            data->touch_mult_read = true;
        }
        // Raise only the set threshold: a hover never sets a touch, while keeping the module's clear
        // threshold stops a light, slow-moving touch from flickering out.
        uint8_t set = MIN(data->touch_mult_base[0] + config->touch_threshold_increase, UINT8_MAX);
        LOG_INF("Touch multiplier set %u -> %u, clear %u (kept)", data->touch_mult_base[0], set,
                data->touch_mult_base[1]);
        ret = iqs5xx_write_reg8(dev, IQS5XX_GLOBAL_TOUCH_SET, set);
        if (ret < 0) {
            LOG_ERR("Failed to set the touch multiplier: %d", ret);
            return ret;
        }
    }

    uint8_t two_finger_gestures = 0;
    two_finger_gestures |= config->two_finger_tap ? IQS5XX_TWO_FINGER_TAP : 0;
    two_finger_gestures |= config->scroll ? IQS5XX_SCROLL : 0;
    // Configure multi finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
        return ret;
    }

    // Configure axes.
    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }

    // Configure system settings.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, IQS5XX_SETUP_COMPLETE | IQS5XX_WDT);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }

    // End communication window.
    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }

    return 0;
}

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);
    k_work_init_delayable(&data->inertia_work, iqs5xx_inertia_work_handler);
    k_work_init_delayable(&data->ble_flush_work, iqs5xx_ble_flush_work_handler);
    k_work_init(&data->inertia_stop_work, iqs5xx_inertia_stop_work_handler);

    // Seed the runtime inertia and cursor parameters from the devicetree defaults;
    // saved settings (if any) override these at the APPLICATION init phase.
    iqs5xx_inertia_data = data;
    iqs5xx_inertia_config = config;
    data->inertia_enabled = config->inertia_cursor || config->inertia_scroll;
    data->inertia_friction_rt = config->inertia_friction;
    data->inertia_min_speed_rt = config->inertia_min_speed;
    data->rotation_deg_rt = config->rotation_deg;
    data->cursor_scale_pct_rt = IQS5XX_CURSOR_SCALE_PCT_DEFAULT;
    data->x_scale_pct_rt = CLAMP(config->x_scale_pct, IQS5XX_X_SCALE_PCT_MIN, IQS5XX_X_SCALE_PCT_MAX);
    iqs5xx_update_rotation_trig(data);
#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&data->inertia_save_work, iqs5xx_inertia_save_work_handler);
#endif

    // Start the dedicated report workqueue once (shared across instances).
    if (!iqs5xx_work_q_started) {
        k_work_queue_init(&iqs5xx_work_q);
        k_work_queue_start(&iqs5xx_work_q, iqs5xx_work_q_stack,
                           K_THREAD_STACK_SIZEOF(iqs5xx_work_q_stack),
                           CONFIG_INPUT_AZOTEQ_IQS5XX_WORKQUEUE_PRIORITY, NULL);
        k_thread_name_set(&iqs5xx_work_q.thread, "iqs5xx");
        iqs5xx_work_q_started = true;
#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_WATCHDOG)
        zmk_crash_guard_watch_work_q(&iqs5xx_work_q, "iqs5xx");
#endif
    }

    // Configure reset GPIO if available.
    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        // Reset the device.
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(1);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(10);
    }

    // Configure RDY GPIO.
    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY GPIO: %d", ret);
        return ret;
    }

    /*
     * The IQS5xx only accepts I2C transactions while RDY is asserted (its
     * communication window). The original code waited a fixed 100ms and then
     * wrote the configuration blindly, which NACKs (-EIO) when the window is
     * closed. Instead, wait for RDY to assert before each setup attempt and
     * retry a few times.
     */
    for (int attempt = 0; attempt < 10; attempt++) {
        int waited = 0;
        while (gpio_pin_get_dt(&config->rdy_gpio) <= 0 && waited < 100) {
            k_msleep(1);
            waited++;
        }

        ret = iqs5xx_setup_device(dev);
        if (ret == 0) {
            break;
        }

        LOG_WRN("Setup attempt %d failed (%d), retrying", attempt + 1, ret);
        k_msleep(10);
    }
    if (ret < 0) {
        LOG_ERR("Failed to setup device: %d", ret);
        return ret;
    }

    /*
     * Enable the RDY interrupt only AFTER setup, to avoid the work handler
     * racing for the bus during initialization.
     */
    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add RDY callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY interrupt: %d", ret);
        return ret;
    }

    data->initialized = true;
    LOG_INF("IQS5xx trackpad initialized");

    return 0;
}

// Replace CONFIG_INPUT_INIT_PRIORITY with the azoteq specific value.
#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .tap_and_drag = DT_INST_PROP(n, tap_and_drag),                                             \
        .accel_gain_min = DT_INST_PROP_OR(n, accel_gain_min, 100),                                 \
        .accel_gain_max = DT_INST_PROP_OR(n, accel_gain_max, 100),                                 \
        .accel_speed_low = DT_INST_PROP_OR(n, accel_speed_low, 0),                                 \
        .accel_speed_high = DT_INST_PROP_OR(n, accel_speed_high, 0),                               \
        .x_scale_pct = DT_INST_PROP_OR(n, x_scale_pct, 100),                                       \
        .ble_min_report_interval_ms = DT_INST_PROP_OR(n, ble_min_report_interval_ms, 0),           \
        .tap_distance = DT_INST_PROP_OR(n, tap_distance, 0),                                       \
        .touch_threshold_increase = DT_INST_PROP_OR(n, touch_threshold_increase, 0),               \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
        .active_report_rate = DT_INST_PROP_OR(n, active_report_rate, 0),                           \
        .x_resolution = DT_INST_PROP_OR(n, x_resolution, 0),                                       \
        .y_resolution = DT_INST_PROP_OR(n, y_resolution, 0),                                       \
        .rotation_deg = (int16_t)DT_INST_PROP_OR(n, rotation_deg, 0),                              \
        .inertia_cursor = DT_INST_PROP(n, inertia_cursor),                                         \
        .inertia_scroll = DT_INST_PROP(n, inertia_scroll),                                         \
        .inertia_friction = DT_INST_PROP_OR(n, inertia_friction, 242),                             \
        .inertia_min_speed = DT_INST_PROP_OR(n, inertia_min_speed, 2),                             \
        .inertia_tick_ms = DT_INST_PROP_OR(n, inertia_tick_ms, 8),                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
