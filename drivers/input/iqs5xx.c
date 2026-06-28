/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
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

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

// Bounds for the runtime-tunable inertia parameters.
#define IQS5XX_FRICTION_MIN 160
#define IQS5XX_FRICTION_MAX 252
#define IQS5XX_MIN_SPEED_MIN 0
#define IQS5XX_MIN_SPEED_MAX 60

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
            // NOTE: This is a potential race.
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

    if (data->glide_mode == IQS5XX_GLIDE_CURSOR) {
        data->glide_x_acc += data->vel_x;
        data->glide_y_acc += data->vel_y;
        int32_t out_x = data->glide_x_acc >> 8;
        int32_t out_y = data->glide_y_acc >> 8;
        data->glide_x_acc -= out_x << 8;
        data->glide_y_acc -= out_y << 8;
        if (out_x != 0 || out_y != 0) {
            input_report_rel(dev, INPUT_REL_X, out_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, out_y, true, K_FOREVER);
        }
    } else if (data->glide_mode == IQS5XX_GLIDE_SCROLL) {
        const int16_t scroll_div = 32;
        data->glide_x_acc += data->vel_x;
        data->glide_y_acc += data->vel_y;
        int32_t step_x = data->glide_x_acc >> 8;
        int32_t step_y = data->glide_y_acc >> 8;
        data->glide_x_acc -= step_x << 8;
        data->glide_y_acc -= step_y << 8;
        if (step_x != 0) {
            data->scroll_x_acc += step_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
        }
        if (step_y != 0) {
            data->scroll_y_acc += step_y;
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

/* ---- Runtime inertia control + flash persistence ---- */

#if IS_ENABLED(CONFIG_SETTINGS)
struct iqs5xx_inertia_persist {
    uint8_t enabled;
    uint16_t friction;
    uint16_t min_speed;
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
    struct iqs5xx_inertia_persist p;
    if (len != sizeof(p) || data == NULL) {
        return -EINVAL;
    }
    int rc = read_cb(cb_arg, &p, sizeof(p));
    if (rc < 0) {
        return rc;
    }
    data->inertia_enabled = p.enabled != 0;
    data->inertia_friction_rt = CLAMP(p.friction, IQS5XX_FRICTION_MIN, IQS5XX_FRICTION_MAX);
    data->inertia_min_speed_rt = CLAMP(p.min_speed, IQS5XX_MIN_SPEED_MIN, IQS5XX_MIN_SPEED_MAX);
    return 0;
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
    k_work_schedule_for_queue(&iqs5xx_work_q, &data->inertia_save_work, K_SECONDS(2));
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

void iqs5xx_inertia_reset(void) {
    struct iqs5xx_data *data = iqs5xx_inertia_data;
    const struct iqs5xx_config *config = iqs5xx_inertia_config;
    if (data == NULL || config == NULL) {
        return;
    }
    data->inertia_enabled = config->inertia_cursor || config->inertia_scroll;
    data->inertia_friction_rt = config->inertia_friction;
    data->inertia_min_speed_rt = config->inertia_min_speed;
    LOG_INF("Inertia reset to defaults");
    iqs5xx_inertia_schedule_save(data);
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
     *   [4] number of fingers (unused)
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
        k_work_cancel_delayable(&data->inertia_work);
        data->glide_mode = IQS5XX_GLIDE_NONE;
        data->vel_x = 0;
        data->vel_y = 0;
        data->glide_x_acc = 0;
        data->glide_y_acc = 0;
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    } else if (fingers == 0 && data->last_fingers > 0) {
        // The finger was lifted: start gliding if the release was fast enough.
        bool enabled = data->inertia_enabled &&
                       ((data->glide_mode == IQS5XX_GLIDE_CURSOR && config->inertia_cursor) ||
                        (data->glide_mode == IQS5XX_GLIDE_SCROLL && config->inertia_scroll));
        int32_t min_q8 = (int32_t)data->inertia_min_speed_rt << 8;
        if (enabled && (abs(data->vel_x) >= min_q8 || abs(data->vel_y) >= min_q8)) {
            data->glide_x_acc = 0;
            data->glide_y_acc = 0;
            k_work_schedule_for_queue(&iqs5xx_work_q, &data->inertia_work,
                                      K_MSEC(config->inertia_tick_ms));
        } else {
            data->glide_mode = IQS5XX_GLIDE_NONE;
            data->vel_x = 0;
            data->vel_y = 0;
        }
    }
    data->last_fingers = fingers;

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = true;
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

        // Press the button immediately.
        input_report_key(dev, button_code, 1, true, K_FOREVER);
        data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);

        // Schedule release after 100ms.
        k_work_schedule(&data->button_release_work, K_MSEC(100));
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
    } else if (tp_movement) {
        if (rel_x != 0 || rel_y != 0) {
            input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
        }
        // Track velocity for cursor inertia.
        data->vel_x = (data->vel_x * 3 + ((int32_t)rel_x << 8)) >> 2;
        data->vel_y = (data->vel_y * 3 + ((int32_t)rel_y << 8)) >> 2;
        data->glide_mode = IQS5XX_GLIDE_CURSOR;
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

    // Optionally raise the active mode report rate for a smoother cursor.
    if (config->active_report_rate != 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_ACTIVE_REPORT_RATE, config->active_report_rate);
        if (ret < 0) {
            LOG_ERR("Failed to set active report rate: %d", ret);
            return ret;
        }
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

    // Seed the runtime inertia parameters from the devicetree defaults; saved
    // settings (if any) override these at the APPLICATION init phase.
    iqs5xx_inertia_data = data;
    iqs5xx_inertia_config = config;
    data->inertia_enabled = config->inertia_cursor || config->inertia_scroll;
    data->inertia_friction_rt = config->inertia_friction;
    data->inertia_min_speed_rt = config->inertia_min_speed;
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
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
        .active_report_rate = DT_INST_PROP_OR(n, active_report_rate, 0),                           \
        .x_resolution = DT_INST_PROP_OR(n, x_resolution, 0),                                       \
        .y_resolution = DT_INST_PROP_OR(n, y_resolution, 0),                                       \
        .inertia_cursor = DT_INST_PROP(n, inertia_cursor),                                         \
        .inertia_scroll = DT_INST_PROP(n, inertia_scroll),                                         \
        .inertia_friction = DT_INST_PROP_OR(n, inertia_friction, 242),                             \
        .inertia_min_speed = DT_INST_PROP_OR(n, inertia_min_speed, 2),                             \
        .inertia_tick_ms = DT_INST_PROP_OR(n, inertia_tick_ms, 8),                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
