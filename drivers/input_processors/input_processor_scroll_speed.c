/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 *
 * Input processor that scales scroll (wheel / horizontal wheel) events by a
 * runtime percentage chosen per host: one value for USB and one per BLE
 * profile. Apps differ a lot in how far one wheel notch scrolls, and a phone
 * or tablet on its own BLE profile often scrolls much further than a desktop,
 * so each host keeps its own speed.
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_speed

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <drivers/input_processor.h>

#include <zmk/endpoints.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <zmk_scroll_speed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SCROLL_SPEED_PCT_MIN 10
#define SCROLL_SPEED_PCT_MAX 300
#define SCROLL_SPEED_PCT_DEFAULT 100

#if IS_ENABLED(CONFIG_ZMK_BLE)
#define SCROLL_SPEED_BLE_SLOTS ZMK_BLE_PROFILE_COUNT
#else
#define SCROLL_SPEED_BLE_SLOTS 0
#endif

// Slot 0 = USB, slot 1 + n = BLE profile n.
#define SCROLL_SPEED_SLOT_USB 0
#define SCROLL_SPEED_SLOTS (1 + SCROLL_SPEED_BLE_SLOTS)
#define SCROLL_SPEED_PERSIST_MAX_SLOTS 16

BUILD_ASSERT(SCROLL_SPEED_SLOTS <= SCROLL_SPEED_PERSIST_MAX_SLOTS,
             "Too many BLE profiles for the persisted scroll speed table");

static uint16_t scroll_pct[SCROLL_SPEED_SLOTS] = {
    [0 ... SCROLL_SPEED_SLOTS - 1] = SCROLL_SPEED_PCT_DEFAULT,
};

static int current_slot(void) {
    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();
    if (ep.transport == ZMK_TRANSPORT_NONE) {
        ep = zmk_endpoint_get_preferred();
    }
    if (ep.transport == ZMK_TRANSPORT_BLE && ep.ble.profile_index >= 0 &&
        ep.ble.profile_index < SCROLL_SPEED_BLE_SLOTS) {
        return 1 + ep.ble.profile_index;
    }
    return SCROLL_SPEED_SLOT_USB;
}

static int scroll_speed_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    ARG_UNUSED(dev);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t value = (int32_t)event->value * (int32_t)scroll_pct[current_slot()];
    if (state && state->remainder) {
        value += *state->remainder;
    }
    int32_t scaled = value / 100;
    if (state && state->remainder) {
        *state->remainder = (int16_t)(value - scaled * 100);
    }
    event->value = CLAMP(scaled, INT16_MIN, INT16_MAX);

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api scroll_speed_driver_api = {
    .handle_event = scroll_speed_handle_event,
};

#if IS_ENABLED(CONFIG_SETTINGS)
static void scroll_speed_save_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    int rc = settings_save_one("iqs5xx_scroll/pct", scroll_pct, sizeof(scroll_pct));
    if (rc) {
        LOG_WRN("Failed to save scroll speed settings: %d", rc);
    }
}

// Runs on the system workqueue so the flash write never stalls input processing.
static K_WORK_DELAYABLE_DEFINE(scroll_speed_save_work, scroll_speed_save_work_handler);

static int scroll_speed_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                     void *cb_arg) {
    if (!settings_name_steq(name, "pct", NULL)) {
        return -ENOENT;
    }

    // The table length follows the BLE profile count; accept a different
    // count from an older build and keep the slots both builds share.
    uint16_t stored[SCROLL_SPEED_PERSIST_MAX_SLOTS];
    if (len == 0 || len > sizeof(stored) || (len % sizeof(stored[0])) != 0) {
        return -EINVAL;
    }
    int rc = read_cb(cb_arg, stored, len);
    if (rc < 0) {
        return rc;
    }

    size_t count = MIN(len / sizeof(stored[0]), (size_t)SCROLL_SPEED_SLOTS);
    for (size_t i = 0; i < count; i++) {
        scroll_pct[i] = CLAMP(stored[i], SCROLL_SPEED_PCT_MIN, SCROLL_SPEED_PCT_MAX);
    }
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(iqs5xx_scroll, "iqs5xx_scroll", NULL, scroll_speed_settings_set,
                               NULL, NULL);

static int scroll_speed_settings_load(void) { return settings_load_subtree("iqs5xx_scroll"); }
SYS_INIT(scroll_speed_settings_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif // CONFIG_SETTINGS

static void scroll_speed_schedule_save(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_reschedule(&scroll_speed_save_work, K_SECONDS(2));
#endif
}

static void slot_to_str(int slot, char *buf, size_t size) {
    if (slot == SCROLL_SPEED_SLOT_USB) {
        snprintf(buf, size, "USB");
    } else {
        snprintf(buf, size, "BLE%d", slot - 1);
    }
}

void zmk_scroll_speed_adjust(int delta_pct) {
    int slot = current_slot();
    int v = CLAMP((int)scroll_pct[slot] + delta_pct, SCROLL_SPEED_PCT_MIN, SCROLL_SPEED_PCT_MAX);
    scroll_pct[slot] = (uint16_t)v;
    LOG_INF("Scroll speed slot %d: %d%%", slot, v);
    scroll_speed_schedule_save();
}

void zmk_scroll_speed_reset_current(void) {
    int slot = current_slot();
    scroll_pct[slot] = SCROLL_SPEED_PCT_DEFAULT;
    LOG_INF("Scroll speed slot %d reset", slot);
    scroll_speed_schedule_save();
}

int zmk_scroll_speed_format_status(char *buf, size_t size) {
    char name[16];
    int cur = current_slot();
    slot_to_str(cur, name, sizeof(name));
    int off = snprintf(buf, size, "[Scroll] Now:%s %u%% |", name, (unsigned)scroll_pct[cur]);
    for (int i = 0; i < SCROLL_SPEED_SLOTS && off > 0 && (size_t)off < size; i++) {
        slot_to_str(i, name, sizeof(name));
        off += snprintf(buf + off, size - off, " %s:%u", name, (unsigned)scroll_pct[i]);
    }
    if (off > 0 && (size_t)off < size) {
        off += snprintf(buf + off, size - off, "\n");
    }
    return off;
}

#define SCROLL_SPEED_INST(n)                                                                       \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scroll_speed_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_SPEED_INST)
