/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_inertia

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>

#include <iqs5xx_inertia.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Action codes carried in the behavior binding parameter. */
#define INERTIA_TOGGLE 0
#define INERTIA_FRICTION_UP 1
#define INERTIA_FRICTION_DOWN 2
#define INERTIA_MIN_SPEED_UP 3
#define INERTIA_MIN_SPEED_DOWN 4
#define INERTIA_RESET 5

/* Tuning step sizes for the adjustable parameters. */
#define INERTIA_FRICTION_STEP 8
#define INERTIA_MIN_SPEED_STEP 2

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case INERTIA_TOGGLE:
        iqs5xx_inertia_toggle();
        break;
    case INERTIA_FRICTION_UP:
        iqs5xx_inertia_adjust_friction(INERTIA_FRICTION_STEP);
        break;
    case INERTIA_FRICTION_DOWN:
        iqs5xx_inertia_adjust_friction(-INERTIA_FRICTION_STEP);
        break;
    case INERTIA_MIN_SPEED_UP:
        iqs5xx_inertia_adjust_min_speed(INERTIA_MIN_SPEED_STEP);
        break;
    case INERTIA_MIN_SPEED_DOWN:
        iqs5xx_inertia_adjust_min_speed(-INERTIA_MIN_SPEED_STEP);
        break;
    case INERTIA_RESET:
        iqs5xx_inertia_reset();
        break;
    default:
        LOG_WRN("Unknown inertia action %d", binding->param1);
        return -ENOTSUP;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_inertia_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

static int behavior_inertia_init(const struct device *dev) { return 0; }

BEHAVIOR_DT_INST_DEFINE(0, behavior_inertia_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_inertia_driver_api);

// A glide started under one layer (e.g. Scroll) must not carry on as another action
// (cursor movement) after that layer changes.
static int inertia_layer_state_listener(const zmk_event_t *eh) {
    iqs5xx_inertia_stop();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(behavior_inertia_layer, inertia_layer_state_listener);
ZMK_SUBSCRIPTION(behavior_inertia_layer, zmk_layer_state_changed);
