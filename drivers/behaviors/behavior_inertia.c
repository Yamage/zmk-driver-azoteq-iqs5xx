/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_inertia

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/split/central.h>

#include <iqs5xx_inertia.h>

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD)
#include <zmk_crash_guard.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Action codes carried in the behavior binding parameter. */
#define INERTIA_TOGGLE 0
#define INERTIA_FRICTION_UP 1
#define INERTIA_FRICTION_DOWN 2
#define INERTIA_MIN_SPEED_UP 3
#define INERTIA_MIN_SPEED_DOWN 4
#define INERTIA_RESET 5
#define INERTIA_BLE_DIAG 6
#define INERTIA_KEYMAP_GUIDE 7
#define INERTIA_ROT_CW 8
#define INERTIA_ROT_CCW 9
#define INERTIA_SCALE_UP 10
#define INERTIA_SCALE_DOWN 11

/* Tuning step sizes for the adjustable parameters. */
#define INERTIA_FRICTION_STEP 8
#define INERTIA_MIN_SPEED_STEP 2
#define INERTIA_ROT_STEP_DEG 5
#define INERTIA_SCALE_STEP_PCT 10
#define DIAG_TYPE_STEP_MS 12

static char diag_buf[512];
static const char *diag_active_text;
static size_t diag_pos;
static uint32_t diag_active_keycode;
static bool diag_key_pressed;
static struct k_work_delayable diag_type_work;

static const char keymap_guide_ascii[] =
    "=== My Keyboard Keymap Guide ===\n"
    "Base\n"
    " +[ hold -> L1\n"
    " |  +} hold -> L3\n"
    " +] hold -> L2\n"
    " |  +{ hold -> L3\n"
    " |  +Q hold -> L6\n"
    " |     +R-top1 hold -> L7\n"
    " +Touchpad -> L4\n"
    "    +S/; hold -> L5 Scroll\n"
    ".. = transparent\n"
    "\n"
    "[Base]\n"
    "Q     W     E     R     T     CTRL  | SHFT  Y     U     I     O     P\n"
    "A     S     D     F     G     [/L1  | ]/L2  H     J     K     L     ;\n"
    "Z     X     C     V     B     SPC   | ENT   N     M/Win ,/Alt ./Ctl //Sft\n"
    "\n"
    "[L1 Sym/Num/Arrow]\n"
    "!     @     #     $     %     ..    | SHFT  ^     &     *     '     =\n"
    "1     2     3     4     5     ..    | }/L3  Left  Down  Up    Right :\n"
    "6     7     8     9     0     ..    | ENT   -     +     (     )     ?\n"
    "\n"
    "[L2 Edit/NumPad/IME]\n"
    "->L6  ..    ..    ..    ..    CTRL  | ALT   ..    S+INS DEL   BSPC  ESC\n"
    "KP1   KP2   KP3   KP4   KP5   {/L3  | ..    ..    PgDn  PgUp  C+Spc Alt+`\n"
    "KP6   KP7   KP8   KP9   KP0   TAB   | ENT   '     ..    ALT   CTRL  \\/Sft\n"
    "\n"
    "[L3 Fn/NumPad Ops]\n"
    "F11   F12   ..    ..    ..    ..    | ..    `     ..    ..    \"     |\n"
    "F1    F2    F3    F4    F5    ..    | ..    ..    PgDn  PgUp  CTRL  SHFT\n"
    "F6    F7    F8    F9    F10   ..    | ..    KP-   KP+   ALT   KP./C KP//S\n"
    "\n"
    "[L4 Auto Mouse]\n"
    "..    ..    ..    ..    ..    ..    | ..    ..    ..    ..    ..    ..\n"
    "..    Scrl  RClk  MClk  LClk  [/L1  | ]/L2  ..    ..    ..    ..    Scrl\n"
    "Z/Sft X/Ctl C/Alt ..    ..    ..    | ..    LClk  MClk  RClk  ..    ..\n"
    "\n"
    "[L6 BLE/Studio/Diag]\n"
    "..    ..    ..    ..    Diag  ..    | ->L7  BT0   BT1   BT2   BT3   BT4\n"
    "..    ..    ..    ..    ..    ..    | ..    ..    ..    Guide ..    ..\n"
    "..    ..    ..    ..    ..    ..    | ..    BTCLR OutTg Unlck ..    ..\n"
    "\n"
    "[L7 Touchpad (Fric=glide, MSpd=flick, Rot=angle, Spd=cursor speed)]\n"
    "..    ..    ..    ..    ..    ..    | ..    Fric+ Fric- MSpd+ MSpd- ..\n"
    "..    ..    ..    ..    ..    ..    | ..    Inert Rot+  Rot-  Spd+  Spd-\n"
    "..    ..    ..    ..    ..    ..    | ..    Reset ..    ..    ..    ..\n";

static uint32_t char_to_zmk_keycode(char c) {
    if (c >= 'a' && c <= 'z') {
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_A + (c - 'a'));
    }
    if (c >= 'A' && c <= 'Z') {
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_A + (c - 'A')));
    }
    if (c >= '1' && c <= '9') {
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION + (c - '1'));
    }
    switch (c) {
    case '0':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS);
    case ' ':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_SPACEBAR);
    case '\n':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_RETURN_ENTER);
    case '-':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_MINUS_AND_UNDERSCORE);
    case '_':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_MINUS_AND_UNDERSCORE));
    case '=':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_EQUAL_AND_PLUS);
    case '+':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_EQUAL_AND_PLUS));
    case '[':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_LEFT_BRACKET_AND_LEFT_BRACE);
    case '{':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_LEFT_BRACKET_AND_LEFT_BRACE));
    case ']':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_RIGHT_BRACKET_AND_RIGHT_BRACE);
    case '}':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_RIGHT_BRACKET_AND_RIGHT_BRACE));
    case '(':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_9_AND_LEFT_PARENTHESIS));
    case ')':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS));
    case ':':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_SEMICOLON_AND_COLON));
    case ';':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_SEMICOLON_AND_COLON);
    case '\'':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_APOSTROPHE_AND_QUOTE);
    case '"':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_APOSTROPHE_AND_QUOTE));
    case '`':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_GRAVE_ACCENT_AND_TILDE);
    case '~':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_GRAVE_ACCENT_AND_TILDE));
    case '/':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_SLASH_AND_QUESTION_MARK);
    case '?':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_SLASH_AND_QUESTION_MARK));
    case '\\':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_BACKSLASH_AND_PIPE);
    case '|':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_BACKSLASH_AND_PIPE));
    case '%':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_5_AND_PERCENT));
    case '.':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_PERIOD_AND_GREATER_THAN);
    case '>':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_PERIOD_AND_GREATER_THAN));
    case ',':
        return ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_COMMA_AND_LESS_THAN);
    case '<':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_COMMA_AND_LESS_THAN));
    case '!':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION));
    case '@':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_2_AND_AT));
    case '#':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_3_AND_HASH));
    case '$':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_4_AND_DOLLAR));
    case '^':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_6_AND_CARET));
    case '&':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_7_AND_AMPERSAND));
    case '*':
        return LS(ZMK_HID_USAGE(HID_USAGE_KEY, HID_USAGE_KEY_KEYBOARD_8_AND_ASTERISK));
    default:
        return 0;
    }
}

#if IS_ENABLED(CONFIG_BT)
static int read_conn_rssi(struct bt_conn *conn, int8_t *rssi_out) {
    uint16_t handle;
    int err = bt_hci_get_conn_handle(conn, &handle);
    if (err) {
        return err;
    }

    struct net_buf *buf =
        bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(struct bt_hci_cp_read_rssi));
    if (!buf) {
        return -ENOBUFS;
    }

    struct bt_hci_cp_read_rssi *cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);

    struct net_buf *rsp = NULL;
    err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
    if (err) {
        return err;
    }

    struct bt_hci_rp_read_rssi *rp = (void *)rsp->data;
    int8_t rssi = rp->rssi;
    net_buf_unref(rsp);

    if (rssi == BT_HCI_LE_RSSI_NOT_AVAILABLE) {
        return -ENOENT;
    }

    *rssi_out = rssi;
    return 0;
}

static const char *rssi_quality_label(int8_t rssi) {
    if (rssi >= -60) {
        return "Good";
    }
    if (rssi >= -72) {
        return "Fair";
    }
    return "Weak";
}

static void find_split_conn_cb(struct bt_conn *conn, void *user_data) {
    struct bt_conn **out = user_data;
    if (*out != NULL) {
        return;
    }
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED &&
        info.role == BT_CONN_ROLE_CENTRAL) {
        *out = bt_conn_ref(conn);
    }
}

static int format_conn_line(char *buf, size_t size, const char *label, struct bt_conn *conn,
                            const char *disconn_note) {
    struct bt_conn_info info;
    if (conn == NULL || bt_conn_get_info(conn, &info) != 0 ||
        info.state != BT_CONN_STATE_CONNECTED) {
        return snprintf(buf, size, "[%s] DISCONNECTED (%s)\n", label, disconn_note);
    }

    uint16_t interval = info.le.interval;
    uint32_t int_x100 = (uint32_t)interval * 125;
    uint32_t hz = interval > 0 ? (800 + interval / 2) / interval : 0;
    uint32_t timeout_ms = (uint32_t)info.le.timeout * 10;

    int8_t rssi = 0;
    if (read_conn_rssi(conn, &rssi) == 0) {
        return snprintf(buf, size,
                        "[%s] RSSI:%ddBm (%s) | Int:%u.%02ums (%uHz, #%u) | Lat:%u | Tmo:%ums\n",
                        label, (int)rssi, rssi_quality_label(rssi), int_x100 / 100, int_x100 % 100,
                        hz, interval, info.le.latency, timeout_ms);
    }
    return snprintf(buf, size,
                    "[%s] RSSI:N/A | Int:%u.%02ums (%uHz, #%u) | Lat:%u | Tmo:%ums\n",
                    label, int_x100 / 100, int_x100 % 100, hz, interval, info.le.latency,
                    timeout_ms);
}
#endif

static void build_ble_diag_report(char *buf, size_t size) {
    bool is_ble = zmk_endpoint_get_selected().transport == ZMK_TRANSPORT_BLE;
    int profile = 0;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    profile = zmk_ble_active_profile_index();
#endif
    uint8_t right_batt = 0;
#if IS_ENABLED(CONFIG_ZMK_BATTERY)
    right_batt = zmk_battery_state_of_charge();
#endif

    struct bt_conn *host_conn = NULL;
    struct bt_conn *split_conn = NULL;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    host_conn = zmk_ble_active_profile_conn();
    bt_conn_foreach(BT_CONN_TYPE_LE, find_split_conn_cb, &split_conn);
#endif

    char left_batt_str[12] = "--%";
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    uint8_t left_batt = 0;
    if (split_conn != NULL &&
        zmk_split_central_get_peripheral_battery_level(0, &left_batt) == 0 && left_batt > 0) {
        snprintf(left_batt_str, sizeof(left_batt_str), "%u%%", left_batt);
    }
#endif

    int off = snprintf(buf, size,
                       "=== My Keyboard Status ===\n"
                       "[Out] %s (Profile %d) | [Batt] L:%s / R:%u%%\n",
                       is_ble ? "BLE" : "USB", profile, left_batt_str, right_batt);

#if IS_ENABLED(CONFIG_BT)
    if (off > 0 && (size_t)off < size) {
        off += format_conn_line(buf + off, size - off, "Host PC", host_conn,
                                is_ble ? "No BLE conn" : "USB active");
    }
    if (off > 0 && (size_t)off < size) {
        off += format_conn_line(buf + off, size - off, "Left KB", split_conn, "Scanning!");
    }
    if (host_conn != NULL) {
        bt_conn_unref(host_conn);
    }
    if (split_conn != NULL) {
        bt_conn_unref(split_conn);
    }
#endif

    if (off > 0 && (size_t)off < size) {
        off += iqs5xx_format_status(buf + off, size - off);
    }

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD)
    if (off > 0 && (size_t)off < size) {
        off += zmk_crash_guard_format(buf + off, size - off);
    }
#endif
}

static bool stop_active_typing_if_running(void) {
    bool was_running = k_work_delayable_is_pending(&diag_type_work) || diag_key_pressed ||
                       (diag_active_text != NULL && diag_active_text[diag_pos] != '\0');
    k_work_cancel_delayable(&diag_type_work);
    if (diag_key_pressed) {
        raise_zmk_keycode_state_changed_from_encoded(diag_active_keycode, false, k_uptime_get());
        diag_key_pressed = false;
    }
    diag_active_text = NULL;
    diag_pos = 0;
    return was_running;
}

static void start_text_typing(const char *text) {
    if (stop_active_typing_if_running()) {
        return;
    }
    diag_active_text = text;
    diag_pos = 0;
    k_work_schedule(&diag_type_work, K_MSEC(80));
}

static void diag_type_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (diag_key_pressed) {
        raise_zmk_keycode_state_changed_from_encoded(diag_active_keycode, false, k_uptime_get());
        diag_key_pressed = false;
        if (diag_active_text == NULL || diag_active_text[diag_pos] == '\0') {
            diag_active_text = NULL;
            return;
        }
        k_work_schedule(&diag_type_work, K_MSEC(DIAG_TYPE_STEP_MS));
        return;
    }

    if (diag_active_text == NULL) {
        return;
    }

    while (diag_active_text[diag_pos] != '\0') {
        uint32_t kc = char_to_zmk_keycode(diag_active_text[diag_pos++]);
        if (kc == 0) {
            continue;
        }
        diag_active_keycode = kc;
        diag_key_pressed = true;
        raise_zmk_keycode_state_changed_from_encoded(kc, true, k_uptime_get());
        k_work_schedule(&diag_type_work, K_MSEC(DIAG_TYPE_STEP_MS));
        return;
    }
    diag_active_text = NULL;
}

static void start_ble_diag_typing(void) {
    if (stop_active_typing_if_running()) {
        return;
    }
    build_ble_diag_report(diag_buf, sizeof(diag_buf));
    diag_active_text = diag_buf;
    diag_pos = 0;
    k_work_schedule(&diag_type_work, K_MSEC(80));
}

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
    case INERTIA_BLE_DIAG:
        start_ble_diag_typing();
        break;
    case INERTIA_KEYMAP_GUIDE:
        start_text_typing(keymap_guide_ascii);
        break;
    case INERTIA_ROT_CW:
        iqs5xx_adjust_rotation(INERTIA_ROT_STEP_DEG);
        break;
    case INERTIA_ROT_CCW:
        iqs5xx_adjust_rotation(-INERTIA_ROT_STEP_DEG);
        break;
    case INERTIA_SCALE_UP:
        iqs5xx_adjust_cursor_scale(INERTIA_SCALE_STEP_PCT);
        break;
    case INERTIA_SCALE_DOWN:
        iqs5xx_adjust_cursor_scale(-INERTIA_SCALE_STEP_PCT);
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

static int behavior_inertia_init(const struct device *dev) {
    ARG_UNUSED(dev);
    k_work_init_delayable(&diag_type_work, diag_type_work_handler);
    return 0;
}

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

bool iqs5xx_is_ble_transport(void) {
    return zmk_endpoint_get_selected().transport == ZMK_TRANSPORT_BLE;
}
