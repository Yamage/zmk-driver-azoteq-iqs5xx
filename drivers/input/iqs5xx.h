#include <zephyr/device.h>

#define IQS5XX_NUM_FINGERS 0x0011
#define IQS5XX_REL_X 0x0012          // 2 bytes.
#define IQS5XX_REL_Y 0x0014          // 2 bytes.
#define IQS5XX_ABS_X 0x0016          // 2 bytes.
#define IQS5XX_ABS_Y 0x0018          // 2 bytes.
#define IQS5XX_TOUCH_STRENGTH 0x001A // 2 bytes.
#define IQS5XX_TOUCH_AREA 0x001C

#define IQS5XX_BOTTOM_BETA 0x0637
#define IQS5XX_STATIONARY_THRESH 0x0672

// Trackpad resolution (max coordinate), 2 bytes each. Higher values give the
// chip finer internal tracking -> smoother slow movement (less "staircase") and
// higher effective sensitivity; compensate speed with an input-processor scaler.
#define IQS5XX_X_RESOLUTION 0x066E
#define IQS5XX_Y_RESOLUTION 0x0670

#define IQS5XX_END_COMM_WINDOW 0xEEEE

// Active mode report rate, in ms (2 bytes wide). Lower = faster/smoother.
#define IQS5XX_ACTIVE_REPORT_RATE 0x057A

#define IQS5XX_SYSTEM_CONTROL_0 0x0431
// System Control 0 bits.
#define IQS5XX_ACK_RESET BIT(7)
#define IQS5XX_AUTO_ATI BIT(5)
#define IQS5XX_ALP_RESEED BIT(4)
#define IQS5XX_RESEED BIT(3)

#define IQS5XX_SYSTEM_CONFIG_0 0x058E
// System Config 0 bits.
#define IQS5XX_MANUAL_CONTROL BIT(7)
#define IQS5XX_SETUP_COMPLETE BIT(6)
#define IQS5XX_WDT BIT(5)
#define IQS5XX_SW_INPUT_EVENT BIT(4)
#define IQS5XX_ALP_REATI BIT(3)
#define IQS5XX_REATI BIT(2)
#define IQS5XX_SW_INPUT_SELECT BIT(1)
#define IQS5XX_SW_INPUT BIT(0)

#define IQS5XX_SYSTEM_CONFIG_1 0x058F
// System Config 1 bits.
#define IQS5XX_EVENT_MODE BIT(0)
#define IQS5XX_GESTURE_EVENT BIT(1)
#define IQS5XX_TP_EVENT BIT(2)
#define IQS5XX_REATI_EVENT BIT(3)
#define IQS5XX_ALP_PROX_EVENT BIT(4)
#define IQS5XX_SNAP_EVENT BIT(5)
#define IQS5XX_TOUCH_EVENT BIT(6)
#define IQS5XX_PROX_EVENT BIT(7)

// Filter settings register.
#define IQS5XX_FILTER_SETTINGS 0x0632
// Filter settings bits.
#define IQS5XX_IIR_FILTER BIT(0)
#define IQS5XX_MAV_FILTER BIT(1)
#define IQS5XX_IIR_SELECT BIT(2)
#define IQS5XX_ALP_COUNT_FILTER BIT(3)

#define IQS5XX_SYSTEM_INFO_0 0x000F
// System Info 0 bits.
#define IQS5XX_SHOW_RESET BIT(7)
#define IQS5XX_ALP_REATI_OCCURRED BIT(6)
#define IQS5XX_ALP_ATI_ERROR BIT(5)
#define IQS5XX_REATI_OCCURRED BIT(4)
#define IQS5XX_ATI_ERROR BIT(3)

#define IQS5XX_SYSTEM_INFO_1 0x0010
// System Info 1 bits.
#define IQS5XX_SWITCH_STATE BIT(5)
#define IQS5XX_SNAP_TOGGLE BIT(4)
#define IQS5XX_RR_MISSED BIT(3)
#define IQS5XX_TOO_MANY_FINGERS BIT(2)
#define IQS5XX_PALM_DETECT BIT(1)
#define IQS5XX_TP_MOVEMENT BIT(0)

// These 2 registers have the same bit map.
// The first one configures the gestures,
// the second one reports gesture events at runtime.
#define IQS5XX_SINGLE_FINGER_GESTURES_CONF 0x06B7
#define IQS5XX_GESTURE_EVENTS_0 0x000D
// Single finger gesture identifiers.
#define IQS5XX_SINGLE_TAP BIT(0)
#define IQS5XX_PRESS_AND_HOLD BIT(1)
#define IQS5XX_SWIPE_LEFT BIT(2)
#define IQS5XX_SWIPE_RIGHT BIT(3)
#define IQS5XX_SWIPE_UP BIT(4)
#define IQS5XX_SWIPE_DOWN BIT(5)

// Time in ms, 2 registers wide.
// Hold time + tap time is used as
// a threshold for the press and
// hold gesture.
#define IQS5XX_HOLD_TIME 0x06BD
// TODO: Make hold time configurable with KConfig.

// Mouse button helpers.
#define LEFT_BUTTON_BIT BIT(0)
#define RIGHT_BUTTON_BIT BIT(1)
#define MIDDLE_BUTTON_BIT BIT(2)
#define LEFT_BUTTON_CODE INPUT_BTN_0
#define RIGHT_BUTTON_CODE INPUT_BTN_0 + 1
#define MIDDLE_BUTTON_CODE INPUT_BTN_0 + 2

// These 2 registers have the same bit map.
// The first one configures the gestures,
// the second one reports gesture events at runtime.
#define IQS5XX_MULTI_FINGER_GESTURES_CONF 0x06B8
#define IQS5XX_GESTURE_EVENTS_1 0x000E
// Multi finger gesture identifiers.
#define IQS5XX_TWO_FINGER_TAP BIT(0)
#define IQS5XX_SCROLL BIT(1)
#define IQS5XX_ZOOM BIT(2)

// Axes configuration.
#define IQS5XX_XY_CONFIG_0 0x0669
#define IQS5XX_FLIP_X BIT(0)
#define IQS5XX_FLIP_Y BIT(1)
#define IQS5XX_SWITCH_XY_AXIS BIT(2)

struct iqs5xx_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec reset_gpio;

    // Gesture configuration.
    bool one_finger_tap;
    bool press_and_hold;
    bool two_finger_tap;
    uint16_t press_and_hold_time;

    // Scrolling configuration.
    bool scroll;
    bool natural_scroll_x;
    bool natural_scroll_y;

    // Axes configuration.
    bool switch_xy;
    bool flip_x;
    bool flip_y;

    // Sensitivity. configuration.
    uint8_t bottom_beta;
    uint8_t stationary_threshold;

    // Active mode report rate in ms (0 keeps the chip default).
    uint16_t active_report_rate;

    // Trackpad resolution / max coordinate (0 keeps the chip default).
    uint16_t x_resolution;
    uint16_t y_resolution;

    // Cursor/scroll inertia (trackball-like glide after the finger is lifted).
    bool inertia_cursor;       // Enable glide for cursor movement.
    bool inertia_scroll;       // Enable glide for scrolling (flick scroll).
    uint16_t inertia_friction; // Velocity retained per tick, out of 256 (242 ~= 0.945).
    uint16_t inertia_min_speed; // Min release speed (counts/report) to start a glide.
    uint16_t inertia_tick_ms;  // Glide timer cadence in ms.
};

// Inertia/glide mode: which kind of motion the post-release glide replays.
#define IQS5XX_GLIDE_NONE 0
#define IQS5XX_GLIDE_CURSOR 1
#define IQS5XX_GLIDE_SCROLL 2

struct iqs5xx_data {
    const struct device *dev;
    struct gpio_callback rdy_cb;
    struct k_work work;
    struct k_work_delayable button_release_work;
    // TODO: Pack flags into a bitfield to save space.
    bool initialized;
    // Flag to indicate if the button was pressed in a previous cycle.
    uint8_t buttons_pressed;
    bool active_hold;
    // Scroll accumulators.
    int16_t scroll_x_acc;
    int16_t scroll_y_acc;

    // Inertia / glide state.
    struct k_work_delayable inertia_work;
    int32_t vel_x;       // Q8 fixed-point velocity estimate (counts/report).
    int32_t vel_y;
    int32_t glide_x_acc; // Q8 fractional carry for emitted glide movement.
    int32_t glide_y_acc;
    uint8_t glide_mode;  // IQS5XX_GLIDE_* : last motion kind, used while gliding.
    uint8_t last_fingers; // Finger count from the previous report (touch/release edge).

    // Runtime-tunable inertia parameters (initialised from the devicetree
    // defaults, then overridden by saved settings and the keymap controls).
    bool inertia_enabled;          // Master on/off, toggled from the keymap.
    uint16_t inertia_friction_rt;  // Live friction (retained/256 per tick).
    uint16_t inertia_min_speed_rt; // Live min release speed (counts/report).
    struct k_work_delayable inertia_save_work; // Debounced settings save.
};
