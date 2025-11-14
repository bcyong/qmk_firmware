#include QMK_KEYBOARD_H
#include "process_unicode.h"
#include <print.h>
#include "common.h"
#include "immobile.h"
#include "color.h"

#ifdef MULTIMODE_ENABLE
#    include "multimode.h"
#endif

#ifdef RGB_MATRIX_BLINK_ENABLE
#    include "rgb_matrix_blink.h"
#endif

enum alt_keycodes {
    U_T_AUTO = SAFE_RANGE, //USB Extra Port Toggle Auto Detect / Always Active
    U_T_AGCR,              //USB Toggle Automatic GCR control
    DBG_TOG,               //DEBUG Toggle On / Off
    DBG_MTRX,              //DEBUG Toggle Matrix Prints
    DBG_KBD,               //DEBUG Toggle Keyboard Prints
    DBG_MOU,               //DEBUG Toggle Mouse Prints
    MD_BOOT,               //Restart into bootloader after hold timeout
};

enum alt_layers {
    _BL = 0,
    _FL,
    _ML,
};

#define KEY_COLOR_OFF 0x00, 0x00, 0x00
#define KEY_COLOR_HOMEEND RGB_CORAL
#define KEY_COLOR_ARROWS RGB_PURPLE
#define KEY_COLOR_PAGEUPDN RGB_MAGENTA
#define KEY_COLOR_MOUSE RGB_GOLD
#define KEY_COLOR_MOUSE_WHEEL RGB_GOLDENROD

#define TG_NKRO MAGIC_TOGGLE_NKRO //Toggle 6KRO / NKRO mode

// Tapdance Keycodes
enum td_keycodes {
    SPC_MBTN // BTN1/BTN2/BTN3 when tapped 1/2/3 times
};

// Tapdance States
typedef enum {
    SINGLE_TAP,
    DOUBLE_TAP,
    TRIPLE_TAP,
} td_state_t;

typedef struct {
    bool is_press_action;
    td_state_t state;
} td_tap_t;

void spc_mbtn_finished(tap_dance_state_t *state, void *user_data);
void spc_mbtn_reset(tap_dance_state_t *state, void *user_data);

// RGB Matrix Blink configuration
#ifdef RGB_MATRIX_BLINK_ENABLE
blink_rgb_t blink_rgbs[RGB_MATRIX_BLINK_COUNT] = {
    {.index = IM_MM_RGB_BLINK_INDEX_BT1, .interval = 250, .times = 1, .color = {.r = 0x00, .g = 0xFF, .b = 0x00}, .blink_cb = im_mm_rgb_blink_cb},
    {.index = IM_MM_RGB_BLINK_INDEX_BT2, .interval = 250, .times = 1, .color = {.r = 0x00, .g = 0xFF, .b = 0x00}, .blink_cb = im_mm_rgb_blink_cb},
    {.index = IM_MM_RGB_BLINK_INDEX_BT3, .interval = 250, .times = 1, .color = {.r = 0x00, .g = 0xFF, .b = 0x00}, .blink_cb = im_mm_rgb_blink_cb},
    {.index = IM_MM_RGB_BLINK_INDEX_2G4, .interval = 250, .times = 1, .color = {.r = 0x00, .g = 0xFF, .b = 0x00}, .blink_cb = im_mm_rgb_blink_cb},
    {.index = IM_MM_RGB_BLINK_INDEX_W2M, .interval = 500, .times = 3, .color = {.r = 150, .g = 150, .b = 150}, .blink_cb = NULL},
    {.index = RGB_MATRIX_BLINK_INDEX_BAT, .interval = 250, .times = 3, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL},
    {.index = RGB_MATRIX_BLINK_INDEX_ALL, .interval = 500, .times = 5, .color = {.r = 100, .g = 100, .b = 100}, .blink_cb = NULL},
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
    {.index = 0xFF, .interval = 0, .times = 0, .color = {.r = 0x00, .g = 0x00, .b = 0x00}, .blink_cb = NULL}, // Padding
};
#endif

keymap_config_t keymap_config;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_BL] = LAYOUT(
        QK_GESC, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, _______, KC_DEL, \
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_MUTE,  \
        KC_LGUI, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_ENT,           KC_PGUP, \
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,          KC_UP,   KC_PGDN, \
        KC_LCTL, MO(_FL), KC_LALT,                            KC_SPC,  KC_SPC,  KC_SPC,           MO(_FL), KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT  \
    ),
    [_FL] = LAYOUT(
        KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______, _______, KC_END, \
        _______, _______, _______, _______, _______, _______, _______, KC_HOME, KC_UP,   KC_PGUP, KC_PSCR, KC_SCRL, KC_PAUS, _______, _______, \
        _______, _______, _______, _______, _______, _______, _______, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______,          _______, KC_VOLU, \
        _______, _______, _______, _______, _______, _______, _______, KC_END,  _______, KC_PGDN, _______, _______,          KC_PGUP, KC_VOLD, \
        _______, _______, MO(_ML),                            _______,  _______,_______,          _______, _______, KC_HOME, KC_PGDN, KC_END   \
    ),
    [_ML] = LAYOUT(
        QK_BOOT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, QK_CLEAR_EEPROM,  XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_U, KC_WH_U, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_L, KC_MS_D, KC_MS_R, XXXXXXX, XXXXXXX,          XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_WH_D, XXXXXXX, XXXXXXX,          XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, _______,                            TD(SPC_MBTN), TD(SPC_MBTN), TD(SPC_MBTN),    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX  \
    ),
    
};

const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [_BL] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [_FL] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [_ML] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
};

// Runs just one time when the keyboard initializes.
void matrix_init_user(void) {
    // Set RGB matrix to custom mode for layer indicators
    rgb_matrix_mode_noeeprom(RGB_MATRIX_SOLID_COLOR);
    rgb_matrix_sethsv_noeeprom(0, 0, 0);
};

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {
};

#define MODS_SHIFT  (get_mods() & MOD_BIT(KC_LSFT) || get_mods() & MOD_BIT(KC_RSFT))
#define MODS_CTRL  (get_mods() & MOD_BIT(KC_LCTL) || get_mods() & MOD_BIT(KC_RCTL))
#define MODS_ALT  (get_mods() & MOD_BIT(KC_LALT) || get_mods() & MOD_BIT(KC_RALT))

bool fn_down = false;
bool alt_down = false;

// RGB Matrix indicator function - called every frame to set LED colors
bool rgb_matrix_indicators_user(void) {
    // Turn off all LEDs first
    for (uint8_t i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
        rgb_matrix_set_color(i, 0, 0, 0);
    }
    
    if (fn_down && alt_down) {
        // ML layer: Light up IJKLO. for mouse controls
        uint8_t keys_mouse[] = {KL_I, KL_J, KL_K, KL_L};
        foreach (uint8_t* key, keys_mouse) {
            rgb_matrix_set_color(*key, RGB_GOLD);
        };
        uint8_t keys_mousewheel[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_mousewheel) {
            rgb_matrix_set_color(*key, RGB_GOLDENROD);
        };
    } else if (fn_down) {
        // FL layer: Light up IJKLUMO. for navigation
        uint8_t keys_homeend[] = {KL_U, KL_M};
        foreach (uint8_t* key, keys_homeend) {
            rgb_matrix_set_color(*key, RGB_CORAL);
        };
        uint8_t keys_arrow[] = {KL_I, KL_J, KL_K, KL_L};
        foreach (uint8_t* key, keys_arrow) {
            rgb_matrix_set_color(*key, RGB_PURPLE);
        };
        uint8_t keys_page[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_page) {
            rgb_matrix_set_color(*key, RGB_MAGENTA);
        };
    }
    
    return false;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    static uint32_t key_timer;

    switch (keycode) {
        case MO(_FL):
            fn_down = record->event.pressed;
            break;

        case MO(_ML):
            alt_down = record->event.pressed;
            break;

        default:
            break;
    }

    switch (keycode) {
        /* Massdrop debug - disabled, functions not available in this firmware */
        case U_T_AUTO:
            // USB extra port toggle not available
            return false;
        case U_T_AGCR:
            // USB GCR auto mode not available
            return false;
        case DBG_TOG:
            if (record->event.pressed) {
                debug_enable = !debug_enable;
                uprintf("Debug mode: %s\n", debug_enable ? "on" : "off");
            }
            return false;
        case DBG_MTRX:
            if (record->event.pressed) {
                debug_matrix = !debug_matrix;
                uprintf("Debug matrix: %s\n", debug_matrix ? "on" : "off");
            }
            return false;
        case DBG_KBD:
            if (record->event.pressed) {
                debug_keyboard = !debug_keyboard;
                uprintf("Debug keyboard: %s\n", debug_keyboard ? "on" : "off");
            }
            return false;
        case DBG_MOU:
            if (record->event.pressed) {
                debug_mouse = !debug_mouse;
                uprintf("Debug mouse: %s\n", debug_mouse ? "on" : "off");
            }
            return false;
        case MD_BOOT:
            if (record->event.pressed) {
                key_timer = timer_read32();
            } else {
                if (timer_elapsed32(key_timer) >= 500) {
                    reset_keyboard();
                }
            }
            return false;
        case RGB_TOG:
            if (record->event.pressed) {            
                switch (rgb_matrix_get_flags()) {
                    case LED_FLAG_ALL: {
                        rgb_matrix_set_flags(LED_FLAG_KEYLIGHT);
                        rgb_matrix_set_color_all(0, 0, 0);
                    }
                    break;
                    case LED_FLAG_KEYLIGHT: {
                        rgb_matrix_set_flags(LED_FLAG_UNDERGLOW);
                        rgb_matrix_set_color_all(0, 0, 0);
                    }
                    break;
                    case LED_FLAG_UNDERGLOW: {
                        rgb_matrix_set_flags(LED_FLAG_NONE);
                        rgb_matrix_disable_noeeprom();
                    }
                    break;
                    default: {
                        rgb_matrix_set_flags(LED_FLAG_ALL);
                        rgb_matrix_enable_noeeprom();
                    }
                    break;
                }
            }
            return false;
        default:
            return true; //Process all other keycodes normally
    }
}

// Initialize tap structure for spc_mbtn
static td_tap_t spc_mbtn_tap = {
    .is_press_action = true,
    .state = 0
};

// Function to determine the current tapdance state
int cur_dance (tap_dance_state_t *state) {
    if (state->count == 1) {
        return SINGLE_TAP;
    } else if (state->count == 2) {
        return DOUBLE_TAP;
    } else if (state->count == 3) {
        return TRIPLE_TAP;
    } else {
         return 3; // any number higher than the maximum state value you return above
    }
}

void spc_mbtn_finished (tap_dance_state_t *state, void *user_data) {
    spc_mbtn_tap.state = cur_dance(state);
    switch (spc_mbtn_tap.state) {
        case SINGLE_TAP:
            register_code16(KC_BTN1);
            break;
        case DOUBLE_TAP:
            register_code16(KC_BTN2);
            break;
        case TRIPLE_TAP:
            register_code16(KC_BTN3);
            break;
        default:
            break;
    }
}

void spc_mbtn_reset (tap_dance_state_t *state, void *user_data) {
    switch (spc_mbtn_tap.state) {
        case SINGLE_TAP:
            unregister_code16(KC_BTN1);
            break;
        case DOUBLE_TAP:
            unregister_code16(KC_BTN2);
            break;
        case TRIPLE_TAP:
            unregister_code16(KC_BTN3);
            break;
        default:
            break;
    }
    // Reset state
    spc_mbtn_tap.state = 0;
}

// Define tap dance actions
tap_dance_action_t tap_dance_actions[] = {
    [SPC_MBTN] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, spc_mbtn_finished, spc_mbtn_reset)
};