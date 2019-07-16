#include QMK_KEYBOARD_H
#include "process_unicode.h"
#include <print.h>
#include "rgblight.h"
#include "common.h"

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

// create a global instance of the tapdance state type
static td_state_t td_state;

// Tapdance Functions

// Function to determine the current tapdance state
int cur_dance (qk_tap_dance_state_t *state);

// `finished` and `reset` functions for each tapdance keycode
void spc_mbtn_finished (qk_tap_dance_state_t *state, void *user_data);
void spc_mbtn_reset (qk_tap_dance_state_t *state, void *user_data);

keymap_config_t keymap_config;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_BL] = LAYOUT(
        KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_DEL,  \
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_HOME, \
        KC_LGUI, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_PGUP, \
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,          KC_UP,   KC_PGDN, \
        KC_LCTL, MO(_FL), KC_LALT,                            KC_SPC,                             KC_RALT, MO(_FL), KC_LEFT, KC_DOWN, KC_RGHT  \
    ),
    [_FL] = LAYOUT(
        KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______, KC_MUTE, \
        _______, RGB_SPD, RGB_VAI, RGB_SPI, RGB_HUI, RGB_SAI, _______, KC_HOME, KC_UP,   KC_PGUP, KC_PSCR, KC_SLCK, KC_PAUS, _______, KC_END, \
        _______, RGB_RMOD,RGB_VAD, RGB_MOD, RGB_HUD, RGB_SAD, _______, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______,          _______, KC_VOLU, \
        _______, RGB_TOG, _______, _______, _______, MD_BOOT, _______, KC_END,  _______, KC_PGDN, _______, _______,          KC_PGUP, KC_VOLD, \
        _______, _______, MO(_ML),                            _______,                            _______, _______, KC_HOME, KC_PGDN, KC_END   \
    ),
    [_ML] = LAYOUT(
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_U, KC_WH_U, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_L, KC_MS_D, KC_MS_R, XXXXXXX, XXXXXXX,          XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_WH_D, XXXXXXX, XXXXXXX,          XXXXXXX, XXXXXXX, \
        XXXXXXX, XXXXXXX, _______,                       TD(SPC_MBTN),                            XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX  \
    ),
    
};

// Runs just one time when the keyboard initializes.
void matrix_init_user(void) {
    rgb_matrix_set_flags(LED_FLAG_UNDERGLOW);
    rgb_matrix_sethsv(HSV_PURPLE);
    rgb_matrix_mode(RGB_MATRIX_SOLID_REACTIVE_SIMPLE);
};

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {
};

#define MODS_SHIFT  (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT))
#define MODS_CTRL  (get_mods() & MOD_BIT(KC_LCTL) || get_mods() & MOD_BIT(KC_RCTRL))
#define MODS_ALT  (get_mods() & MOD_BIT(KC_LALT) || get_mods() & MOD_BIT(KC_RALT))

bool fn_down = false;
bool alt_down = false;

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

    if (fn_down && alt_down) {
        uint8_t keys_homeend[] = {KL_U, KL_M};
        foreach (uint8_t* key, keys_homeend) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };
        uint8_t keys_arrow[] = {KL_I, KL_J, KL_K, KL_L};
        foreach (uint8_t* key, keys_arrow) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };
        uint8_t keys_page[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_page) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };

        uint8_t keys_mouse[] = {KL_I, KL_J, KL_K, KL_L, KL_SPC};
        foreach (uint8_t* key, keys_mouse) {
            rgb_matrix_set_color(*key, KEY_COLOR_MOUSE);
        };
        uint8_t keys_mousewheel[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_mousewheel) {
            rgb_matrix_set_color(*key, KEY_COLOR_MOUSE_WHEEL);
        };
    } else if (fn_down) {
        uint8_t keys_mouse[] = {KL_I, KL_J, KL_K, KL_L, KL_SPC};
        foreach (uint8_t* key, keys_mouse) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };
        uint8_t keys_mousewheel[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_mousewheel) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };

        uint8_t keys_homeend[] = {KL_U, KL_M};
        foreach (uint8_t* key, keys_homeend) {
            rgb_matrix_set_color(*key, KEY_COLOR_HOMEEND);
        };
        uint8_t keys_arrow[] = {KL_I, KL_J, KL_K, KL_L};
        foreach (uint8_t* key, keys_arrow) {
            rgb_matrix_set_color(*key, KEY_COLOR_ARROWS);
        };
        uint8_t keys_page[] = {KL_O, KL_DOT};
        foreach (uint8_t* key, keys_page) {
            rgb_matrix_set_color(*key, KEY_COLOR_PAGEUPDN);
        };
    } else {
        uint8_t keys_all[] = {KEYBOARD_ALL};
        foreach (uint8_t* key, keys_all) {
            rgb_matrix_set_color(*key, KEY_COLOR_OFF);
        };
    }

    switch (keycode) {
        /* Massdrop debug */
        case U_T_AUTO:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_extra_manual, "USB extra port manual mode");
            }
            return false;
        case U_T_AGCR:
            if (record->event.pressed && MODS_SHIFT && MODS_CTRL) {
                TOGGLE_FLAG_AND_PRINT(usb_gcr_auto, "USB GCR auto mode");
            }
            return false;
        case DBG_TOG:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_enable, "Debug mode");
            }
            return false;
        case DBG_MTRX:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_matrix, "Debug matrix");
            }
            return false;
        case DBG_KBD:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_keyboard, "Debug keyboard");
            }
            return false;
        case DBG_MOU:
            if (record->event.pressed) {
                TOGGLE_FLAG_AND_PRINT(debug_mouse, "Debug mouse");
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

// Determine the tapdance state to return
int cur_dance (qk_tap_dance_state_t *state) {
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

// Handle the possible states for each tapdance keycode you define:

void spc_mbtn_finished (qk_tap_dance_state_t *state, void *user_data) {
    td_state = cur_dance(state);
    switch (td_state) {
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

void spc_mbtn_reset (qk_tap_dance_state_t *state, void *user_data) {
    switch (td_state) {
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
}

// Define `ACTION_TAP_DANCE_FN_ADVANCED()` for each tapdance keycode, passing in `finished` and `reset` functions
qk_tap_dance_action_t tap_dance_actions[] = {
    [SPC_MBTN] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, spc_mbtn_finished, spc_mbtn_reset)
};
