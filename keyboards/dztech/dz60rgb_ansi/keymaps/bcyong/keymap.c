#include QMK_KEYBOARD_H
#include "process_unicode.h"
#include <print.h>

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
    [_BL] = LAYOUT_60_ansi(
        KC_GESC, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS,
        KC_LGUI, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,
        KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,          KC_RSFT,
        KC_LCTL, MO(_FL), KC_LALT,                            KC_SPC,                             KC_RALT, KC_RGUI, MO(2),   KC_RCTL
    ),
    [_FL] = LAYOUT_60_ansi(
        KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_DEL,
        _______, _______, _______, _______, _______, _______, KC_CALC, KC_HOME, KC_UP,   KC_PGUP, KC_PSCR, KC_SLCK, KC_PAUS, QK_BOOT,
        _______, _______, _______, _______, _______, _______, _______, KC_LEFT, KC_DOWN, KC_RGHT, _______, _______,          _______,
        KC_MPRV,          KC_VOLD, KC_VOLU, KC_MUTE, _______, _______, KC_END, _______,  KC_PGDN, _______, _______,          KC_MNXT,
        _______, _______, _______,                            _______,                            _______, _______, _______, _______
    ),
    [_ML] = LAYOUT_60_ansi(
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_U, KC_WH_U, XXXXXXX, XXXXXXX, XXXXXXX, QK_BOOT,
        XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_MS_L, KC_MS_D, KC_MS_R, XXXXXXX, XXXXXXX,          XXXXXXX,
        XXXXXXX,          XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_WH_D, XXXXXXX, XXXXXXX,          XXXXXXX,
        XXXXXXX, XXXXXXX, XXXXXXX,                            TD(SPC_MBTN),                       XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX
    )
};

#define MODS_SHIFT  (get_mods() & MOD_BIT(KC_LSHIFT) || get_mods() & MOD_BIT(KC_RSHIFT))
#define MODS_CTRL  (get_mods() & MOD_BIT(KC_LCTL) || get_mods() & MOD_BIT(KC_RCTRL))
#define MODS_ALT  (get_mods() & MOD_BIT(KC_LALT) || get_mods() & MOD_BIT(KC_RALT))

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
