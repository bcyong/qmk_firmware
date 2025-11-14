#define foreach(item, array)                                                                                            \
    for (int keep = 1, count = 0, size = sizeof(array) / sizeof *(array); keep && count != size; keep = !keep, count++) \
        for (item = (array) + count; keep; keep = !keep)

enum keyboard_led {
    // Row 1 (top number row) - Corrected based on physical LED test
    KL_ESC = 66, KL_1 = 65, KL_2 = 64, KL_3 = 63, KL_4 = 62, KL_5 = 61, KL_6 = 60, KL_7 = 59, KL_8 = 58, KL_9 = 57, KL_0 = 56, KL_HYPHEN = 55, KL_EQL = 54, KL_BSP = 53, KL_INS = 67, KL_MUTE = 52,
    // Row 2 (QWERTY row) - Corrected based on physical LED test
    KL_TAB = 37, KL_Q = 38, KL_W = 39, KL_E = 40, KL_R = 41, KL_T = 42, KL_Y = 43, KL_U = 44, KL_I = 45, KL_O = 46, KL_P = 47, KL_LBRACKET = 48, KL_RBRACKET = 49, KL_BACKSLASH = 50, KL_DEL = 51,
    // Row 3 (ASDF row) - Corrected based on physical LED test
    KL_CPSL = 36, KL_A = 35, KL_S = 34, KL_D = 33, KL_F = 32, KL_G = 31, KL_H = 30, KL_J = 29, KL_K = 28, KL_L = 27, KL_SEMICOLON = 26, KL_GRAVE = 25, KL_RET = 24, KL_PGUP = 23,
    // Row 4 (ZXCV row) - Corrected based on physical LED test
    KL_L_SHIFT = 4, KL_Z = 5, KL_X = 6, KL_C = 7, KL_V = 8, KL_B = 9, KL_N = 10, KL_M = 11, KL_COMMA = 12, KL_DOT = 13, KL_SLASH = 14, KL_R_SHIFT = 15, KL_ARROW_UP = 16, KL_PGDN = 17,
    // Row 5 (bottom row) - Corrected based on physical LED test
    KL_L_CTRL = 3, KL_L_META = 2, KL_L_ALT = 1, KL_SPC = 0, KL_FN = 18, KL_R_CTRL = 19, KL_ARROW_LEFT = 20, KL_ARROW_DOWN = 21, KL_ARROW_RIGHT = 22,
    FIRST_LED = 0
} ;
#define LAST_LED FIRST_LED + 37

#define KEYBOARD_ALL KL_ESC, KL_1, KL_2, KL_3, KL_4, KL_5, KL_6, KL_7, KL_8, KL_9, KL_0, KL_HYPHEN, KL_EQL, KL_BSP, KL_INS, KL_TAB, KL_Q, KL_W, KL_E, KL_R, KL_T, KL_Y, KL_U, KL_I, KL_O, KL_P, KL_LBRACKET, KL_RBRACKET, KL_BACKSLASH, KL_DEL, KL_CPSL, KL_A, KL_S, KL_D, KL_F, KL_G, KL_H, KL_J, KL_K, KL_L, KL_SEMICOLON, KL_GRAVE, KL_RET, KL_PGUP, KL_L_SHIFT, KL_Z, KL_X, KL_C, KL_V, KL_B, KL_N, KL_M, KL_COMMA, KL_DOT, KL_SLASH, KL_R_SHIFT, KL_ARROW_UP, KL_PGDN, KL_L_CTRL, KL_L_META, KL_L_ALT, KL_SPC, KL_FN, KL_R_CTRL, KL_ARROW_LEFT, KL_ARROW_DOWN, KL_ARROW_RIGHT
#define KEYBOARD_LED_INNER KL_Q, KL_W, KL_E, KL_R, KL_T, KL_Y, KL_U, KL_I, KL_O, KL_P, KL_LBRACKET, KL_RBRACKET, KL_A, KL_S, KL_D, KL_F, KL_G, KL_H, KL_J, KL_K, KL_L, KL_SEMICOLON, KL_GRAVE, KL_Z, KL_X, KL_C, KL_V, KL_B, KL_N, KL_M, KL_COMMA, KL_DOT, KL_SLASH
#define KEYBOARD_ARROWS KL_U, KL_I, KL_O, KL_J, KL_K, KL_L, KL_M, KL_DOT
#define KEYBOARD_MOUSE KL_I, KL_J, KL_K, KL_L, KL_SPC
