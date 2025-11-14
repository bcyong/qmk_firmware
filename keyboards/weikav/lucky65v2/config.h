/* Copyright 2021 Glorious, LLC <salman@pcgamingrace.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

/* Multimode */
#ifdef MULTIMODE_ENABLE
#    define MM_USB_EN_PIN C13
#    define MM_USB_EN_STATE 0

#    define MM_BT1_NAME "Lucky65V2-1"
#    define MM_BT2_NAME "Lucky65V2-2"
#    define MM_BT3_NAME "Lucky65V2-3"

#    define MM_DONGLE_PRODUCT "Lucky65V2"


// USB模式未插入USB时跟随无线模式一致，进行超时休眠
#    define MM_USB_AUTO_SLEEP_DISABLE
#    define IM_MM_USB_TIMES 1

#    define MM_BAT_CABLE_PIN B9

#    define RGB_MATRIX_BAT_INDEX_MAP {67  , 66 , 65,  64,  63,  62,  61,  60,  59,  58}
#    define IM_BAT_REQ_LEVEL1_COLOR RGB_RED
#    define IM_BAT_REQ_LEVEL2_COLOR RGB_RED
#    define IM_BAT_REQ_LEVEL3_COLOR RGB_RED

#    define MM_SLEEP_TIMEOUT (3 * 60000) // 3 min

#    define RGB_DRIVER_EN_PIN B12
#    define IM_LKEY_COUNT 4 // 用户自定义长按按键数量

#    define RGB_MATRIX_BLINK_COUNT 13
#    define RGB_MATRIX_BLINK_INDEX_BAT 5
#    define RGB_MATRIX_BLINK_INDEX_ALL 0xFF

#    define IM_MM_RGB_BLINK_INDEX_BT1 40
#    define IM_MM_RGB_BLINK_INDEX_BT2 41
#    define IM_MM_RGB_BLINK_INDEX_BT3 42
#    define IM_MM_RGB_BLINK_INDEX_2G4 43

#    define IM_MM_LBACK_TIMEOUT (30 * 1000)
#    define IM_MM_PAIR_TIMEOUT (180 * 1000)
#    define IM_MM_SUCCEED_TIME (2 * 1000)

#    define IM_MM_LBACK_COLOR_BT1 0, 100, 150
#    define IM_MM_LBACK_COLOR_BT2 RGB_YELLOW
#    define IM_MM_LBACK_COLOR_BT3 255,0,127
#    define IM_MM_LBACK_COLOR_2G4 RGB_BLUE

#    define IM_MM_PAIR_COLOR_BT1 0, 100, 150
#    define IM_MM_PAIR_COLOR_BT2 RGB_YELLOW
#    define IM_MM_PAIR_COLOR_BT3 255,0,127
#    define IM_MM_PAIR_COLOR_2G4 RGB_BLUE

#define IM_RGB_BAT_REQ_CLOSE_ALL

#endif

/*DCDC*/
#define CHRG_PIN C7
#define FULL_PIN C6

#define POWER_DCDC_EN_PIN B13

#define BATTERY_CAPACITY_LOW 30
#define BATTERY_CAPACITY_STOP 0

#define IM_MM_RGB_BLINK_INDEX_W2M 39
#define IM_MM_RGB_BLINK_INDEX_W2H 39

#define ENCODER_MAP_KEY_DELAY 1

/* UART */
#define SERIAL_DRIVER SD3
#define SD1_TX_PIN C10
#define SD1_RX_PIN C11
#define SELECT_SOFT_SERIAL_SPEED 1

/* DEBOUNCE */
#define DEBOUNCE 8
#define HOLD_ON_OTHER_KEY_PRESS


/* SPI Config for LED Driver */
#define SPI_DRIVER SPIDQ
#define SPI_SCK_PIN A5
#define SPI_MOSI_PIN A7
#define SPI_MISO_PIN A6

/* Flash */
#define EXTERNAL_FLASH_SPI_SLAVE_SELECT_PIN B14
#define WEAR_LEVELING_BACKING_SIZE (4 * 1024)
#define WEAR_LEVELING_LOGICAL_SIZE (WEAR_LEVELING_BACKING_SIZE / 2)
#define FEE_PAGE_COUNT (WEAR_LEVELING_BACKING_SIZE / FEE_PAGE_SIZE)

/* RGB Matrix */
#define WS2812_SPI_DRIVER SPIDM2
#define WS2812_SPI_DIVISOR 16

#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
#define RGB_MATRIX_KEYPRESSES
#define RGB_TRIGGER_ON_KEYDOWN
