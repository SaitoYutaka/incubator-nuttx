/****************************************************************************
 * boards/arm/nrf51/microbit/src/nrf51_userleds.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf51-generic.h"
#include "nrf51_rtc.h"
#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  void board_userled_initialize(void);
 *  void board_userled(int led, bool ledon);
 *  void board_userled_all(uint8_t ledset);
 */

#ifdef CONFIG_NRF51_GENERIC_LED_ACTIVELOW
#define LED_ON 0
#define LED_OFF 1
#else
#define LED_ON 1
#define LED_OFF 0
#endif

static const uint32_t ascii_table[] = 
{
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // 32:Space
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0110010010111101001010010, // 65:A
    0b1110010010111001001011100, // 66:B
    0b0111010000100001000001110, // 67:C
    0b1110010010100101001011100, // 68:D
    0b1111010000111001000011110, // 69:E
    0b1111010000111001000010000, // 70:F
    0b0111010000100111000101110, // 71:G
    0b1001010010111101001010010, // 72:H
    0b1110001000010000100011100, // 73:I
    0b1111100010000101001001100, // 74:J
    0b1001010100110001010010010, // 75:K
    0b1000010000100001000011110, // 76:L
    0b1000111011101011000110001, // 77:M
    0b1000111001101011001110001, // 78:N
    0b0110010010100101001001100, // 79:O
    0b1110010010111001000010000, // 80:P
    0b0110010010100100110000110, // 81:Q
    0b1110010010111001001010001, // 82:R
    0b0111010000011000001011100, // 83:S
    0b1111100100001000010000100, // 84:T
    0b1001010010100101001001100, // 85:U
    0b1000110001100010101000100, // 86:V
    0b1000110001101011101110001, // 87:W
    0b1001010010011001001010010, // 88:X
    0b1000101010001000010000100, // 89:Y
    0b1111000100010001000011110, // 90:Z
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
    0b0000000000000000000000000, // NOT YET
};
/* This array maps an LED number to GPIO pin configuration */

static const uint32_t g_ledcfg[BOARD_NLEDS] =
{
// #if 0 < BOARD_NLEDS
//   GPIO_LED1,
// #endif
// #if 1 < BOARD_NLEDS
//   GPIO_LED2,
// #endif
// #if 2 < BOARD_NLEDS
//   GPIO_LED3,
// #endif
// #if 3 < BOARD_NLEDS
//   GPIO_LED4,
// #endif
  GPIO_LED_ROW3,
  GPIO_LED_ROW2,
  GPIO_LED_ROW1,
  GPIO_LED_COL9,
  GPIO_LED_COL8,
  GPIO_LED_COL7,
  GPIO_LED_COL6,
  GPIO_LED_COL5,
  GPIO_LED_COL4,
  GPIO_LED_COL3,
  GPIO_LED_COL2,
  GPIO_LED_COL1,
};

static uint8_t microbit_led = 0;
/****************************************************************************
 * Private Functions
 ****************************************************************************/
#define ROW3 (1<<15)
#define ROW2 (1<<14)
#define ROW1 (1<<13)
#define COL9 (1<<12)
#define COL8 (1<<11)
#define COL7 (1<<10)
#define COL6 (1<<9)
#define COL5 (1<<8)
#define COL4 (1<<7)
#define COL3 (1<<6)
#define COL2 (1<<5)
#define COL1 (1<<4)
#define ALL_LEDS (ROW3 | ROW2 | ROW1 | COL9 | COL8 | COL7 | COL6 | COL5 | COL4 | COL3 | COL2 | COL1)
#define ALL_COLS (COL9 | COL8 | COL7 | COL6 | COL5 | COL4 | COL3 | COL2 | COL1)
#define ALL_ROWS (ROW3 | ROW2 | ROW1)

int row = 0x0;
int col = 0x0;

int led_row1 = 0x0;
int led_row2 = 0x0;
int led_row3 = 0x0;

static void led_on(int led)
{
/*
        [R1/C1] [R2/C4] [R1/C2] [R2/C5] [R1/C3]
        [R3/C4] [R3/C5] [R3/C6] [R3/C7] [R3/C8]
  BTN A [R2/C2] [R1/C9] [R2/C3] [R3/C9] [R2/C1] BTN B
        [R1/C8] [R1/C7] [R1/C6] [R1/C5] [R1/C4]
        [R3/C3] [R2/C7] [R3/C1] [R2/C6] [R3/C2]
*/
  // int output = 0x0;
  // int row = 0x0;
  // int col = 0x0;
  // modifyreg32(NRF51_GPIO0_DIRSET, 0, ALL_LEDS);
  // putreg32(ALL_LEDS, NRF51_GPIO0_DIRSET);
  // 118 fe31

  // row = 0x0;
  // col = 0x0;
  // switch(sel){
  //   case 1:
  //     if(led & 0b1000000000000000000000000) { row |= ROW1; col |= COL1;} // [R1/C1]
  //     if(led & 0b0010000000000000000000000) { row |= ROW1; col |= COL2;} // [R1/C2]
  //     if(led & 0b0000100000000000000000000) { row |= ROW1; col |= COL3;} // [R1/C3]
  //     if(led & 0b0000000000010000000000000) { row |= ROW1; col |= COL9;} // [R1/C9]
  //     if(led & 0b0000000000000001000000000) { row |= ROW1; col |= COL8;} // [R1/C8]
  //     if(led & 0b0000000000000000100000000) { row |= ROW1; col |= COL7;} // [R1/C7]
  //     if(led & 0b0000000000000000010000000) { row |= ROW1; col |= COL6;} // [R1/C6]
  //     if(led & 0b0000000000000000001000000) { row |= ROW1; col |= COL5;} // [R1/C5]
  //     if(led & 0b0000000000000000000100000) { row |= ROW1; col |= COL4;} // [R1/C4]
  //     break;
  //   case 2:
  //     if(led & 0b0100000000000000000000000) { row |= ROW2; col |= COL4;} // [R2/C4]
  //     if(led & 0b0001000000000000000000000) { row |= ROW2; col |= COL5;} // [R2/C5]
  //     if(led & 0b0000000000100000000000000) { row |= ROW2; col |= COL2;} // [R2/C2]
  //     if(led & 0b0000000000001000000000000) { row |= ROW2; col |= COL3;} // [R2/C3]
  //     if(led & 0b0000000000000010000000000) { row |= ROW2; col |= COL1;} // [R2/C1]
  //     if(led & 0b0000000000000000000001000) { row |= ROW2; col |= COL7;} // [R2/C7]
  //     if(led & 0b0000000000000000000000010) { row |= ROW2; col |= COL6;} // [R2/C6]
  //     break;
  //   case 3:
  //     if(led & 0b0000010000000000000000000) { row |= ROW3; col |= COL4;} // [R3/C4]
  //     if(led & 0b0000001000000000000000000) { row |= ROW3; col |= COL5;} // [R3/C5]
  //     if(led & 0b0000000100000000000000000) { row |= ROW3; col |= COL6;} // [R3/C6]
  //     if(led & 0b0000000010000000000000000) { row |= ROW3; col |= COL7;} // [R3/C7]
  //     if(led & 0b0000000001000000000000000) { row |= ROW3; col |= COL8;} // [R3/C8]
  //     if(led & 0b0000000000000100000000000) { row |= ROW3; col |= COL9;} // [R3/C9]
  //     if(led & 0b0000000000000000000010000) { row |= ROW3; col |= COL3;} // [R3/C3]
  //     if(led & 0b0000000000000000000000100) { row |= ROW3; col |= COL1;} // [R3/C1]
  //     if(led & 0b0000000000000000000000001) { row |= ROW3; col |= COL2;} // [R3/C2]
  //     break;
  // }

  if(led & 0b1000000000000000000000000) { row |= ROW1; led_row1 |= COL1;} // [R1/C1]
  if(led & 0b0100000000000000000000000) { row |= ROW2; led_row2 |= COL4;} // [R2/C4]
  if(led & 0b0010000000000000000000000) { row |= ROW1; led_row1 |= COL2;} // [R1/C2]
  if(led & 0b0001000000000000000000000) { row |= ROW2; led_row2 |= COL5;} // [R2/C5]
  if(led & 0b0000100000000000000000000) { row |= ROW1; led_row1 |= COL3;} // [R1/C3]

  if(led & 0b0000010000000000000000000) { row |= ROW3; led_row3 |= COL4;} // [R3/C4]
  if(led & 0b0000001000000000000000000) { row |= ROW3; led_row3 |= COL5;} // [R3/C5]
  if(led & 0b0000000100000000000000000) { row |= ROW3; led_row3 |= COL6;} // [R3/C6]
  if(led & 0b0000000010000000000000000) { row |= ROW3; led_row3 |= COL7;} // [R3/C7]
  if(led & 0b0000000001000000000000000) { row |= ROW3; led_row3 |= COL8;} // [R3/C8]

  if(led & 0b0000000000100000000000000) { row |= ROW2; led_row2 |= COL2;} // [R2/C2]
  if(led & 0b0000000000010000000000000) { row |= ROW1; led_row1 |= COL9;} // [R1/C9]
  if(led & 0b0000000000001000000000000) { row |= ROW2; led_row2 |= COL3;} // [R2/C3]
  if(led & 0b0000000000000100000000000) { row |= ROW3; led_row3 |= COL9;} // [R3/C9]
  if(led & 0b0000000000000010000000000) { row |= ROW2; led_row2 |= COL1;} // [R2/C1]

  if(led & 0b0000000000000001000000000) { row |= ROW1; led_row1 |= COL8;} // [R1/C8]
  if(led & 0b0000000000000000100000000) { row |= ROW1; led_row1 |= COL7;} // [R1/C7]
  if(led & 0b0000000000000000010000000) { row |= ROW1; led_row1 |= COL6;} // [R1/C6]
  if(led & 0b0000000000000000001000000) { row |= ROW1; led_row1 |= COL5;} // [R1/C5]
  if(led & 0b0000000000000000000100000) { row |= ROW1; led_row1 |= COL4;} // [R1/C4]

  if(led & 0b0000000000000000000010000) { row |= ROW3; led_row3 |= COL3;} // [R3/C3]
  if(led & 0b0000000000000000000001000) { row |= ROW2; led_row2 |= COL7;} // [R2/C7]
  if(led & 0b0000000000000000000000100) { row |= ROW3; led_row3 |= COL1;} // [R3/C1]
  if(led & 0b0000000000000000000000010) { row |= ROW2; led_row2 |= COL6;} // [R2/C6]
  if(led & 0b0000000000000000000000001) { row |= ROW3; led_row3 |= COL2;} // [R3/C2]

///////
  // if(led & 0x00000001) { row |= ROW1; col |= COL1;} // [R1/C1]
  // if(led & 0x00000002) { row |= ROW2; col |= COL4;} // [R2/C4]
  // if(led & 0x00000004) { row |= ROW1; col |= COL2;} // [R1/C2]
  // if(led & 0x00000008) { row |= ROW2; col |= COL5;} // [R2/C5]
  // if(led & 0x00000010) { row |= ROW1; col |= COL3;} // [R1/C3]

  // if(led & 0x00000020) { row |= ROW3; col |= COL4;} // [R3/C4]
  // if(led & 0x00000040) { row |= ROW3; col |= COL5;} // [R3/C5]
  // if(led & 0x00000080) { row |= ROW3; col |= COL6;} // [R3/C6]
  // if(led & 0x00000100) { row |= ROW3; col |= COL7;} // [R3/C7]
  // if(led & 0x00000200) { row |= ROW3; col |= COL8;} // [R3/C8]

  // if(led & 0x00000400) { row |= ROW2; col |= COL2;} // [R2/C2]
  // if(led & 0x00000800) { row |= ROW1; col |= COL9;} // [R1/C9]
  // if(led & 0x00001000) { row |= ROW2; col |= COL3;} // [R2/C3]
  // if(led & 0x00002000) { row |= ROW3; col |= COL9;} // [R3/C9]
  // if(led & 0x00004000) { row |= ROW2; col |= COL1;} // [R2/C1]

  // if(led & 0x00008000) { row |= ROW1; col |= COL8;} // [R1/C8]
  // if(led & 0x00010000) { row |= ROW1; col |= COL7;} // [R1/C7]
  // if(led & 0x00020000) { row |= ROW1; col |= COL6;} // [R1/C6]
  // if(led & 0x00040000) { row |= ROW1; col |= COL5;} // [R1/C5]
  // if(led & 0x00080000) { row |= ROW1; col |= COL4;} // [R1/C4]

  // if(led & 0x00100000) { row |= ROW3; col |= COL3;} // [R3/C3]
  // if(led & 0x00200000) { row |= ROW2; col |= COL7;} // [R2/C7]
  // if(led & 0x00400000) { row |= ROW3; col |= COL1;} // [R3/C1]
  // if(led & 0x00800000) { row |= ROW2; col |= COL6;} // [R2/C6]
  // if(led & 0x01000000) { row |= ROW3; col |= COL2;} // [R3/C2]

  // ledinfo("row %x col %x\n", row, col);
  // ledinfo("NRF51_GPIO0_DIRSET %x\n", getreg32(NRF51_GPIO0_DIRSET));
  // ledinfo("NRF51_GPIO0_OUTCLR %x\n", getreg32(NRF51_GPIO0_OUTCLR));
  // ledinfo("NRF51_GPIO0_OUTSET %x\n", getreg32(NRF51_GPIO0_OUTSET));

  // modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
  // modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
  // modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, row | col);
  // modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, col);
  // modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, row); // ON

  // putreg32(((getreg32(NRF51_GPIO0_DIRSET) & ~ALL_LEDS) | (row | col)), NRF51_GPIO0_DIRSET);
  // putreg32(((getreg32(NRF51_GPIO0_OUTCLR) & ~ALL_LEDS) | col), NRF51_GPIO0_OUTCLR);
  // putreg32(((getreg32(NRF51_GPIO0_OUTSET) & ~ALL_LEDS) | row), NRF51_GPIO0_OUTSET);
  // putreg32(row | col, NRF51_GPIO0_DIRSET);
  // putreg32(col, NRF51_GPIO0_OUTCLR);
  // putreg32(row, NRF51_GPIO0_OUTSET); // ON
  // putreg32((1<<ROW1)|(1<<ROW2)|(1<<COL1)|(1<<COL4)|(1<<COL2)|(1<<COL5)|(1<<COL3), NRF51_GPIO0_DIRSET);
  // putreg32((1<<COL1)|(1<<COL4)|(1<<COL2)|(1<<COL5)|(1<<COL3), NRF51_GPIO0_OUTCLR);
  // putreg32((1<<ROW1)|(1<<ROW2), NRF51_GPIO0_OUTSET); // ON
  return;
}

static void microbit_display(
  char led01, char led02, char led03, char led04, char led05, 
  char led06, char led07, char led08, char led09, char led10, 
  char led11, char led12, char led13, char led14, char led15, 
  char led16, char led17, char led18, char led19, char led20, 
  char led21, char led22, char led23, char led24, char led25 
   )
{
  int output = 0x0;
  if(led01 == 1){ output |= 0x0000001; }
  if(led02 == 1){ output |= 0x0000002; }
  if(led03 == 1){ output |= 0x0000004; }
  if(led04 == 1){ output |= 0x0000008; }
  if(led05 == 1){ output |= 0x0000010; }
  if(led06 == 1){ output |= 0x0000020; }
  if(led07 == 1){ output |= 0x0000040; }
  if(led08 == 1){ output |= 0x0000080; }
  if(led09 == 1){ output |= 0x0000100; }
  if(led10 == 1){ output |= 0x0000200; }
  if(led11 == 1){ output |= 0x0000400; }
  if(led12 == 1){ output |= 0x0000800; }
  if(led13 == 1){ output |= 0x0001000; }
  if(led14 == 1){ output |= 0x0002000; }
  if(led15 == 1){ output |= 0x0004000; }
  if(led16 == 1){ output |= 0x0008000; }
  if(led17 == 1){ output |= 0x0010000; }
  if(led18 == 1){ output |= 0x0020000; }
  if(led19 == 1){ output |= 0x0040000; }
  if(led20 == 1){ output |= 0x0080000; }
  if(led21 == 1){ output |= 0x0100000; }
  if(led22 == 1){ output |= 0x0200000; }
  if(led23 == 1){ output |= 0x0400000; }
  if(led24 == 1){ output |= 0x0800000; }
  if(led25 == 1){ output |= 0x1000000; }
  ledinfo("output %x\n", output);
  // led_on(output);
  return;
}

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(FAR const char *msg)
{
  nrf51_pin_dump(PINCONFIG_LED, msg);
  nrf51_gpio_dump(GPIO_LED, msg);
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/
static uint8_t microbit_cnt = 1;
static uint8_t microbit_switch = 0;
static int nrf51_microbitled(int irq, uint32_t *regs, void *arg)
{
  if(getreg32(NRF51_RTC0_TICK)){
    putreg32(0x0, NRF51_RTC0_TICK);
    putreg32(NRF51_RTC0_BIT_TICK, NRF51_RTC0_EVTENCLR); // clear
    // nxsched_process_timer();
    microbit_cnt++;
    // if(microbit_cnt % 20 == 0){
    if(1){
      ledinfo("nrf51_microbitled\n");
      microbit_cnt = 1;
      led_on(ascii_table[65]);


      switch(microbit_switch){
        case 0:
          microbit_switch = 1;
          // led_on(ascii_table[66], 1);
          modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW1 | led_row1);
          modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row1);
          modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW1); // ON
          break;
        case 1:
          microbit_switch = 2;
          // led_on(ascii_table[66], 2);
          modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW2 | led_row2);
          modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row2);
          modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW2); // ON
          break;
        case 2:
          microbit_switch = 0;
          // led_on(ascii_table[66], 3);
          modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
          modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW3 | led_row3);
          modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row3);
          modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW3); // ON
          break;
        default:
          break;
      }

      // switch(microbit_switch){
      //   case 0:
      //     microbit_switch = 1;
      //     microbit_display(
      //       1,0,1,0,1,
      //       0,0,0,0,0,
      //       0,1,0,0,0,
      //       1,1,1,1,1,
      //       0,0,0,0,0
      //     );
      //     break;
      //   case 1:
      //     microbit_switch = 2;
      //     microbit_display(
      //       0,0,0,0,0,
      //       1,1,1,1,1,
      //       0,0,0,1,0,
      //       0,0,0,0,0,
      //       1,0,1,0,1
      //     );
      //     break;
      //   case 2:
      //     microbit_switch = 0;
      //     microbit_display(
      //       0,1,0,1,0,
      //       0,0,0,0,0,
      //       1,0,1,0,1,
      //       0,0,0,0,0,
      //       0,1,0,1,0
      //     );
      //     break;
      //   default:
      //     break;
      // }
    }
  }
  return 0;
}

void board_userled_initialize(void)
{
  // int i;

  // /* Configure LED pin as a GPIO outputs */

  // led_dumppins("board_userled_initialize() Entry)");

  // /* Configure GPIO as an outputs */

  // for (i = 0; i < BOARD_NLEDS; i++)
  //   {
  //     nrf51_gpio_config(g_ledcfg[i]);
  //   }

  // led_dumppins("board_userled_initialize() Exit");
  (void)irq_attach(NRF51_IRQ_RTC0, (xcpt_t)nrf51_microbitled, NULL);
  up_enable_irq(NRF51_IRQ_RTC0);

  // putreg32(0xFFF, NRF51_RTC0_PRESCALER); // PRESCALER
  putreg32(60, NRF51_RTC0_PRESCALER); // PRESCALER

  putreg32(NRF51_RTC0_BIT_TICK, NRF51_RTC0_INTENCLR); // INTENCLR
  putreg32(NRF51_RTC0_BIT_TICK, NRF51_RTC0_INTEN); // EVTENSET
  putreg32(NRF51_RTC0_BIT_TICK, NRF51_RTC0_INTENSET); // EVTENSET
  putreg32(1, NRF51_RTC0_START); // START
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      nrf51_gpio_write(g_ledcfg[led], ledon ? LED_ON : LED_OFF);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/


void board_userled_all(uint8_t ledset)
{
  // int i;

  /* Configure LED1-8 GPIOs for output */
  ledinfo("ledset %x\n", ledset);
  microbit_led = ledset;
  // for (i = 0; i < BOARD_NLEDS; i++)
  //   {
  //     nrf51_gpio_write(g_ledcfg[i], (ledset & (1 << i)) ? LED_ON : LED_OFF);
  //   }
  // if(ledset == 0) return ;

}

#endif /* !CONFIG_ARCH_LEDS */
