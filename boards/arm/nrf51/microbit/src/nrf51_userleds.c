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
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf51-generic.h"
#include "nrf51_rtc.h"
#include <nuttx/leds/microbit_led.h>
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

// See https://github.com/SaitoYutaka/microbit_image_generator
static const uint32_t ascii_table[] = 
{
    0b0000000000000000000000000, // blank
    0b0101000000001001000101110, // 
    0b0101000000000000111010001, // 
    0b1101111111111110111000100, // 
    0b0000000000000000000000000, // blank
    0b0100010100010001110000000, // 
    0b0010001110011100010001110, // 
    0b1110011100111000000000000, // 
    0b0000000000000000000000000, // blank
    0b0100010100010000000000000, // 
    0b0000000000000000000000000, // blank
    0b0111100111111111010111100, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b1111001010010100101010100, // 
    0b1010101110110110111010101, // 
    0b0010000110001110011000100, // 
    0b0010001100111000110000100, // 
    0b0010001110001000111000100, // 
    0b0101001010010100000001010, // 
    0b0011011010010100101001010, // 
    0b0000000000000000000000000, // 
    0b1111111111000000000000000, // 
    0b0000000000000000000000000, // blank
    0b0010001110101010010000100, // 
    0b0010000100101010111000100, // 
    0b0010000010111110001000100, // 
    0b0010001000111110100000100, // 
    0b1000010000111100000000000, // 
    0b0000000000000000000000000, // blank
    0b0010001110111110000000000, // 
    0b1111101110001000000000000, // 
    0b0000000000000000000000000, // 
    0b0010000100001000000000100, //  !
    0b1010010100000000000000000, //  "
    0b0101011111010101111101010, //  #
    0b1111110100111110010111111, //  $
    0b1000100010001000100010001, //  %
    0b0110010010011011001001101, //  &
    0b0100010000000000000000000, //  '
    0b0001000100001000010000010, //  (
    0b0100000100001000010001000, //  )
    0b1010101110111110111010101, //  *
    0b0100011100010000000000000, //  +
    0b0100010000000000000000000, //  
    0b1110000000000000000000000, //  -
    0b1000000000000000000000000, //  .
    0b0001000100001000010001000, //  /
    0b0111010011101011100101110, //  0
    0b0010001100001000010001110, //  1
    0b0111000010011100100001110, //  2
    0b0111000010001100001001110, //  3
    0b0100001000010100111000010, //  4
    0b0111001000011100001001110, //  5
    0b0110001000011100101001110, //  6
    0b0111000010001100001000010, //  7
    0b0111001010011100101001110, //  8
    0b0111001010011100001001110, //  9
    0b1000000000100000000000000, //  :
    0b0100000000010001000000000, //  ;
    0b0001000100010000010000010, //  <
    0b1110000000111000000000000, //  =
    0b0100000100000100010001000, //  >
    0b0111010001001100000000100, //  ?
    0b0111010010101001000001110, //  @
    0b0110010010100011111110001, //  A
    0b1110010010111101000111110, //  B
    0b0111010001100001000101110, //  C
    0b1111010001100011000111110, //  D
    0b1111110000111101000011111, //  E
    0b1111110000111101000010000, //  F
    0b0111010000100111000101110, //  G
    0b1000110001111111000110001, //  H
    0b0111000100001000010001110, //  I
    0b0011000010000100101001110, //  J
    0b1001010100110001010010010, //  K
    0b1000010000100001000011110, //  L
    0b1000111011101011000110001, //  M
    0b1000111001101011001110001, //  N
    0b0111010001100011000101110, //  O
    0b1110010010111001000010000, //  P
    0b0111010001100011001101111, //  Q
    0b1110010010111001010010010, //  R
    0b0111110000011100000111110, //  S
    0b1111100100001000010000100, //  T
    0b1000110001100011000101110, //  U
    0b0101001010010100101000100, //  V
    0b1000110001101011010101010, //  W
    0b1000101010001000101010001, //  X
    0b1000101010001000010000100, //  Y
    0b1111100010001000100011111, //  Z
    0b0011000100001000010000110, //  [
    0b0100000100001000010000010, //  "\"
    0b0110000100001000010001100, //  ]
    0b0100010100000000000000000, //  ^
    0b1110000000000000000000000, //  _
    0b1000001000000000000000000, //  `
    0b0110010010100011111110001, //  a
    0b1110010010111101000111110, //  b
    0b0111010001100001000101110, //  c
    0b1111010001100011000111110, //  d
    0b1111110000111101000011111, //  e
    0b1111110000111101000010000, //  f
    0b0111010000100111000101110, //  g
    0b1000110001111111000110001, //  h
    0b0111000100001000010001110, //  i
    0b0011000010000100101001110, //  j
    0b0100101010011000101001001, //  k
    0b1000010000100001000011110, //  l
    0b1000111011101011000110001, //  m
    0b1000111001101011001110001, //  n
    0b0111010001100011000101110, //  o
    0b1110010010111001000010000, //  p
    0b0111010001100011001101111, //  q
    0b1110010010111001010010010, //  r
    0b0111110000011100000111110, //  s
    0b1111100100001000010000100, //  t
    0b1000110001100011000101110, //  u
    0b0101001010010100101000100, //  v
    0b1000110001101011010101010, //  w
    0b1000101010001000101010001, //  x
    0b1000101010001000010000100, //  y
    0b1111100010001000100011111, //  z
    0b0011000100010000010000110, //  {
    0b0010000100001000010000100, //  |
    0b0110000100000100010001100, //  }
    0b0101010100000000000000000, //  ~
    0b0100010100101001110000000, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0010000000001000010000100, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0010001001100100100100100, // 
    0b1110000100001000000000000, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0010001110001000000001110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b1000000000000000000000000, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0010010010010011001000100, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0010000000011001000101110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000011100101001111, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000011100101001110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000010100101001110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0110010010101101000110110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000011100101001111, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000011100101001110, // 
    0b0010000000011100000000100, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0101000000010100101001110, // 
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
    0b0000000000000000000000000, // blank
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

uint32_t static led_row1 = 0x0;
uint32_t static led_row2 = 0x0;
uint32_t static led_row3 = 0x0;

uint32_t static scroll[9];
char g_str_led[30];
uint32_t g_str_len;
scroll_dir_e g_str_dir;
uint32_t g_str_led_index = 0xfffffff;
uint32_t i = 0;

static void save_str_info(struct ledinfo_s * ledsinfo){
  memset(g_str_led, ' ', sizeof(g_str_led));
  memcpy(g_str_led+1, ledsinfo->ledset, strlen(ledsinfo->ledset));
  g_str_len = strlen(ledsinfo->ledset);
  g_str_dir = ledsinfo->dir;
}

static void make_animation(uint32_t index){

  if(g_str_led_index == index){
    return;
  }
  g_str_led_index = index;
  i = g_str_led[index];
  switch(g_str_dir){
    case SCROLL_H:
      scroll[0] = ((ascii_table[i] & 0b1000010000100001000010000) >> 4);
      scroll[1] = ((ascii_table[i] & 0b1100011000110001100011000) >> 3);
      scroll[2] = ((ascii_table[i] & 0b1110011100111001110011100) >> 2);
      scroll[3] = ((ascii_table[i] & 0b1111011110111101111011110) >> 1);
      scroll[4] = ascii_table[i]; // ((led & 0b1111111111111111111111111) >> 0);
      scroll[5] = ((ascii_table[i] & 0b0111101111011110111101111) << 1);
      scroll[6] = ((ascii_table[i] & 0b0011100111001110011100111) << 2);
      scroll[7] = ((ascii_table[i] & 0b0001100011000110001100011) << 3);
      scroll[8] = ((ascii_table[i] & 0b0000100001000010000100001) << 4);
      break;
    case SCROLL_V:
    default:
      scroll[0] = (ascii_table[i] >> 20);
      scroll[1] = (ascii_table[i] >> 15);
      scroll[2] = (ascii_table[i] >> 10);
      scroll[3] = (ascii_table[i] >> 5);
      scroll[4] = ascii_table[i]; // ((led & 0b1111111111111111111111111) >> 0);
      scroll[5] = (ascii_table[i] << 5);
      scroll[6] = (ascii_table[i] << 10);
      scroll[7] = (ascii_table[i] << 15);
      scroll[8] = (ascii_table[i] << 20);
      break;
  }

  return;
}


static void led_on(int led)
{
/*
        [R1/C1] [R2/C4] [R1/C2] [R2/C5] [R1/C3]
        [R3/C4] [R3/C5] [R3/C6] [R3/C7] [R3/C8]
  BTN A [R2/C2] [R1/C9] [R2/C3] [R3/C9] [R2/C1] BTN B
        [R1/C8] [R1/C7] [R1/C6] [R1/C5] [R1/C4]
        [R3/C3] [R2/C7] [R3/C1] [R2/C6] [R3/C2]
*/
  led_row1 = 0x0;
  led_row2 = 0x0;
  led_row3 = 0x0;
  if(led & 0b1000000000000000000000000) { led_row1 |= COL1;} // [R1/C1]
  if(led & 0b0100000000000000000000000) { led_row2 |= COL4;} // [R2/C4]
  if(led & 0b0010000000000000000000000) { led_row1 |= COL2;} // [R1/C2]
  if(led & 0b0001000000000000000000000) { led_row2 |= COL5;} // [R2/C5]
  if(led & 0b0000100000000000000000000) { led_row1 |= COL3;} // [R1/C3]
  if(led & 0b0000010000000000000000000) { led_row3 |= COL4;} // [R3/C4]
  if(led & 0b0000001000000000000000000) { led_row3 |= COL5;} // [R3/C5]
  if(led & 0b0000000100000000000000000) { led_row3 |= COL6;} // [R3/C6]
  if(led & 0b0000000010000000000000000) { led_row3 |= COL7;} // [R3/C7]
  if(led & 0b0000000001000000000000000) { led_row3 |= COL8;} // [R3/C8]
  if(led & 0b0000000000100000000000000) { led_row2 |= COL2;} // [R2/C2]
  if(led & 0b0000000000010000000000000) { led_row1 |= COL9;} // [R1/C9]
  if(led & 0b0000000000001000000000000) { led_row2 |= COL3;} // [R2/C3]
  if(led & 0b0000000000000100000000000) { led_row3 |= COL9;} // [R3/C9]
  if(led & 0b0000000000000010000000000) { led_row2 |= COL1;} // [R2/C1]
  if(led & 0b0000000000000001000000000) { led_row1 |= COL8;} // [R1/C8]
  if(led & 0b0000000000000000100000000) { led_row1 |= COL7;} // [R1/C7]
  if(led & 0b0000000000000000010000000) { led_row1 |= COL6;} // [R1/C6]
  if(led & 0b0000000000000000001000000) { led_row1 |= COL5;} // [R1/C5]
  if(led & 0b0000000000000000000100000) { led_row1 |= COL4;} // [R1/C4]
  if(led & 0b0000000000000000000010000) { led_row3 |= COL3;} // [R3/C3]
  if(led & 0b0000000000000000000001000) { led_row2 |= COL7;} // [R2/C7]
  if(led & 0b0000000000000000000000100) { led_row3 |= COL1;} // [R3/C1]
  if(led & 0b0000000000000000000000010) { led_row2 |= COL6;} // [R2/C6]
  if(led & 0b0000000000000000000000001) { led_row3 |= COL2;} // [R3/C2]
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
static uint32_t microbit_cnt = 0;
static uint8_t microbit_switch = 0;
static uint8_t microbit_index = 0;
static uint8_t is_animation = 0;
uint8_t led_index = 0;
static int nrf51_microbitled(int irq, uint32_t *regs, void *arg)
{
  if(getreg32(NRF51_RTC0_TICK)){
    putreg32(0x0, NRF51_RTC0_TICK);
    putreg32(NRF51_RTC0_BIT_TICK, NRF51_RTC0_EVTENCLR); // clear
    microbit_cnt++;

  if(is_animation == 1){
    make_animation(led_index);
    if((microbit_cnt % 0x30) == 0){
      led_on(scroll[microbit_index++]);
      if(microbit_index > 8) {
        microbit_index = 0;
        // is_animation = 0;
        led_index++;
        if(led_index > g_str_len){
          led_index = 0;
        }
      }
    }
  }
#if 0
    make_animation(ascii_table['A']);

    if((microbit_cnt % 0x111) == 0){
      led_on(scroll[microbit_index++]);
      if(microbit_index > 9) {
        microbit_index = 0;
      }
    }
#endif
    switch(microbit_switch){
      case 0:
        microbit_switch = 1;
        modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW1 | led_row1);
        modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row1);
        modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW1);
        break;
      case 1:
        microbit_switch = 2;
        modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW2 | led_row2);
        modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row2);
        modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW2);
        break;
      case 2:
        microbit_switch = 0;
        modifyreg32(NRF51_GPIO0_DIRCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_OUTCLR, 0, ALL_LEDS);
        modifyreg32(NRF51_GPIO0_DIRSET, ALL_LEDS, ROW3 | led_row3);
        modifyreg32(NRF51_GPIO0_OUTCLR, ALL_COLS, led_row3);
        modifyreg32(NRF51_GPIO0_OUTSET, ALL_ROWS, ROW3);
        break;
      default:
        break;
    }

  }
  return 0;
}

void board_userled_initialize(void)
{
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
  ledinfo("ledset %x\n", ledset);
  led_on(ascii_table[ledset]);
}

void board_scrollchar(struct ledinfo_s * ledsinfo)
{
  is_animation = 1;
  save_str_info(ledsinfo);
  ledinfo("ledset %s len %d\n", ledsinfo->ledset, strlen(ledsinfo->ledset));
  return;
}

#endif /* !CONFIG_ARCH_LEDS */
