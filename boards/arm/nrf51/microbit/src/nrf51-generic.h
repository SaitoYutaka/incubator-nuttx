/****************************************************************************
 * boards/arm/nrf51/microbit/src/nrf51-generic.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_NRF51_NRF51GENERIC_SRC_NRF51GENERIC_H
#define __BOARDS_ARM_NRF51_NRF51GENERIC_SRC_NRF51GENERIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "nrf51_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* Definitions to configure LED GPIO as outputs */

// #define GPIO_LED1  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED1_PIN))
// #define GPIO_LED2  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED2_PIN))
// #define GPIO_LED3  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED3_PIN))
// #define GPIO_LED4  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED4_PIN))

#define GPIO_LED_ROW3 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_ROW3_PIN))
#define GPIO_LED_ROW2 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_ROW2_PIN))
#define GPIO_LED_ROW1 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_ROW1_PIN))
#define GPIO_LED_COL9 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL9_PIN))
#define GPIO_LED_COL8 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL8_PIN))
#define GPIO_LED_COL7 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL7_PIN))
#define GPIO_LED_COL6 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL6_PIN))
#define GPIO_LED_COL5 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL5_PIN))
#define GPIO_LED_COL4 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL4_PIN))
#define GPIO_LED_COL3 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL3_PIN))
#define GPIO_LED_COL2 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL2_PIN))
#define GPIO_LED_COL1 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PIN(CONFIG_NRF51_GENERIC_LED_COL1_PIN))
/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define GPIO_BUTTONA (GPIO_INPUT | GPIO_PULLUP | GPIO_PIN17)
#define GPIO_BUTTONB (GPIO_INPUT | GPIO_PULLUP | GPIO_PIN26)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf51_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf51_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NRF51_NRF51GENERIC_SRC_NRF51GENERIC_H */
