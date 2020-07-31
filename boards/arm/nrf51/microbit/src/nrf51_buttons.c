/****************************************************************************
 * boards/arm/nrf51/microbit/src/nrf51_buttons.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nrf51_gpio.h"

#include "nrf51-generic.h"
#include "up_arch.h"
#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each PCA10040 button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BUTTONA,
  GPIO_BUTTONB,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  int i;
  iinfo("board_button_initialize\n");
  /* Configure the GPIO pins as inputs. */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      nrf51_gpio_config(g_buttons[i]);
    }
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  iinfo("board_buttons\n");
  /* Check that state of each key */
  if(getreg32(NRF51_GPIOTE_IN0)){
    if((getreg32(NRF51_GPIO0_IN) & 0x20000) == 0) {
      ret |= BUTTON_BTNA_BIT;
    }
    // gpioinfo("NGPIO_BUTTONA %x\n", getreg32(NRF51_GPIO0_IN) & 0x20000);
    putreg32(0, NRF51_GPIOTE_IN0);
    putreg32(1, NRF51_GPIOTE_IN0);
  }

  if(getreg32(NRF51_GPIOTE_IN1)){
    if((getreg32(NRF51_GPIO0_IN) & 0x4000000) == 0) {
      ret |= BUTTON_BTNB_BIT;
    }
    // gpioinfo("NGPIO_BUTTONB %x\n", getreg32(NRF51_GPIO0_IN) & 0x4000000);
    putreg32(0, NRF51_GPIOTE_IN1);
    putreg32(1, NRF51_GPIOTE_IN1);
  }
  // if (!nrf51_gpio_read(g_buttons[BUTTON_BTNA]))
  //   {
  //     ret |= BUTTON_BTNA_BIT;
  //   }

  // if (!nrf51_gpio_read(g_buttons[BUTTON_BTNB]))
  //   {
  //     ret |= BUTTON_BTNB_BIT;
  //   }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
//   int ret = -ENOSYS;

// #warning Missing Implementation!
  iinfo("board_button_irq\n");
  putreg32(NRF51_GPIOTE_TASKS_ALL, NRF51_GPIOTE_INTENCLR);

  putreg32((NRF51_GPIOTE_CONF_POLARITY(NRF51_GPIOTE_CONF_POLARITY_Toggle)) | (NRF51_GPIOTE_CONF_SEL0(17)) | NRF51_GPIOTE_CONF_MODE_EVENT, NRF51_GPIOTE_CONFIG0);
  putreg32((NRF51_GPIOTE_CONF_POLARITY(NRF51_GPIOTE_CONF_POLARITY_Toggle)) | (NRF51_GPIOTE_CONF_SEL0(26)) | NRF51_GPIOTE_CONF_MODE_EVENT, NRF51_GPIOTE_CONFIG1);

  putreg32(NRF51_GPIOTE_TASKS_IN0 | NRF51_GPIOTE_TASKS_IN1, NRF51_GPIOTE_INTEN);
  putreg32(NRF51_GPIOTE_TASKS_IN0 | NRF51_GPIOTE_TASKS_IN1, NRF51_GPIOTE_INTENSET);

  putreg32(1, NRF51_GPIOTE_IN0);
  putreg32(1, NRF51_GPIOTE_IN1);
  (void)irq_attach(NRF51_IRQ_GPIOTE, irqhandler, NULL);
  up_enable_irq(NRF51_IRQ_GPIOTE);
  return 1;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
