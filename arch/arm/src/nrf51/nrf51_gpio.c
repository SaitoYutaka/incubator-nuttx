/****************************************************************************
 * arch/arm/src/nrf51/nrf51_gpio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "hardware/nrf51_gpio.h"
#include "nrf51_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf51_gpio_input
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf51_gpio_input(unsigned int port, unsigned int pin)
{
  /* Set as input */

  if (port == 0)
    {
      putreg32(1U << pin, NRF51_GPIO0_DIRCLR);
    }
}

/****************************************************************************
 * Name: nrf51_gpio_output
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf51_gpio_output(nrf51_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  nrf51_gpio_write(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  /* Configure the pin as an output */

  if (port == 0)
    {
      putreg32(1U << pin, NRF51_GPIO0_DIRSET);
    }
}

/****************************************************************************
 * Name: nrf51_gpio_mode
 *
 * Description:
 *   Configure a GPIO mode based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf51_gpio_mode(nrf51_pinset_t cfgset,
                                   unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;

  mode = cfgset & GPIO_MODE_MASK;
  regval = getreg32(NRF51_GPIO0_CNF(pin));

  regval &= NRF51_GPIO_CNF_PULL_MASK;

  if (mode == GPIO_PULLUP)
    {
      regval &= NRF51_GPIO_CNF_PULL_MASK;
      regval |= NRF51_GPIO_CNF_PULL_UP;
    }
  else if (mode == GPIO_PULLDOWN)
    {
      regval &= NRF51_GPIO_CNF_PULL_MASK;
      regval |= NRF51_GPIO_CNF_PULL_DOWN;
    }

  putreg32(regval, NRF51_GPIO0_CNF(pin));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf51_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf51_gpio_config(nrf51_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  // gpioinfo("port %d\n", port);
  if (port < NRF51_GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for
       * that pin.
       */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* First, configure the port as a generic input so that we have a
       * known starting point and consistent behavior during the re-
       * configuration.
       */

      nrf51_gpio_input(port, pin);

      /* Set the mode bits */

      nrf51_gpio_mode(cfgset, port, pin);

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          // gpioinfo("GPIO_INPUT\n");
          break;           /* Already configured */

// #ifdef CONFIG_NRF51_GPIOIRQ
//         case GPIO_INTFE:   /* GPIO interrupt falling edge */
//         case GPIO_INTRE:   /* GPIO interrupt rising edge */
//         case GPIO_INTBOTH: /* GPIO interrupt both edges */
//         case GPIO_INTLOW:  /* GPIO interrupt low level */
//         case GPIO_INTHIGH: /* GPIO interrupt high level */
//           nrf51_gpio_interrupt(cfgset);
//           break;
// #endif

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          nrf51_gpio_output(cfgset, port, pin);
          break;

        default:
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf51_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nrf51_gpio_write(nrf51_pinset_t pinset, bool value)
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (value)
    {
      putreg32(1 << pin, NRF51_GPIO0_OUTSET);
    }
  else
    {
      putreg32(1 << pin, NRF51_GPIO0_OUTCLR);
    }
}

/****************************************************************************
 * Name: nrf51_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nrf51_gpio_read(nrf51_pinset_t pinset)
{
  uint32_t regval;
  unsigned int pin;

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  regval = getreg32(NRF51_GPIO0_IN);

  return (regval >> pin) & 1UL;
}

static int nrf51_gpio_interrupt(int irq, FAR void *context, FAR void *arg)
{
  gpioinfo("NRF51_GPIO0_IN %x NRF51_GPIOTE_CONFIG0 %x\n",getreg32(NRF51_GPIO0_IN), getreg32(NRF51_GPIOTE_CONFIG0));
  if(getreg32(NRF51_GPIOTE_IN0)){

    if((getreg32(NRF51_GPIO0_IN) & 0x20000) == 0) {

    } else {

    }
    putreg32(0, NRF51_GPIOTE_IN0);
    putreg32(NRF51_GPIOTE_TASKS_IN0, NRF51_GPIOTE_INTENCLR);
    putreg32((NRF51_GPIOTE_CONF_POLARITY(NRF51_GPIOTE_CONF_POLARITY_HiToLo)) | (NRF51_GPIOTE_CONF_SEL0(17)) | NRF51_GPIOTE_CONF_MODE_EVENT, NRF51_GPIOTE_CONFIG0);
    // putreg32((getreg32(NRF51_GPIO0_OUTSET) | 0x20000), NRF51_GPIO0_OUTSET);

    putreg32(NRF51_GPIOTE_TASKS_IN0, NRF51_GPIOTE_INTEN);
    putreg32(NRF51_GPIOTE_TASKS_IN0, NRF51_GPIOTE_INTENSET);
    putreg32(1, NRF51_GPIOTE_IN0);
  }
  return 0;
}

void nrf51_gpio_irqinitialize(void)
{
  gpioinfo("nrf51_gpio_irqinitialize\n");

  putreg32(NRF51_GPIOTE_TASKS_ALL, NRF51_GPIOTE_INTENCLR);

  putreg32((NRF51_GPIOTE_CONF_POLARITY(NRF51_GPIOTE_CONF_POLARITY_HiToLo)) | (NRF51_GPIOTE_CONF_SEL0(17)) | NRF51_GPIOTE_CONF_MODE_EVENT, NRF51_GPIOTE_CONFIG0);

  putreg32(NRF51_GPIOTE_TASKS_IN0, NRF51_GPIOTE_INTEN);
  putreg32(NRF51_GPIOTE_TASKS_IN0, NRF51_GPIOTE_INTENSET);

  putreg32(1, NRF51_GPIOTE_IN0);
  (void)irq_attach(NRF51_IRQ_GPIOTE, nrf51_gpio_interrupt, NULL);
  up_enable_irq(NRF51_IRQ_GPIOTE);

  // gpioinfo("NRF51_GPIOTE_INTENCLR addr %x val %x\n",NRF51_GPIOTE_INTENCLR, getreg32(NRF51_GPIOTE_INTENCLR));
  // gpioinfo("NRF51_GPIOTE_CONFIG0 addr %x val %x\n",NRF51_GPIOTE_CONFIG0, getreg32(NRF51_GPIOTE_CONFIG0));
  // gpioinfo("NRF51_GPIOTE_INTEN addr %x val %x\n",NRF51_GPIOTE_INTEN, getreg32(NRF51_GPIOTE_INTEN));
  // gpioinfo("NRF51_GPIOTE_INTENSET addr %x val %x\n",getreg32(NRF51_GPIOTE_INTENSET));
  // gpioinfo("NRF51_GPIOTE_IN0 addr %x val %x\n",getreg32(NRF51_GPIOTE_IN0));
  return;
}
