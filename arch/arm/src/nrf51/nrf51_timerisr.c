/****************************************************************************
 * arch/arm/src/nrf51/nrf51_timerisr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "up_internal.h"
#include "up_arch.h"
#include "nrf51_rtc.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  nrf51_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int nrf51_timerisr(int irq, uint32_t *regs, void *arg)
{
  if(getreg32(NRF51_RTC1_TICK)){
    putreg32(0x0, NRF51_RTC1_TICK);
    putreg32(NRF51_RTC1_BIT_TICK, NRF51_RTC1_EVTENCLR); // clear
    nxsched_process_timer();
  }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  arm_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void arm_timer_initialize(void)
{
  tmrinfo("wait run sata %x\n", getreg32(0x40000418));

  tmrinfo("LFCLKSTOP\n");
  putreg32(1, 0x4000000C); // LFCLKSTOP

  while((getreg32(0x40000418) & 0x00010000) != 0x00000000)
  {
      tmrinfo("wait stop 1\n");
  }

  putreg32(2, 0x40000518); // LFCLKSRC
  putreg32(0, 0x40000514); // LFCLKSTARTED
  putreg32(1, 0x40000008); // LFCLKSTART
  while (getreg32(0x40000418) == 0)
  {
      tmrinfo("wait stop 2\n");
  }
  putreg32(0, 0x40000514); // LFCLKSTARTED

  (void)irq_attach(NRF51_IRQ_RTC1, (xcpt_t)nrf51_timerisr, NULL);
  up_enable_irq(NRF51_IRQ_RTC1);

  putreg32(327, NRF51_RTC1_PRESCALER); // PRESCALER

  putreg32(NRF51_RTC1_BIT_TICK, NRF51_RTC1_INTENCLR); // INTENCLR
  putreg32(NRF51_RTC1_BIT_TICK, NRF51_RTC1_INTEN); // EVTENSET
  putreg32(NRF51_RTC1_BIT_TICK, NRF51_RTC1_INTENSET); // EVTENSET
  putreg32(1, NRF51_RTC1_START); // START
}
