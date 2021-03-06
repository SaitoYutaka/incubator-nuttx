/************************************************************************************************
 * arch/arm/src/nrf51/hardware/nrf51_clock.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_CLOCK_H
#define __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_CLOCK_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf51_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Register offsets *****************************************************************************/

#define NRF51_CLOCK_TASKS_HFCLKSTART_OFFSET    0x0000 /* Start HFCLK crystal oscillator */
#define NRF51_CLOCK_TASKS_HFCLKSTOP_OFFSET     0x0004 /* Stop HFCLK crystal oscillator */
#define NRF51_CLOCK_TASKS_LFCLKSTART_OFFSET    0x0008 /* Start LFCLK source */
#define NRF51_CLOCK_TASKS_LFCLKSTOP_OFFSET     0x000c /* Stop LFCLK source */
#define NRF51_CLOCK_TASKS_CAL_OFFSET           0x0010 /* Start calibration of LFRC oscillator */
#define NRF51_CLOCK_TASKS_CTSTART_OFFSET       0x0014 /* Start calibration timer */
#define NRF51_CLOCK_TASKS_CTSTOP_OFFSET        0x0018 /* Stop calibration timer */
#define NRF51_CLOCK_EVENTS_HFCLKSTARTED_OFFSET 0x0100 /* HFCLK oscillator started */
#define NRF51_CLOCK_EVENTS_LFCLKSTARTED_OFFSET 0x0104 /* LFCLK started */
#define NRF51_CLOCK_EVENTS_DONE_OFFSET         0x010c /* Calibration of LFCLK RC oscillator complete event */
#define NRF51_CLOCK_EVENTS_CTTO_OFFSET         0x0110 /* Calibration timer timeout */
#define NRF51_CLOCK_INTENSET_OFFSET            0x0304 /* Enable interrupt */
#define NRF51_CLOCK_INTENCLR_OFFSET            0x0308 /* Disable interrupt */
#define NRF51_CLOCK_HFCLKRUN_OFFSET            0x0408 /* Status indicating that HFCLKSTART task has been triggered */
#define NRF51_CLOCK_HFCLKSTAT_OFFSET           0x040c /* HFCLK status */
#define NRF51_CLOCK_LFCLKRUN_OFFSET            0x0414 /* Status indicating that LFCLKSTART task has been triggered */
#define NRF51_CLOCK_LFCLKSTAT_OFFSET           0x0418 /* LFCLK status */
#define NRF51_CLOCK_LFCLKSRCCOPY_OFFSET        0x041c /* Copy of LFCLKSRC register, set when LFCLKSTART task was triggered */
#define NRF51_CLOCK_LFCLKSRC_OFFSET            0x0518 /* Clock source for the LFCLK */
#define NRF51_CLOCK_CTIV_OFFSET                0x0538 /* Calibration timer interval */
#define NRF51_CLOCK_CRYSTALFREQUENCY_OFFSET    0x0550 /* Crystal frequency */

/* Register Addresses ***************************************************************************/

#define NRF51_CLOCK_TASKS_HFCLKSTART    (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_HFCLKSTART_OFFSET)
#define NRF51_CLOCK_TASKS_HFCLKSTOP     (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_HFCLKSTOP_OFFSET)
#define NRF51_CLOCK_TASKS_LFCLKSTART    (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_LFCLKSTART_OFFSET)
#define NRF51_CLOCK_TASKS_LFCLKSTOP     (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_LFCLKSTOP_OFFSET)
#define NRF51_CLOCK_TASKS_CAL           (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_CAL_OFFSET)
#define NRF51_CLOCK_TASKS_CTSTART       (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_CTSTART_OFFSET)
#define NRF51_CLOCK_TASKS_CTSTOP        (NRF51_CLOCK_BASE + NRF51_CLOCK_TASKS_CTSTOP_OFFSET)
#define NRF51_CLOCK_EVENTS_HFCLKSTARTED (NRF51_CLOCK_BASE + NRF51_CLOCK_EVENTS_HFCLKSTARTED_OFFSET)
#define NRF51_CLOCK_EVENTS_LFCLKSTARTED (NRF51_CLOCK_BASE + NRF51_CLOCK_EVENTS_LFCLKSTARTED_OFFSET)
#define NRF51_CLOCK_EVENTS_DONE         (NRF51_CLOCK_BASE + NRF51_CLOCK_EVENTS_DONE_OFFSET)
#define NRF51_CLOCK_EVENTS_CTTO         (NRF51_CLOCK_BASE + NRF51_CLOCK_EVENTS_CTTO_OFFSET)
#define NRF51_CLOCK_INTENSET            (NRF51_CLOCK_BASE + NRF51_CLOCK_INTENSET_OFFSET)
#define NRF51_CLOCK_INTENCLR            (NRF51_CLOCK_BASE + NRF51_CLOCK_INTENCLR_OFFSET)
#define NRF51_CLOCK_HFCLKRUN            (NRF51_CLOCK_BASE + NRF51_CLOCK_HFCLKRUN_OFFSET)
#define NRF51_CLOCK_HFCLKSTAT           (NRF51_CLOCK_BASE + NRF51_CLOCK_HFCLKSTAT_OFFSET)
#define NRF51_CLOCK_LFCLKRUN            (NRF51_CLOCK_BASE + NRF51_CLOCK_LFCLKRUN_OFFSET)
#define NRF51_CLOCK_LFCLKSTAT           (NRF51_CLOCK_BASE + NRF51_CLOCK_LFCLKSTAT_OFFSET)
#define NRF51_CLOCK_LFCLKSRCCOPY        (NRF51_CLOCK_BASE + NRF51_CLOCK_LFCLKSRCCOPY_OFFSET)
#define NRF51_CLOCK_LFCLKSRC            (NRF51_CLOCK_BASE + NRF51_CLOCK_LFCLKSRC_OFFSET )
#define NRF51_CLOCK_CTIV                (NRF51_CLOCK_BASE + NRF51_CLOCK_CTIV_OFFSET)
#define NRF51_CLOCK_CRYSTALFREQUENCY    (NRF51_CLOCK_BASE + NRF51_CLOCK_CRYSTALFREQUENCY_OFFSET)

/* Register Bitfield Definitions ****************************************************************/

#endif /* __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_LCD_H */
