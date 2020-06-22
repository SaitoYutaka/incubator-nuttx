/************************************************************************************
 * arch/arm/src/nrf51/nrf51_rtc.h
 * 
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_NRF51_RTC1_H
#define __ARCH_NRF51_RTC1_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf51_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
#define NRF51_RTC1_START_OFFSET      0x000 /* Start RTC COUNTER */
#define NRF51_RTC1_STOP_OFFSET       0x004 /* Stop RTC COUNTER */
#define NRF51_RTC1_CLEAR_OFFSET      0x008 /* Clear RTC COUNTER */
#define NRF51_RTC1_TRIGOVRFLW_OFFSET 0x00C /* Set COUNTER to 0xFFFFF0 */

#define NRF51_RTC1_TICK_OFFSET       0x100 /* Event on COUNTER increment */
#define NRF51_RTC1_OVRFLW_OFFSET     0x104 /* Event on COUNTER overflow */
#define NRF51_RTC1_COMPARE0_OFFSET   0x140 /* Compare event on CC[0] match */
#define NRF51_RTC1_COMPARE1_OFFSET   0x144 /* Compare event on CC[1] match */
#define NRF51_RTC1_COMPARE2_OFFSET   0x148 /* Compare event on CC[2] match */
#define NRF51_RTC1_COMPARE3_OFFSET   0x14C /* Compare event on CC[3] match */

#define NRF51_RTC1_INTEN_OFFSET      0x300 /* Enable or disable interrupt */
#define NRF51_RTC1_INTENSET_OFFSET   0x304 /* Enable interrupt */
#define NRF51_RTC1_INTENCLR_OFFSET   0x308 /* Disable interrupt */
#define NRF51_RTC1_EVTEN_OFFSET      0x340 /* Enable or disable event routing */
#define NRF51_RTC1_EVTENSET_OFFSET   0x344 /* Enable event routing */
#define NRF51_RTC1_EVTENCLR_OFFSET   0x348 /* Disable event routing */
#define NRF51_RTC1_COUNTER_OFFSET    0x504 /* Current COUNTER value */
#define NRF51_RTC1_PRESCALER_OFFSET  0x508 /* 12 bit prescaler for COUNTER frequency (32768/(PRESCALER+1)).Must be written when RTC isstopped */
#define NRF51_RTC1_CC0_OFFSET        0x540 /* Compare register 0 */
#define NRF51_RTC1_CC1_OFFSET        0x544 /* Compare register 1 */
#define NRF51_RTC1_CC2_OFFSET        0x548 /* Compare register 2 */
#define NRF51_RTC1_CC3_OFFSET        0x54C /* Compare register 3 */

/* Register Addresses ***************************************************************/
#define NRF51_RTC1_START      (NRF51_RTC1_BASE+NRF51_RTC1_START_OFFSET)
#define NRF51_RTC1_STOP       (NRF51_RTC1_BASE+NRF51_RTC1_STOP_OFFSET)
#define NRF51_RTC1_CLEAR      (NRF51_RTC1_BASE+NRF51_RTC1_CLEAR_OFFSET)
#define NRF51_RTC1_TRIGOVRFLW (NRF51_RTC1_BASE+NRF51_RTC1_TRIGOVRFLW_OFFSET)

#define NRF51_RTC1_TICK       (NRF51_RTC1_BASE+NRF51_RTC1_TICK_OFFSET)
#define NRF51_RTC1_OVRFLW     (NRF51_RTC1_BASE+NRF51_RTC1_OVRFLW_OFFSET)
#define NRF51_RTC1_COMPARE0   (NRF51_RTC1_BASE+NRF51_RTC1_COMPARE0_OFFSET)
#define NRF51_RTC1_COMPARE1   (NRF51_RTC1_BASE+NRF51_RTC1_COMPARE1_OFFSET)
#define NRF51_RTC1_COMPARE2   (NRF51_RTC1_BASE+NRF51_RTC1_COMPARE2_OFFSET)
#define NRF51_RTC1_COMPARE3   (NRF51_RTC1_BASE+NRF51_RTC1_COMPARE3_OFFSET)

#define NRF51_RTC1_INTEN      (NRF51_RTC1_BASE+NRF51_RTC1_INTEN_OFFSET)
#define NRF51_RTC1_INTENSET   (NRF51_RTC1_BASE+NRF51_RTC1_INTENSET_OFFSET)
#define NRF51_RTC1_INTENCLR   (NRF51_RTC1_BASE+NRF51_RTC1_INTENCLR_OFFSET)
#define NRF51_RTC1_EVTEN      (NRF51_RTC1_BASE+NRF51_RTC1_EVTEN_OFFSET)
#define NRF51_RTC1_EVTENSET   (NRF51_RTC1_BASE+NRF51_RTC1_EVTENSET_OFFSET)
#define NRF51_RTC1_EVTENCLR   (NRF51_RTC1_BASE+NRF51_RTC1_EVTENCLR_OFFSET)
#define NRF51_RTC1_COUNTER    (NRF51_RTC1_BASE+NRF51_RTC1_COUNTER_OFFSET)
#define NRF51_RTC1_PRESCALER  (NRF51_RTC1_BASE+NRF51_RTC1_PRESCALER_OFFSET)
#define NRF51_RTC1_CC0        (NRF51_RTC1_BASE+NRF51_RTC1_CC0_OFFSET)
#define NRF51_RTC1_CC1        (NRF51_RTC1_BASE+NRF51_RTC1_CC1_OFFSET)
#define NRF51_RTC1_CC2        (NRF51_RTC1_BASE+NRF51_RTC1_CC2_OFFSET)
#define NRF51_RTC1_CC3        (NRF51_RTC1_BASE+NRF51_RTC1_CC3_OFFSET)


/* Register Bit-field Definitions ***************************************************/

#define NRF51_RTC1_BIT_TICK     0x00000001
#define NRF51_RTC1_BIT_OVRFLW   0x00000002
#define NRF51_RTC1_BIT_COMPARE0 0x00010000
#define NRF51_RTC1_BIT_COMPARE1 0x00020000
#define NRF51_RTC1_BIT_COMPARE2 0x00040000
#define NRF51_RTC1_BIT_COMPARE3 0x00080000

#endif /* __ARCH_NRF51_RTC1_H */

