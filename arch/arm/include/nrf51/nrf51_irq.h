/****************************************************************************************************
 * arch/arm/include/nrf51/nrf51_irq.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_NRF51_NRF51_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF51_NRF51_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Cortex-M0 External interrupts (vectors >= 16) */

#define NRF51_IRQ_POWER_CLOCK   (NRF51_IRQ_EXTINT+0)   /* Power, Clock, Bprot */
#define NRF51_IRQ_RADIO         (NRF51_IRQ_EXTINT+1)   /* Radio controller */
#define NRF51_IRQ_UART0         (NRF51_IRQ_EXTINT+2)   /* UART/UARTE 0 */
#define NRF51_IRQ_SPI_TWI_0     (NRF51_IRQ_EXTINT+3)   /* SPI / TWI 0 */
#define NRF51_IRQ_SPI_TWI_1     (NRF51_IRQ_EXTINT+4)   /* SPI / TWI 1 */
                                                       /* Reserved */
#define NRF51_IRQ_GPIOTE        (NRF51_IRQ_EXTINT+6)   /* GPIO Task & Event */
#define NRF51_IRQ_SAADC         (NRF51_IRQ_EXTINT+7)   /* Analog to Digital Converter */
#define NRF51_IRQ_TIMER0        (NRF51_IRQ_EXTINT+8)   /* Timer 0 */
#define NRF51_IRQ_TIMER1        (NRF51_IRQ_EXTINT+9)   /* Timer 1 */
#define NRF51_IRQ_TIMER2        (NRF51_IRQ_EXTINT+10)  /* Timer 2 */
#define NRF51_IRQ_RTC0          (NRF51_IRQ_EXTINT+11)  /* Real-time counter 0 */
#define NRF51_IRQ_TEMP          (NRF51_IRQ_EXTINT+12)  /* Temperature Sensor */
#define NRF51_IRQ_RNG           (NRF51_IRQ_EXTINT+13)  /* Random Number Generator */
#define NRF51_IRQ_ECB           (NRF51_IRQ_EXTINT+14)  /* AES ECB Mode Encryption */
#define NRF51_IRQ_CCM_AAR       (NRF51_IRQ_EXTINT+15)  /* AES CCM Mode Encryption/Accel. Address Resolve */
#define NRF51_IRQ_WDT           (NRF51_IRQ_EXTINT+16)  /* Watchdog Timer */
#define NRF51_IRQ_RTC1          (NRF51_IRQ_EXTINT+17)  /* Real-time counter 1 */
#define NRF51_IRQ_QDEC          (NRF51_IRQ_EXTINT+18)  /* Quadrature decoder */
#define NRF51_IRQ_COMP_LPCOMP   (NRF51_IRQ_EXTINT+19)  /* Low power comparator */
#define NRF51_IRQ_SWI0_EGU0     (NRF51_IRQ_EXTINT+20)  /* Software interrupt 0 / Event Gen. Unit 0 */
#define NRF51_IRQ_SWI1_EGU1     (NRF51_IRQ_EXTINT+21)  /* Software interrupt 1 / Event Gen. Unit 1 */
#define NRF51_IRQ_SWI2_EGU2     (NRF51_IRQ_EXTINT+22)  /* Software interrupt 2 / Event Gen. Unit 2 */
#define NRF51_IRQ_SWI3_EGU3     (NRF51_IRQ_EXTINT+23)  /* Software interrupt 3 / Event Gen. Unit 3 */
#define NRF51_IRQ_SWI4_EGU4     (NRF51_IRQ_EXTINT+24)  /* Software interrupt 4 / Event Gen. Unit 4 */
#define NRF51_IRQ_SWI5_EGU5     (NRF51_IRQ_EXTINT+25)  /* Software interrupt 5 / Event Gen. Unit 5 */


#define NRF51_IRQ_NEXTINT       (26)

#define NRF51_IRQ_NIRQS         (NRF51_IRQ_EXTINT+NRF51_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS                 NRF51_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF51_NRF51_IRQ_H */
