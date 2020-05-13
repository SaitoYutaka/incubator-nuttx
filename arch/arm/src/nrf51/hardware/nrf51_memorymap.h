/****************************************************************************
 * arch/arm/src/nrf51/hardware/nrf51_memorymap.h
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

#ifndef __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_MEMORYMAP_H
#define __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory Map */

#define NRF51_FLASH_BASE     0x00000000  /* Flash memory Start Address */
#define NRF51_SRAM_BASE      0x20000000  /* SRAM Start Address */
#define NRF51_FICR_BASE      0x10000000  /* FICR */
#define NRF51_UICR_BASE      0x10001000  /* UICR */
// #define NRF51_APB0_BASE      0x40000000  /* APB  */

// #define NRF51_CORTEXM4_BASE  0xe0000000  /* Cortex-M0 Private Peripheral Bus */

/* APB Peripherals */

#define NRF51_POWER_BASE    0x40000000
#define NRF51_CLOCK_BASE    0x40000000
#define NRF51_MPU_BASE      0x40000000
#define NRF51_RADIO_BASE    0x40001000
#define NRF51_UART0_BASE    0x40002000
#define NRF51_SPI0_BASE     0x40003000
#define NRF51_TWI0_BASE     0x40003000
#define NRF51_SPI1_BASE     0x40004000
#define NRF51_TWI1_BASE     0x40004000
#define NRF51_SPIS1_BASE    0x40004000
#define NRF51_GPIOTE_BASE   0x40006000
#define NRF51_ADC_BASE      0x40007000
#define NRF51_TIMER0_BASE   0x40008000
#define NRF51_TIMER1_BASE   0x40009000
#define NRF51_TIMER2_BASE   0x4000A000
#define NRF51_RTC0_BASE     0x4000B000
#define NRF51_TEMP_BASE     0x4000C000
#define NRF51_RNG_BASE      0x4000D000
#define NRF51_ECB_BASE      0x4000E000
#define NRF51_AAR_BASE      0x4000F000
#define NRF51_CCM_BASE      0x4000F000
#define NRF51_WDT_BASE      0x40010000
#define NRF51_RTC1_BASE     0x40011000
#define NRF51_QDEC_BASE     0x40012000
#define NRF51_LPCOMP_BASE   0x40013000
#define NRF51_SWI_BASE      0x40014000
#define NRF51_NVMC_BASE     0x4001E000
#define NRF51_PPI_BASE      0x4001F000
#define NRF51_FICR_BASE     0x10000000
#define NRF51_UICR_BASE     0x10001000
#define NRF51_GPIO_BASE     0x50000000

// #define NRF51_USBD_BASE     0x40027000
// #define NRF51_UARTE1_BASE   0x40028000
// #define NRF51_QSPI_BASE     0x40029000
// #define NRF51_PWM3_BASE     0x4002d000
// #define NRF51_SPIM3_BASE    0x4002f000

// #define NRF51_GPIO_P0_BASE  0x50000000
// #define NRF51_GPIO_P1_BASE  0x50003000

#endif /* __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_MEMORYMAP_H */
