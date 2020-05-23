/***************************************************************************************************
 * arch/arm/src/nrf51/hardware/nrf51_nvmc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
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
 ***************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_NVMC_H
#define __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_NVMC_H

/***************************************************************************************************
 * Included Files
 ***************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf51_memorymap.h"

/***************************************************************************************************
 * Pre-processor Definitions
 ***************************************************************************************************/

/* NVMC Register Offsets ****************************************************************************/

/* Registers for the NVMC */

#define NRF51_NVMC_READY_OFFSET          0x400  /* Ready flag */
#define NRF51_NVMC_CONFIG_OFFSET         0x504  /* Configuration register */
#define NRF51_NVMC_ERASEPAGE_OFFSET      0x508  /* Register for erasing a page in Code area */
#define NRF51_NVMC_ERASEPCR1_OFFSET      0x508  /* Equivalent to ERASEPAGE */
#define NRF51_NVMC_ERASEALL_OFFSET       0x50c  /* Register for erasing all non-volatile user memory */
#define NRF51_NVMC_ERASEPCR0_OFFSET      0x510  /* Register for erasing a page in Code area. Equiv. ERASEPAGE */
#define NRF51_NVMC_ERASEUICR_OFFSET      0x514  /* Register for erasing User Information Configuration Registers */

/* NVMC Register Addresses **************************************************************************/

#define NRF51_NVMC_READY                 (NRF51_NVMC_BASE + NRF51_NVMC_READY_OFFSET)
#define NRF51_NVMC_CONFIG                (NRF51_NVMC_BASE + NRF51_NVMC_CONFIG_OFFSET)
#define NRF51_NVMC_ERASEPAGE             (NRF51_NVMC_BASE + NRF51_NVMC_ERASEPAGE_OFFSET)
#define NRF51_NVMC_ERASEPCR1             (NRF51_NVMC_BASE + NRF51_NVMC_ERASEPCR1_OFFSET)
#define NRF51_NVMC_ERASEALL              (NRF51_NVMC_BASE + NRF51_NVMC_ERASEALL_OFFSET)
#define NRF51_NVMC_ERASEPCR0             (NRF51_NVMC_BASE + NRF51_NVMC_ERASEPCR0_OFFSET)
#define NRF51_NVMC_ERASEUICR             (NRF51_NVMC_BASE + NRF51_NVMC_ERASEUICR_OFFSET)

/* NVMC Register Bitfield Definitions **************************************************************/

/* READY Register */

#define NVMC_READY_READY                 (1 << 0) /* NVMC is ready */

/* CONFIG Register */

#define NVMC_CONFIG_SHIFT                (0)
#define NVMC_CONFIG_MASK                 (3 << NVMC_CONFIG_SHIFT)
#define NVMC_CONFIG_REN                  (0 << NVMC_CONFIG_SHIFT) /* Read-only access */
#define NVMC_CONFIG_WEN                  (1 << NVMC_CONFIG_SHIFT) /* Write Enabled */
#define NVMC_CONFIG_EEN                  (2 << NVMC_CONFIG_SHIFT) /* Erase Enabled */

/* ICACHECNF Register */

#define NVMC_ICACHECNF_CACHEEN           (1 << 0) /* Cache enable */
#define NVMC_ICACHECNF_CACHEPROFEN       (1 << 8) /* Cache profiling enable */

/* INTENSET Register */

#endif /* __ARCH_ARM_SRC_NRF51_HARDWARE_NRF51_NVMC_H */
