/************************************************************************************
 * arch/arm/src/stm32h7/stm32_pwr.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32H7_STM32_PWR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "chip/stm32_pwr.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwr_initbkp
 *
 * Description:
 *   Insures the referenced count access to the backup domain (RTC registers,
 *   RTC backup data registers and backup SRAM is consistent with the HW state
 *   without relying on a variable.
 *
 *   NOTE: This function should only be called by SoC Start up code.
 *
 * Input Parameters:
 *   writable - set the initial state of the enable and the
 *              bkp_writable_counter
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_initbkp(bool writable);

/************************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data registers
 *   and backup SRAM).
 *
 * Input Parameters:
 *   writable - True: enable ability to write to backup domain registers
 *
 * Returned Value:
 *   none
 *
 ************************************************************************************/

void stm32_pwr_enablebkp(bool writable);

/************************************************************************************
 * Name: stm32_pwr_enablebreg
 *
 * Description:
 *   Enables the Backup regulator, the Backup regulator (used to maintain backup
 *   SRAM content in Standby and VBAT modes) is enabled. If BRE is reset, the backup
 *   regulator is switched off. The backup SRAM can still be used but its content
 *   will be lost in the Standby and VBAT modes. Once set, the application must wait
 *   that the Backup Regulator Ready flag (BRR) is set to indicate that the data
 *   written into the RAM will be maintained in the Standby and VBAT modes.
 *
 *   This function needs to be called after stm32_pwr_enablebkp(true) has been
 *   called.
 *
 * Input Parameters:
 *   region - state to set it to
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_enablebreg(bool region);

/************************************************************************************
 * Name: stm32_pwr_configurewkup
 *
 * Description:
 *   Configures the external wakeup (WKUP) signals for wakeup from standby mode.
 *   Sets rising/falling edge sensitivity and pull state.
 *
 *
 * Input Parameters:
 *   pin    - WKUP pin number (0-5) to work on
 *   en     - Enables the specified WKUP pin if true
 *   rising - If true, wakeup is triggered on rising edge, otherwise,
 *            it is triggered on the falling edge.
 *   pull   - Specifies the WKUP pin pull resistor configuration
 *            (GPIO_FLOAT, GPIO_PULLUP, or GPIO_PULLDOWN)
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_configurewkup(uint32_t pin, bool en, bool rising, uint32_t pull);

/************************************************************************************
 * Name: stm32_pwr_setvbatcharge
 *
 * Description:
 *   Configures the internal charge resistor to charge a battery attached to
 *   the VBAT pin.
 *
 *
 * Input Parameters:
 *   enable    - Enables the charge resistor if true, disables it if false
 *   resistor  - Sets charge resistor to 1.5 KOhm if true,
 *               sets it to 5 KOhm if false.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_setvbatcharge(bool enable, bool resistor);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_PWR_H */
