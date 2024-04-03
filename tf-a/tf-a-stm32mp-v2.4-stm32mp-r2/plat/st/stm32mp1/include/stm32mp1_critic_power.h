/*
 * Copyright (C) 2019-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CRITIC_POWER_H
#define STM32MP1_CRITIC_POWER_H

 /* Only BL32 compilation unit need stm32_pwr_down_wfi
  * function/variable symbol
  */
#if defined(IMAGE_BL32)
 #if STM32MP_SP_MIN_IN_DDR
extern void (*stm32_pwr_down_wfi)(bool is_cstop, uint32_t mode);
 #else
extern void stm32_pwr_down_wfi(bool is_cstop, uint32_t mode);
 #endif
#endif
extern void stm32_pwr_down_wfi_wrapper(bool is_cstop, uint32_t mode);

#endif /* STM32MP1_CRITIC_POWER_H */
