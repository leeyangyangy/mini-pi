/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_LOW_POWER_H
#define STM32MP1_LOW_POWER_H

#include <stdbool.h>
#include <stdint.h>

#include <stm32mp1_critic_power.h>

void stm32_rcc_wakeup_update(bool state);
void stm32_apply_pmic_suspend_config(uint32_t mode);
bool stm32_is_cstop_done(void);
void stm32_exit_cstop(void);
void stm32_enter_low_power(uint32_t mode, uint32_t nsec_addr);
void stm32_auto_stop(void);
void stm32_init_low_power(void);

#endif /* STM32MP1_LOW_POWER_H */
