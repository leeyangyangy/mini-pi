/*
 * Copyright (c) 2015-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_PRIVATE_H
#define STM32MP1_PRIVATE_H

#include <stdint.h>

#include <drivers/st/etzpc.h>

enum boot_device_e {
	BOOT_DEVICE_USB,
	BOOT_DEVICE_BOARD
};

void configure_mmu(void);

void stm32mp_mask_timer(void);
void __dead2 stm32mp_wait_cpu_reset(void);

void stm32mp1_arch_security_setup(void);
void stm32mp1_security_setup(void);

enum boot_device_e get_boot_device(void);

bool stm32mp1_addr_inside_backupsram(uintptr_t addr);
bool stm32mp1_is_wakeup_from_standby(void);

int stm32_save_boot_interface(uint32_t interface, uint32_t instance);
int stm32_get_boot_interface(uint32_t *interface, uint32_t *instance);
bool stm32_boot_is_serial(void);

enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode);

void stm32mp1_syscfg_init(void);
void stm32mp1_syscfg_enable_io_compensation_start(void);
void stm32mp1_syscfg_enable_io_compensation_finish(void);
void stm32mp1_syscfg_disable_io_compensation(void);

void stm32mp1_deconfigure_uart_pins(void);

#if STM32MP_USE_STM32IMAGE
uint32_t stm32mp_get_ddr_ns_size(void);
#endif

void stm32mp1_init_scmi_server(void);
void stm32mp1_pm_save_scmi_state(uint8_t *state, size_t size);
void stm32mp1_pm_restore_scmi_state(uint8_t *state, size_t size);

#if defined(IMAGE_BL32) && DEBUG
void stm32mp_dump_core_registers(bool fcore);
#endif
#endif /* STM32MP1_PRIVATE_H */
