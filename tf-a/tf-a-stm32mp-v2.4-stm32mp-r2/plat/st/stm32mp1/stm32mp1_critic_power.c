/*
 * Copyright (C) 2019-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <dt-bindings/power/stm32mp1-power.h>

#include <stm32mp1_context.h>
#include <stm32mp1_critic_power.h>

static void cstop_critic_enter(uint32_t mode)
{
	/* Switch to Software Self-Refresh mode */
	ddr_set_sr_mode(DDR_SSR_MODE);

	/*
	 * Set DDR in Self-refresh,.
	 * This is also the procedure awaited when switching off power supply.
	 */
	if (ddr_standby_sr_entry() != 0) {
		ERROR("Unable to put DDR in SR\n");
		if (mode != STM32_PM_SHUTDOWN) {
			panic();
		}
	}
}

static void shutdown_critic_enter(void)
{
	if (dt_pmic_status() > 0) {
		if (!initialize_pmic_i2c()) {
			panic();
		}

		pmic_switch_off();
	}
}

/*
 * stm32_exit_cstop_critic - Exit from CSTOP mode reenable DDR
 */
static void cstop_critic_exit(void)
{
	if (ddr_sw_self_refresh_exit() != 0) {
		panic();
	}
}

void stm32_pwr_down_wfi_load(bool is_cstop, uint32_t mode)
{
	if (mode != STM32_PM_CSLEEP_RUN) {
		dcsw_op_all(DC_OP_CISW);
	}

	if (is_cstop) {
		cstop_critic_enter(mode);
	}

	if (mode == STM32_PM_SHUTDOWN) {
		shutdown_critic_enter();
	}

	/*
	 * Synchronize on memory accesses and instruction flow before the WFI
	 * instruction.
	 */
	dsb();
	isb();
	wfi();

	stm32_iwdg_refresh();

	if (is_cstop) {
		cstop_critic_exit();
	}
}

#if defined(IMAGE_BL32) && !STM32MP_SP_MIN_IN_DDR
extern void wfi_svc_int_enable(uintptr_t stack_addr);
static uint32_t int_stack[STM32MP_INT_STACK_SIZE];

void stm32_pwr_down_wfi(bool is_cstop, uint32_t mode)
{
	uint32_t interrupt = GIC_SPURIOUS_INTERRUPT;

	if (mode != STM32_PM_CSLEEP_RUN) {
		dcsw_op_all(DC_OP_CISW);
	}

	if (is_cstop) {
		cstop_critic_enter(mode);
	}

	if (mode == STM32_PM_SHUTDOWN) {
		shutdown_critic_enter();
	}

	stm32mp1_calib_set_wakeup(false);

	while (interrupt == GIC_SPURIOUS_INTERRUPT &&
	       !stm32mp1_calib_get_wakeup()) {
		wfi_svc_int_enable((uintptr_t)&int_stack[0]);

		interrupt = gicv2_acknowledge_interrupt();

		if (interrupt != GIC_SPURIOUS_INTERRUPT) {
			gicv2_end_of_interrupt(interrupt);
		}

		stm32_iwdg_refresh();
	}

	if (is_cstop) {
		cstop_critic_exit();
	}
}
#endif
