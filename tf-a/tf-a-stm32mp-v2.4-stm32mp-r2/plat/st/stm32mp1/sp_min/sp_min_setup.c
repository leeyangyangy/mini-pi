/*
 * Copyright (c) 2015-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/fdt_fixup.h>
#include <context.h>
#include <drivers/arm/gicv2.h>
#include <drivers/arm/tzc400.h>
#include <drivers/clk.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/regulator.h>
#include <drivers/st/bsec.h>
#include <drivers/st/etzpc.h>
#include <drivers/st/regulator_fixed.h>
#include <drivers/st/stm32_console.h>
#include <drivers/st/stm32_gpio.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32_rng.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32_tamp.h>
#include <drivers/st/stm32_timer.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32mp_reset.h>
#include <drivers/st/stm32mp1_clk.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stpmic1.h>
#include <dt-bindings/clock/stm32mp1-clks.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <platform_sp_min.h>
#include <stm32mp1_context.h>
#include <stm32mp1_low_power.h>
#include <stm32mp1_power_config.h>

/******************************************************************************
 * Placeholder variables for copying the arguments that have been passed to
 * BL32 from BL2.
 ******************************************************************************/
static entry_point_info_t bl33_image_ep_info;

static console_t console;
static struct dt_node_info dt_uart_info;

static const char * const tamper_name[] = {
	[INT_TAMP1] = "RTC power domain",
	[INT_TAMP2] = "Temperature monitoring",
	[INT_TAMP3] = "LSE monitoring",
	[INT_TAMP4] = "HSE monitoring",
};

static int stm32mp1_tamper_action(int id)
{
	const char *tamp_name = NULL;

	if ((id >= 0) && ((size_t)id < ARRAY_SIZE(tamper_name))) {
		tamp_name = tamper_name[id];
	}
	ERROR("Tamper %u (%s) occurs\n", id, tamp_name);

	return 1; /* ack TAMPER and reset system */
}

static void stm32_sgi1_it_handler(void)
{
	uint32_t id;

	stm32mp_mask_timer();

#if DEBUG
	stm32mp_dump_core_registers(false);
#endif

	gicv2_end_of_interrupt(ARM_IRQ_SEC_SGI_1);

	do {
		id = plat_ic_get_pending_interrupt_id();

		if (id <= MAX_SPI_ID) {
			gicv2_end_of_interrupt(id);

			plat_ic_disable_interrupt(id);
		}
	} while (id <= MAX_SPI_ID);

	stm32mp_wait_cpu_reset();
}

static void configure_wakeup_interrupt(void)
{
	int irq_num = fdt_rcc_enable_it("wakeup");

	if (irq_num < 0) {
		ERROR("irq_num = %d\n", irq_num);
		panic();
	}

	plat_ic_set_interrupt_priority(irq_num, STM32MP1_IRQ_RCC_SEC_PRIO);
}

static void initialize_pll1_settings(void)
{
	uint32_t cpu_voltage = 0U;

	if (stm32_are_pll1_settings_valid_in_context()) {
		return;
	}

	if (dt_pmic_status() > 0) {
		struct rdev *regul;
		int ret;

		regul = dt_get_cpu_regulator();
		if (regul == NULL) {
			panic();
		}

		ret = regulator_get_voltage(regul);
		if (ret < 0) {
			panic();
		}

		cpu_voltage = (uint32_t)ret;
	}

	if (stm32mp1_clk_compute_all_pll1_settings(cpu_voltage) != 0) {
		panic();
	}
}

static void disable_usb_phy_regulator(void)
{
	if (dt_pmic_status() > 0) {
		struct rdev *regul = dt_get_usb_phy_regulator();
		int ret;

		if (regul == NULL) {
			return;
		}

		if (regulator_is_enabled(regul) == 1) {
			ret = regulator_disable(regul);
			if (ret < 0) {
				WARN("USBPHYC phy-supply (%s) disable failed\n", regul->reg_name);
			}
		}
	}
}

/*******************************************************************************
 * Interrupt handler for FIQ (secure IRQ)
 ******************************************************************************/
void sp_min_plat_fiq_handler(uint32_t id)
{
	switch (id & INT_ID_MASK) {
	case ARM_IRQ_SEC_PHY_TIMER:
	case STM32MP1_IRQ_MCU_SEV:
	case STM32MP1_IRQ_RCC_WAKEUP:
		stm32mp1_calib_it_handler(id);
		break;
	case STM32MP1_IRQ_TZC400:
		tzc400_init(STM32MP1_TZC_BASE);
		tzc400_it_handler();
		panic();
		break;
	case STM32MP1_IRQ_TAMPSERRS:
		stm32_tamp_it_handler();
		break;
	case ARM_IRQ_SEC_SGI_1:
		stm32_sgi1_it_handler();
		break;
	case ARM_IRQ_SEC_SGI_6:
		/* tell the primary cpu to exit from stm32_pwr_down_wfi() */
		if (plat_my_core_pos() == STM32MP_PRIMARY_CPU) {
			stm32mp1_calib_set_wakeup(true);
		}
		gicv2_end_of_interrupt(ARM_IRQ_SEC_SGI_6);
		break;
	case STM32MP1_IRQ_IWDG1:
	case STM32MP1_IRQ_IWDG2:
		stm32_iwdg_it_handler(id);
		break;
	case STM32MP1_IRQ_AXIERRIRQ:
		ERROR("STM32MP1_IRQ_AXIERRIRQ generated\n");
		panic();
		break;
	default:
		ERROR("SECURE IT handler not define for it : %u\n", id);
		break;
	}
}

/*******************************************************************************
 * Return the value of the saved PC from the backup register if present
 ******************************************************************************/
static uintptr_t get_saved_pc(void)
{
	uint32_t bkpr_core1_addr =
		tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uint32_t saved_pc;
	uint32_t bkpr_core1_magic =
		tamp_bkpr(BOOT_API_CORE1_MAGIC_NUMBER_TAMP_BCK_REG_IDX);
	uint32_t magic_nb;

	clk_enable(RTCAPB);

	magic_nb = mmio_read_32(bkpr_core1_magic);
	saved_pc = mmio_read_32(bkpr_core1_addr);

	clk_disable(RTCAPB);

	if (magic_nb != BOOT_API_A7_CORE0_MAGIC_NUMBER) {
		return 0U;
	}

	/* BL33 return address should be in DDR */
	if ((saved_pc < STM32MP_DDR_BASE) ||
	    (saved_pc > (STM32MP_DDR_BASE + (dt_get_ddr_size() - 1U)))) {
		panic();
	}

	return saved_pc;
}

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL33 corresponds to the non-secure image type
 * while BL32 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *sp_min_plat_get_bl33_ep_info(void)
{
	entry_point_info_t *next_image_info = &bl33_image_ep_info;
	unsigned int console_flags;

	/*
	 * PC is set to 0 when resetting after STANDBY
	 * The context should be restored, and the image information
	 * should be filled with what was saved
	 */
	if (next_image_info->pc == 0U) {
		void *cpu_context;
		uintptr_t saved_pc;

		if (stm32_restore_context() != 0) {
			panic();
		}

		console_flags = CONSOLE_FLAG_CRASH | CONSOLE_FLAG_TRANSLATE_CRLF;
		if ((clk_is_enabled(dt_uart_info.clock)) &&
		    (clk_get_rate(dt_uart_info.clock) != 0U)) {
			console_flags |= CONSOLE_FLAG_BOOT;
#ifdef DEBUG
			console_flags |= CONSOLE_FLAG_RUNTIME;
#endif
		}
		console_set_scope(&console, console_flags);

		cpu_context = cm_get_context(NON_SECURE);

		next_image_info->spsr = read_ctx_reg(get_regs_ctx(cpu_context),
						     CTX_SPSR);

		/* PC should be retrieved in backup register if OK, else it can
		 * be retrieved from non-secure context
		 */
		saved_pc = get_saved_pc();
		if (saved_pc != 0U) {
			next_image_info->pc = saved_pc;
		} else {
			next_image_info->pc =
				read_ctx_reg(get_regs_ctx(cpu_context), CTX_LR);
		}

		regulator_core_resume();
	}

	return next_image_info;
}

CASSERT((STM32MP_SEC_SYSRAM_BASE == STM32MP_SYSRAM_BASE) &&
	((STM32MP_SEC_SYSRAM_BASE + STM32MP_SEC_SYSRAM_SIZE) <=
	 (STM32MP_SYSRAM_BASE + STM32MP_SYSRAM_SIZE)),
	assert_secure_sysram_fits_at_begining_of_sysram);

#ifdef STM32MP_NS_SYSRAM_BASE
CASSERT((STM32MP_NS_SYSRAM_BASE >= STM32MP_SEC_SYSRAM_BASE) &&
	((STM32MP_NS_SYSRAM_BASE + STM32MP_NS_SYSRAM_SIZE) ==
	 (STM32MP_SYSRAM_BASE + STM32MP_SYSRAM_SIZE)),
	assert_non_secure_sysram_fits_at_end_of_sysram);

CASSERT((STM32MP_NS_SYSRAM_BASE & (PAGE_SIZE_4KB - U(1))) == 0U,
	assert_non_secure_sysram_base_is_4kbyte_aligned);

#define TZMA1_SECURE_RANGE \
	(((STM32MP_NS_SYSRAM_BASE - STM32MP_SYSRAM_BASE) >> FOUR_KB_SHIFT) - 1U)
#else
#define TZMA1_SECURE_RANGE		STM32MP1_ETZPC_TZMA_ALL_SECURE
#endif /* STM32MP_NS_SYSRAM_BASE */
#define TZMA0_SECURE_RANGE		STM32MP1_ETZPC_TZMA_ALL_SECURE

static void stm32mp1_etzpc_early_setup(void)
{
	if (etzpc_init() != 0) {
		panic();
	}

	etzpc_configure_tzma(STM32MP1_ETZPC_TZMA_ROM, TZMA0_SECURE_RANGE);
	etzpc_configure_tzma(STM32MP1_ETZPC_TZMA_SYSRAM, TZMA1_SECURE_RANGE);
}

#if STM32MP_SP_MIN_IN_DDR
static void populate_ns_dt(u_register_t ns_dt_addr, uintptr_t sec_base, size_t sec_size)
{
	void *external_fdt = (void *)ns_dt_addr;
	int ret;

	/* Map Base Non Secure DDR for Non secure DT update */
	ret = mmap_add_dynamic_region(ns_dt_addr, ns_dt_addr, STM32MP_HW_CONFIG_MAX_SIZE,
				      MT_NON_CACHEABLE | MT_EXECUTE_NEVER | MT_RW | MT_NS);
	assert(ret == 0);

	if (fdt_check_header(external_fdt) != 0) {
		INFO("Non-secure device tree not found\n");

		goto out;
	}

	ret = fdt_open_into(external_fdt, external_fdt, STM32MP_HW_CONFIG_MAX_SIZE);
	if (ret < 0) {
		WARN("Error opening DT %i\n", ret);
		goto out;
	}

	ret = fdt_add_reserved_memory(external_fdt, "tf-a", sec_base, sec_size);
	if (ret < 0) {
		WARN("Error updating DT %i\n", ret);
		goto out;
	}

	ret = fdt_pack(external_fdt);
	if (ret < 0) {
		WARN("Error packing DT %i\n", ret);
	}

out:
	ret = mmap_remove_dynamic_region(ns_dt_addr, STM32MP_HW_CONFIG_MAX_SIZE);
	assert(ret == 0);
}
#endif

/*******************************************************************************
 * Setup UART console using device tree information.
 ******************************************************************************/
static void setup_uart_console(void)
{
	unsigned int console_flags;
	int result;
	uint32_t boot_itf;
	uint32_t boot_instance;

	result = dt_get_stdout_uart_info(&dt_uart_info);
	if ((result <= 0) || (dt_uart_info.status == DT_DISABLED)) {
		return;
	}

	stm32_get_boot_interface(&boot_itf, &boot_instance);

	if ((boot_itf == BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART) &&
	    (get_uart_address(boot_instance) == dt_uart_info.base)) {
		return;
	}

	if (console_stm32_register(dt_uart_info.base, 0,
				   STM32MP_UART_BAUDRATE, &console) == 0U) {
		panic();
	}

	console_flags = CONSOLE_FLAG_BOOT | CONSOLE_FLAG_CRASH |
			CONSOLE_FLAG_TRANSLATE_CRLF;
#ifdef DEBUG
	console_flags |= CONSOLE_FLAG_RUNTIME;
#endif
	console_set_scope(&console, console_flags);
}

/*******************************************************************************
 * Perform any BL32 specific platform actions.
 ******************************************************************************/
void sp_min_early_platform_setup2(u_register_t arg0, u_register_t arg1,
				  u_register_t arg2, u_register_t arg3)
{
	bl_params_t *params_from_bl2 = (bl_params_t *)arg0;
#if STM32MP_USE_STM32IMAGE
	uintptr_t dt_addr = STM32MP_DTB_BASE;
#else
	uintptr_t dt_addr = arg1;
#endif
#if STM32MP_SP_MIN_IN_DDR
	uintptr_t sec_base = 0U;
	uintptr_t bl2_code_base = 0U;
	uintptr_t bl2_code_end = 0U;
	uintptr_t bl2_end = 0U;
	int result __unused;
#endif

	/* Imprecise aborts can be masked in NonSecure */
	write_scr(read_scr() | SCR_AW_BIT);

	mmap_add_region(BL_CODE_BASE, BL_CODE_BASE,
			BL_CODE_END - BL_CODE_BASE,
			MT_CODE | MT_SECURE);

#if STM32MP_SP_MIN_IN_DDR
	/* BL32 data*/
	mmap_add_region(BL_CODE_END, BL_CODE_END,
			BL_END - BL_CODE_END,
			MT_RW_DATA | MT_SECURE);

	/* BL32 Device Tree Blob */
	mmap_add_region(dt_addr, dt_addr,
			STM32MP_BL32_DTB_SIZE,
			MT_RO_DATA | MT_SECURE);

	/* Map SCMI shared buffers */
	mmap_add_region(STM32MP_SCMI_NS_SHM_BASE, STM32MP_SCMI_NS_SHM_BASE,
			STM32MP_SCMI_NS_SHM_SIZE,
			MT_DEVICE | MT_RW | MT_NS | MT_EXECUTE_NEVER);
#endif

	configure_mmu();

	if (dt_open_and_check(dt_addr) < 0) {
		panic();
	}

	if (bsec_probe() != 0) {
		panic();
	}

	if (stm32mp1_clk_probe() < 0) {
		panic();
	}

	setup_uart_console();

	stm32mp1_etzpc_early_setup();

#if STM32MP_SP_MIN_IN_DDR
	stm32_context_get_bl2_low_power_params(&bl2_code_base, &bl2_code_end, &bl2_end);

	/* BL2 Code */
	result = mmap_add_dynamic_region(bl2_code_base, bl2_code_base,
					 bl2_code_end - bl2_code_base,
					 MT_CODE | MT_SECURE);
	assert(result == 0);

	/* BL2 RW memory */
	result = mmap_add_dynamic_region(bl2_code_end, bl2_code_end,
					 bl2_end - bl2_code_end,
					 MT_RW_DATA | MT_SECURE);
	assert(result == 0);
#endif

	assert(params_from_bl2 != NULL);
	assert(params_from_bl2->h.type == PARAM_BL_PARAMS);
	assert(params_from_bl2->h.version >= VERSION_2);

	bl_params_node_t *bl_params = params_from_bl2->head;

	while (bl_params != NULL) {
		/*
		 * Copy BL33 entry point information.
		 * They are stored in Secure RAM, in BL2's address space.
		 */
		if (bl_params->image_id == BL33_IMAGE_ID) {
			bl33_image_ep_info = *bl_params->ep_info;
			/*
			 *  Check if hw_configuration is given to BL32 and
			 *  share it to BL33
			 */
			if (arg2 != 0U) {
				bl33_image_ep_info.args.arg0 = 0U;
				bl33_image_ep_info.args.arg1 = 0U;
				bl33_image_ep_info.args.arg2 = arg2;
			}
		}

#if STM32MP_SP_MIN_IN_DDR
		if (bl_params->image_id == BL32_IMAGE_ID) {
			sec_base = bl_params->image_info->image_base;
		}
#endif

		bl_params = bl_params->next_params_info;
	}

#if STM32MP_SP_MIN_IN_DDR
	if (arg2 != 0U) {
		/* This will expect the BL32 DT and BL32 are grouped */
		if (dt_addr < sec_base) {
			sec_base = dt_addr;
		}

		populate_ns_dt(arg2, sec_base, DDR_SEC_SIZE);
	} else {
		INFO("Non-secure device tree not found\n");
	}
#endif

	generic_delay_timer_init();

	if (dt_pmic_status() > 0) {
		initialize_pmic();
	}

	fixed_regulator_register();

	if (regulator_core_config() != 0) {
		ERROR("Regulator core config error\n");
		panic();
	}

	disable_usb_phy_regulator();

	initialize_pll1_settings();

	stm32mp1_init_lp_states();
}

static void init_sec_peripherals(void)
{
	int ret;

	/* Disable MCU subsystem protection */
	stm32mp1_clk_mcuss_protect(false);

	/* Init rtc driver */
	ret = stm32_rtc_init();
	if (ret < 0) {
		WARN("RTC driver init error %i\n", ret);
	}

	/*  Init rng driver */
	ret = stm32_rng_init();
	if (ret < 0) {
		WARN("RNG driver init error %i\n", ret);
	}

	/* Init tamper */
	if (stm32_tamp_init() > 0) {
		struct bkpregs_conf bkpregs_conf = {
			.nb_zone1_regs = TAMP_BKP_SEC_NUMBER,
			.nb_zone2_regs = 0 /* no register in zone 2 */
			/* zone3 all remaining */
		};

		/* Enable BKP Register protection */
		if (stm32_tamp_set_secure_bkpregs(&bkpregs_conf) < 0) {
			panic();
		}

		stm32_tamp_configure_secure_access(TAMP_REGS_IT_SECURE);

		stm32_tamp_configure_internal(INT_TAMP1, TAMP_ENABLE, stm32mp1_tamper_action);
		stm32_tamp_configure_internal(INT_TAMP2, TAMP_ENABLE, stm32mp1_tamper_action);
		stm32_tamp_configure_internal(INT_TAMP3, TAMP_ENABLE, stm32mp1_tamper_action);
		stm32_tamp_configure_internal(INT_TAMP4, TAMP_ENABLE, stm32mp1_tamper_action);

		ret = stm32_tamp_set_config();
		if (ret < 0) {
			panic();
		}

		/* Enable timestamp for tamper */
		stm32_rtc_set_tamper_timestamp();
	}

	if (stm32_timer_init() == 0) {
		stm32mp1_calib_init();
	}
}

/*******************************************************************************
 * Initialize the MMU, security and the GIC.
 ******************************************************************************/
void sp_min_platform_setup(void)
{
	stm32_init_low_power();

	ddr_save_sr_mode();

	stm32_gic_init();

	init_sec_peripherals();

	if (stm32_iwdg_init() < 0) {
		panic();
	}

	configure_wakeup_interrupt();

	stm32mp_lock_periph_registering();

	stm32mp1_init_scmi_server();

	/* Cold boot: clean-up regulators state */
	if (get_saved_pc() == 0U) {
		regulator_core_cleanup();
	}
}

void sp_min_plat_arch_setup(void)
{
}
