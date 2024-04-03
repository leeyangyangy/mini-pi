/*
 * Copyright (c) 2015-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <libfdt.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <drivers/arm/gicv2.h>
#include <drivers/clk.h>
#include <drivers/st/stm32_gpio.h>
#include <drivers/st/stm32_iwdg.h>
#include <drivers/st/stm32mp_pmic.h>
#include <lib/mmio.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>
#include <smccc_helpers.h>

/* Internal layout of the 32bit OTP word board_id */
#define BOARD_ID_BOARD_NB_MASK		GENMASK(31, 16)
#define BOARD_ID_BOARD_NB_SHIFT		16
#define BOARD_ID_VARCPN_MASK		GENMASK(15, 12)
#define BOARD_ID_VARCPN_SHIFT		12
#define BOARD_ID_REVISION_MASK		GENMASK(11, 8)
#define BOARD_ID_REVISION_SHIFT		8
#define BOARD_ID_VARFG_MASK		GENMASK(7, 4)
#define BOARD_ID_VARFG_SHIFT		4
#define BOARD_ID_BOM_MASK		GENMASK(3, 0)

#define BOARD_ID2NB(_id)		(((_id) & BOARD_ID_BOARD_NB_MASK) >> \
					 BOARD_ID_BOARD_NB_SHIFT)
#define BOARD_ID2VARCPN(_id)		(((_id) & BOARD_ID_VARCPN_MASK) >> \
					 BOARD_ID_VARCPN_SHIFT)
#define BOARD_ID2REV(_id)		(((_id) & BOARD_ID_REVISION_MASK) >> \
					 BOARD_ID_REVISION_SHIFT)
#define BOARD_ID2VARFG(_id)		(((_id) & BOARD_ID_VARFG_MASK) >> \
					 BOARD_ID_VARFG_SHIFT)
#define BOARD_ID2BOM(_id)		((_id) & BOARD_ID_BOM_MASK)

#define TAMP_BOOT_ITF_BACKUP_REG_ID	U(20)
#define TAMP_BOOT_ITF_MASK		U(0x0000FF00)
#define TAMP_BOOT_ITF_SHIFT		8

#if defined(IMAGE_BL2)
#define MAP_SEC_SYSRAM	MAP_REGION_FLAT(STM32MP_SYSRAM_BASE, \
					STM32MP_SYSRAM_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)
#elif defined(IMAGE_BL32)
#define MAP_SEC_SYSRAM	MAP_REGION_FLAT(STM32MP_SEC_SYSRAM_BASE, \
					STM32MP_SEC_SYSRAM_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

/* Non-secure SYSRAM is used a uncached memory for SCMI message transfer */
#define MAP_NS_SYSRAM	MAP_REGION_FLAT(STM32MP_NS_SYSRAM_BASE, \
					STM32MP_NS_SYSRAM_SIZE, \
					MT_DEVICE | \
					MT_RW | \
					MT_NS | \
					MT_EXECUTE_NEVER)
#endif

#define MAP_RETRAM	MAP_REGION_FLAT(STM32MP_RETRAM_BASE, \
					STM32MP_RETRAM_SIZE, \
					MT_MEMORY | \
					MT_RW | \
					MT_NS | \
					MT_EXECUTE_NEVER)

#define MAP_DEVICE1	MAP_REGION_FLAT(STM32MP1_DEVICE1_BASE, \
					STM32MP1_DEVICE1_SIZE, \
					MT_DEVICE | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

#define MAP_DEVICE2	MAP_REGION_FLAT(STM32MP1_DEVICE2_BASE, \
					STM32MP1_DEVICE2_SIZE, \
					MT_DEVICE | \
					MT_RW | \
					MT_SECURE | \
					MT_EXECUTE_NEVER)

#if defined(IMAGE_BL2)
static const mmap_region_t stm32mp1_mmap[] = {
	MAP_SEC_SYSRAM,
	MAP_DEVICE1,
#if STM32MP_RAW_NAND
	MAP_DEVICE2,
#endif
	{0}
};
#endif
#if defined(IMAGE_BL32)
static const mmap_region_t stm32mp1_mmap[] = {
#if !STM32MP_SP_MIN_IN_DDR
	MAP_SEC_SYSRAM,
	MAP_NS_SYSRAM,
#endif
	MAP_DEVICE1,
	MAP_DEVICE2,
	{0}
};
#endif

void configure_mmu(void)
{
#ifndef MMU_OFF
	unsigned int flags = 0;

	mmap_add(stm32mp1_mmap);
	init_xlat_tables();
#ifdef DCACHE_OFF
	flags |= DISABLE_DCACHE;
#endif
	enable_mmu_svc_mon(flags);
#endif
}

#define ARM_CNTXCTL_IMASK	BIT(1)

void stm32mp_mask_timer(void)
{
	/* Mask timer interrupts */
	write_cntp_ctl(read_cntp_ctl() | ARM_CNTXCTL_IMASK);
	write_cntv_ctl(read_cntv_ctl() | ARM_CNTXCTL_IMASK);
}

void __dead2 stm32mp_wait_cpu_reset(void)
{
	uint32_t id;

	dcsw_op_all(DC_OP_CISW);
	write_sctlr(read_sctlr() & ~SCTLR_C_BIT);
	dcsw_op_all(DC_OP_CISW);
	__asm__("clrex");

	dsb();
	isb();

	for ( ; ; ) {
		do {
			id = plat_ic_get_pending_interrupt_id();

			if (id <= MAX_SPI_ID) {
				gicv2_end_of_interrupt(id);

				plat_ic_disable_interrupt(id);
			}
		} while (id <= MAX_SPI_ID);

		wfi();
	}
}

uintptr_t stm32_get_gpio_bank_base(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return GPIOZ_BASE;
	}

	assert(GPIO_BANK_A == 0 && bank <= GPIO_BANK_K);

	return GPIOA_BASE + (bank * GPIO_BANK_OFFSET);
}

uint32_t stm32_get_gpio_bank_offset(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return 0;
	}

	assert(GPIO_BANK_A == 0 && bank <= GPIO_BANK_K);

	return bank * GPIO_BANK_OFFSET;
}

bool stm32_gpio_is_secure_at_reset(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return true;
	}

	return false;
}

#if STM32MP_UART_PROGRAMMER || defined(IMAGE_BL32)
/*
 * UART Management
 */
static const uintptr_t stm32mp1_uart_addresses[8] = {
	USART1_BASE,
	USART2_BASE,
	USART3_BASE,
	UART4_BASE,
	UART5_BASE,
	USART6_BASE,
	UART7_BASE,
	UART8_BASE,
};

uintptr_t get_uart_address(uint32_t instance_nb)
{
	if ((instance_nb == 0U) ||
	    (instance_nb > ARRAY_SIZE(stm32mp1_uart_addresses))) {
		return 0U;
	}

	return stm32mp1_uart_addresses[instance_nb - 1U];
}
#endif

#if STM32MP_USB_PROGRAMMER
struct gpio_bank_pin_list {
	uint32_t bank;
	uint32_t pin;
};

static const struct gpio_bank_pin_list gpio_list[] = {
	{	/* USART2_RX: GPIOA3 */
		.bank = 0,
		.pin = 3,
	},
	{	/* USART3_RX: GPIOB12 */
		.bank = 1,
		.pin = 12,
	},
	{	/* UART4_RX: GPIOB2 */
		.bank = 1,
		.pin = 2,
	},
	{	/* UART5_RX: GPIOB4 */
		.bank = 1,
		.pin = 5,
	},
	{	/* USART6_RX: GPIOC7 */
		.bank = 2,
		.pin = 7,
	},
	{	/* UART7_RX: GPIOF6 */
		.bank = 5,
		.pin = 6,
	},
	{	/* UART8_RX: GPIOE0 */
		.bank = 4,
		.pin = 0,
	},
};

void stm32mp1_deconfigure_uart_pins(void)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_list); i++) {
		set_gpio_reset_cfg(gpio_list[i].bank, gpio_list[i].pin);
	}
}
#endif

unsigned long stm32_get_gpio_bank_clock(unsigned int bank)
{
	if (bank == GPIO_BANK_Z) {
		return GPIOZ;
	}

	assert(GPIO_BANK_A == 0 && bank <= GPIO_BANK_K);

	return GPIOA + (bank - GPIO_BANK_A);
}

int stm32_get_gpio_bank_pinctrl_node(void *fdt, unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A:
	case GPIO_BANK_B:
	case GPIO_BANK_C:
	case GPIO_BANK_D:
	case GPIO_BANK_E:
	case GPIO_BANK_F:
	case GPIO_BANK_G:
	case GPIO_BANK_H:
	case GPIO_BANK_I:
	case GPIO_BANK_J:
	case GPIO_BANK_K:
		return fdt_path_offset(fdt, "/soc/pin-controller");
	case GPIO_BANK_Z:
		return fdt_path_offset(fdt, "/soc/pin-controller-z");
	default:
		panic();
	}
}

int stm32_get_otp_index(const char *otp_name, uint32_t *otp_idx,
			uint32_t *otp_len)
{
	assert(otp_name != NULL);
	assert(otp_idx != NULL);

	if (bsec_find_otp_name_in_dt(otp_name, otp_idx, otp_len) != BSEC_OK) {
		return -1;
	}

	return 0;
}

int stm32_get_otp_value(const char *otp_name, uint32_t *otp_val)
{
	uint32_t otp_idx;

	assert(otp_name != NULL);
	assert(otp_val != NULL);

	if (stm32_get_otp_index(otp_name, &otp_idx, NULL) != 0) {
		return -1;
	}

	if (stm32_get_otp_value_from_idx(otp_idx, otp_val) != 0) {
		ERROR("BSEC: %s Read Error\n", otp_name);
		return -1;
	}

	return 0;
}

int stm32_get_otp_value_from_idx(const uint32_t otp_idx, uint32_t *otp_val)
{
	int ret = BSEC_NOT_SUPPORTED;

	assert(otp_val != NULL);

#if defined(IMAGE_BL2)
	ret = bsec_shadow_read_otp(otp_val, otp_idx);
#elif defined(IMAGE_BL32)
	ret = bsec_read_otp(otp_val, otp_idx);
#else
#error "Not supported"
#endif
	if (ret != BSEC_OK) {
		ERROR("BSEC: idx=%d Read Error\n", otp_idx);
		return -1;
	}

	return 0;
}

uint32_t stm32mp_get_chip_version(void)
{
	uint32_t version = 0U;

	if (stm32mp1_dbgmcu_get_chip_version(&version) < 0) {
		INFO("Cannot get CPU version, debug disabled\n");
		return 0U;
	}

	return version;
}

uint32_t stm32mp_get_chip_dev_id(void)
{
	uint32_t dev_id;

	if (stm32mp1_dbgmcu_get_chip_dev_id(&dev_id) < 0) {
		INFO("Use default chip ID, debug disabled\n");
		dev_id = STM32MP1_CHIP_ID;
	}

	return dev_id;
}

static uint32_t get_part_number(void)
{
	static uint32_t part_number;

	if (part_number != 0U) {
		return part_number;
	}

	if (stm32_get_otp_value(PART_NUMBER_OTP, &part_number) != 0) {
		panic();
	}

	part_number = (part_number & PART_NUMBER_OTP_PART_MASK) >>
		PART_NUMBER_OTP_PART_SHIFT;

	part_number |= stm32mp_get_chip_dev_id() << 16;

	return part_number;
}

static uint32_t get_cpu_package(void)
{
	uint32_t package;

	if (stm32_get_otp_value(PACKAGE_OTP, &package) != 0) {
		panic();
	}

	package = (package & PACKAGE_OTP_PKG_MASK) >>
		PACKAGE_OTP_PKG_SHIFT;

	return package;
}

bool stm32mp_supports_cpu_opp(uint32_t opp_id)
{
	uint32_t id;

	switch (opp_id) {
	case PLAT_OPP_ID1:
	case PLAT_OPP_ID2:
		id = opp_id;
		break;
	default:
		return false;
	}

	switch (get_part_number()) {
	case STM32MP157F_PART_NB:
	case STM32MP157D_PART_NB:
	case STM32MP153F_PART_NB:
	case STM32MP153D_PART_NB:
	case STM32MP151F_PART_NB:
	case STM32MP151D_PART_NB:
		return true;
	default:
		return id == PLAT_OPP_ID1;
	}
}

void stm32mp_get_soc_name(char name[STM32_SOC_NAME_SIZE])
{
	char *cpu_s, *cpu_r, *pkg;

	/* MPUs Part Numbers */
	switch (get_part_number()) {
	case STM32MP157C_PART_NB:
		cpu_s = "157C";
		break;
	case STM32MP157A_PART_NB:
		cpu_s = "157A";
		break;
	case STM32MP153C_PART_NB:
		cpu_s = "153C";
		break;
	case STM32MP153A_PART_NB:
		cpu_s = "153A";
		break;
	case STM32MP151C_PART_NB:
		cpu_s = "151C";
		break;
	case STM32MP151A_PART_NB:
		cpu_s = "151A";
		break;
	case STM32MP157F_PART_NB:
		cpu_s = "157F";
		break;
	case STM32MP157D_PART_NB:
		cpu_s = "157D";
		break;
	case STM32MP153F_PART_NB:
		cpu_s = "153F";
		break;
	case STM32MP153D_PART_NB:
		cpu_s = "153D";
		break;
	case STM32MP151F_PART_NB:
		cpu_s = "151F";
		break;
	case STM32MP151D_PART_NB:
		cpu_s = "151D";
		break;
	default:
		cpu_s = "????";
		break;
	}

	/* Package */
	switch (get_cpu_package()) {
	case PKG_AA_LFBGA448:
		pkg = "AA";
		break;
	case PKG_AB_LFBGA354:
		pkg = "AB";
		break;
	case PKG_AC_TFBGA361:
		pkg = "AC";
		break;
	case PKG_AD_TFBGA257:
		pkg = "AD";
		break;
	default:
		pkg = "??";
		break;
	}

	/* REVISION */
	switch (stm32mp_get_chip_version()) {
	case STM32MP1_REV_B:
		cpu_r = "B";
		break;
	case STM32MP1_REV_Z:
		cpu_r = "Z";
		break;
	default:
		cpu_r = "?";
		break;
	}

	snprintf(name, STM32_SOC_NAME_SIZE,
		 "STM32MP%s%s Rev.%s", cpu_s, pkg, cpu_r);
}

void stm32mp_print_cpuinfo(void)
{
	char name[STM32_SOC_NAME_SIZE];

	stm32mp_get_soc_name(name);
	NOTICE("CPU: %s\n", name);
}

void stm32mp_print_boardinfo(void)
{
	uint32_t board_id = 0;

	if (stm32_get_otp_value(BOARD_ID_OTP, &board_id) != 0) {
		return;
	}

	if (board_id != 0U) {
		char rev[2];

		rev[0] = BOARD_ID2REV(board_id) - 1 + 'A';
		rev[1] = '\0';
		NOTICE("Board: MB%04x Var%u.%u Rev.%s-%02u\n",
		       BOARD_ID2NB(board_id),
		       BOARD_ID2VARCPN(board_id),
		       BOARD_ID2VARFG(board_id),
		       rev,
		       BOARD_ID2BOM(board_id));
	}
}

/* Return true when SoC provides a single Cortex-A7 core, and false otherwise */
bool stm32mp_is_single_core(void)
{
	switch (get_part_number()) {
	case STM32MP151A_PART_NB:
	case STM32MP151C_PART_NB:
	case STM32MP151D_PART_NB:
	case STM32MP151F_PART_NB:
		return true;
	default:
		return false;
	}
}

/* Return true when device is in closed state */
bool stm32mp_is_closed_device(void)
{
	uint32_t value;

	if (stm32_get_otp_value(CFG0_OTP, &value) != 0) {
		return true;
	}

	return (value & CFG0_CLOSED_DEVICE) == CFG0_CLOSED_DEVICE;
}

/* Return true when device supports secure boot */
bool stm32mp_is_auth_supported(void)
{
	switch (get_part_number()) {
	case STM32MP151C_PART_NB:
	case STM32MP151F_PART_NB:
	case STM32MP153C_PART_NB:
	case STM32MP153F_PART_NB:
	case STM32MP157C_PART_NB:
	case STM32MP157F_PART_NB:
		return true;
	default:
		return false;
	}
}

uint32_t stm32_iwdg_get_instance(uintptr_t base)
{
	switch (base) {
	case IWDG1_BASE:
		return IWDG1_INST;
	case IWDG2_BASE:
		return IWDG2_INST;
	default:
		panic();
	}
}

uint32_t stm32_iwdg_get_otp_config(uint32_t iwdg_inst)
{
	uint32_t iwdg_cfg = 0U;
	uint32_t otp_value;

	if (stm32_get_otp_value(HW2_OTP, &otp_value) != 0) {
		panic();
	}

	if ((otp_value & BIT(iwdg_inst + HW2_OTP_IWDG_HW_POS)) != 0U) {
		iwdg_cfg |= IWDG_HW_ENABLED;
	}

	if ((otp_value & BIT(iwdg_inst + HW2_OTP_IWDG_FZ_STOP_POS)) != 0U) {
		iwdg_cfg |= IWDG_DISABLE_ON_STOP;
	}

	if ((otp_value & BIT(iwdg_inst + HW2_OTP_IWDG_FZ_STANDBY_POS)) != 0U) {
		iwdg_cfg |= IWDG_DISABLE_ON_STANDBY;
	}

	return iwdg_cfg;
}

#if defined(IMAGE_BL2)
uint32_t stm32_iwdg_shadow_update(uint32_t iwdg_inst, uint32_t flags)
{
	uint32_t otp_value;
	uint32_t otp;
	uint32_t result;

	if (stm32_get_otp_index(HW2_OTP, &otp, NULL) != 0) {
		panic();
	}

	if (stm32_get_otp_value(HW2_OTP, &otp_value) != 0) {
		panic();
	}

	if ((flags & IWDG_DISABLE_ON_STOP) != 0) {
		otp_value |= BIT(iwdg_inst + HW2_OTP_IWDG_FZ_STOP_POS);
	}

	if ((flags & IWDG_DISABLE_ON_STANDBY) != 0) {
		otp_value |= BIT(iwdg_inst + HW2_OTP_IWDG_FZ_STANDBY_POS);
	}

	result = bsec_write_otp(otp_value, otp);
	if (result != BSEC_OK) {
		return result;
	}

	/* Sticky lock OTP_IWDG (read and write) */
	if ((bsec_set_sr_lock(otp) != BSEC_OK) ||
	    (bsec_set_sw_lock(otp) != BSEC_OK)) {
		return BSEC_LOCK_FAIL;
	}

	return BSEC_OK;
}
#endif

/*
 * This function allows to split bindings between platform and ETZPC
 * HW mapping. If this conversion was done at driver level, the driver
 * should include all supported platform bindings. ETZPC may be used on
 * other platforms.
 */
enum etzpc_decprot_attributes stm32mp_etzpc_binding2decprot(uint32_t mode)
{
	switch (mode) {
	case DECPROT_S_RW:
		return ETZPC_DECPROT_S_RW;
	case DECPROT_NS_R_S_W:
		return ETZPC_DECPROT_NS_R_S_W;
	case DECPROT_MCU_ISOLATION:
		return ETZPC_DECPROT_MCU_ISOLATION;
	case DECPROT_NS_RW:
		return ETZPC_DECPROT_NS_RW;
	default:
		panic();
	}
}

#if STM32MP_USE_STM32IMAGE
/* Get the non-secure DDR size */
uint32_t stm32mp_get_ddr_ns_size(void)
{
	static uint32_t ddr_ns_size;
	uint32_t ddr_size;

	if (ddr_ns_size != 0U) {
		return ddr_ns_size;
	}

	ddr_size = dt_get_ddr_size();
	if ((ddr_size <= STM32MP_DDR_S_SIZE) || (ddr_size > STM32MP_DDR_MAX_SIZE)) {
		panic();
	}

#if defined(AARCH32_SP_OPTEE)
	ddr_ns_size = ddr_size - (STM32MP_DDR_S_SIZE + STM32MP_DDR_SHMEM_SIZE);
#else
	ddr_ns_size = ddr_size;
#endif

	return ddr_ns_size;
}
#endif

bool stm32mp1_addr_inside_backupsram(uintptr_t addr)
{
	return (addr >= STM32MP_BACKUP_RAM_BASE) &&
		(addr < (STM32MP_BACKUP_RAM_BASE + STM32MP_BACKUP_RAM_SIZE));
}

bool stm32mp1_is_wakeup_from_standby(void)
{
	uint32_t rstsr = mmio_read_32(stm32mp_rcc_base() + RCC_MP_RSTSCLRR);
	uint32_t bkpr_core1_addr = tamp_bkpr(BOOT_API_CORE1_BRANCH_ADDRESS_TAMP_BCK_REG_IDX);
	uint32_t nsec_address;

	if ((rstsr & RCC_MP_RSTSCLRR_PADRSTF) != 0U) {
		return false;
	}

	if (stm32mp_get_boot_action() != BOOT_API_CTX_BOOT_ACTION_WAKEUP_STANDBY) {
		return false;
	}

	clk_enable(RTCAPB);
	nsec_address = mmio_read_32(bkpr_core1_addr);
	clk_disable(RTCAPB);

	if (nsec_address == 0U) {
		return false;
	}

	return stm32_pm_context_is_valid();
}

int stm32_save_boot_interface(uint32_t interface, uint32_t instance)
{
	uint32_t bkpr_itf_idx = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

	clk_enable(RTCAPB);

	mmio_clrsetbits_32(bkpr_itf_idx,
			   TAMP_BOOT_ITF_MASK,
			   ((interface << 4) | (instance & 0xFU)) <<
			   TAMP_BOOT_ITF_SHIFT);

	clk_disable(RTCAPB);

	return 0;
}

int stm32_get_boot_interface(uint32_t *interface, uint32_t *instance)
{
	static uint32_t itf;

	if (itf == 0U) {
		uint32_t bkpr = tamp_bkpr(TAMP_BOOT_ITF_BACKUP_REG_ID);

		clk_enable(RTCAPB);

		itf = (mmio_read_32(bkpr) & TAMP_BOOT_ITF_MASK) >> TAMP_BOOT_ITF_SHIFT;

		clk_disable(RTCAPB);
	}

	*interface = itf >> 4;
	*instance = itf & 0xFU;

	return 0;
}

bool stm32_boot_is_serial(void)
{
	uint32_t boot_itf;
	uint32_t boot_instance;

	stm32_get_boot_interface(&boot_itf, &boot_instance);

	if ((boot_itf == BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART) ||
	    (boot_itf == BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB)) {
		return true;
	}

	return false;
}

#if defined(IMAGE_BL32) && DEBUG
static const char *const dump_table[] = {
	"sp_usr  ",
	"lr_usr  ",
	"spsr_irq",
	"sp_irq  ",
	"lr_irq  ",
	"spsr_fiq",
	"sp_fiq  ",
	"lr_fiq  ",
	"spsr_svc",
	"sp_svc  ",
	"lr_svc  ",
	"spsr_abt",
	"sp_abt  ",
	"lr_abt  ",
	"spsr_und",
	"sp_und  ",
	"lr_und  ",
	"spsr_mon",
	"sp_mon",
	"lr_mon",
	"scr",
	"pmcr",
};

/*
 * Dump CPU registers when entering in monitor.
 */
void stm32mp_dump_core_registers(bool fcore)
{
	static bool firstcore;
	unsigned int i;
	smc_ctx_t *ctx = smc_get_ctx(NON_SECURE);
	uint32_t *reg = (uint32_t *)&ctx->sp_usr;

	if (fcore) {
		firstcore = true;
	}

	if (!firstcore) {
		return;
	}

	INFO("CPU : %i\n", plat_my_core_pos());

	for (i = 0U; i < ARRAY_SIZE(dump_table); i++) {
		INFO("%s : 0x%x\n", dump_table[i], reg[i]);
	}
}
#endif
