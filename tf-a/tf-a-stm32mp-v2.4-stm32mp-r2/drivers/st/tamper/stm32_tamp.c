/*
 * Copyright (c) 2018-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <drivers/arm/gicv2.h>
#include <drivers/clk.h>
#include <drivers/st/stm32_gpio.h>
#include <drivers/st/stm32mp_reset.h>
#include <drivers/st/stm32_rng.h>
#include <drivers/st/stm32_rtc.h>
#include <drivers/st/stm32_tamp.h>
#include <lib/mmio.h>

#define DT_TAMP_COMPAT			"st,stm32-tamp"
/* STM32 Registers */
#define _TAMP_CR1			0x00U
#define _TAMP_CR2			0x04U
#define _TAMP_FLTCR			0x0CU
#define _TAMP_ATCR1			0x10U
#define _TAMP_ATSEEDR			0x14U
#define _TAMP_ATOR			0x18U
#define _TAMP_SMCR			0x20U
#define _TAMP_IER			0x2CU
#define _TAMP_SR			0x30U
#define _TAMP_MISR			0x34U
#define _TAMP_SMISR			0x38U
#define _TAMP_SCR			0x3CU
#define _TAMP_COUNTR			0x40U
#define _TAMP_OR			0x50U
#define _TAMP_HWCFGR2			0x3ECU
#define _TAMP_HWCFGR1			0x3F0U
#define _TAMP_VERR			0x3F4U
#define _TAMP_IPIDR			0x3F8U
#define _TAMP_SIDR			0x3FCU

/* _TAMP_CR1 bit filds */
#define _TAMP_CR1_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_CR1_ETAMP(id)		BIT((id))

/* _TAMP_CR2 bit filds */
#define _TAMP_CR2_ETAMPTRG(id)		BIT((id) + 24U)
#define _TAMP_CR2_ETAMPMSK_MAX_ID	3U
#define _TAMP_CR2_ETAMPMSK(id)		BIT((id) + 16U)
#define _TAMP_CR2_ETAMPNOER(id)		BIT((id))

/* _TAMP_FLTCR bit fields */
#define _TAMP_FLTCR_TAMPFREQ		GENMASK(2, 0)
#define _TAMP_FLTCR_TAMPFLT		GENMASK(4, 3)
#define _TAMP_FLTCR_TAMPPRCH		GENMASK(6, 5)
#define _TAMP_FLTCR_TAMPPUDIS		BIT(7)

/* _TAMP_ATCR bit fields */
#define _TAMP_ATCR1_ATCKSEL		GENMASK(18, 16)
#define _TAMP_ATCR1_ATPER		GENMASK(26, 24)
#define _TAMP_ATCR1_COMMON_MASK		GENMASK(31, 16)
#define _TAMP_ATCR1_ETAMPAM(id)		BIT((id))
#define _TAMP_ATCR1_ATOSEL_MASK(i)	GENMASK(((i) + 1) * 2U + 7U, (i) * 2U + 8U)
#define _TAMP_ATCR1_ATOSEL(i, o)	(((o) - 1U) << ((i) * 2U + 8U))

/* _TAMP_ATOR bit fields */
#define _TAMP_PRNG			GENMASK(7, 0)
#define _TAMP_SEEDF			BIT(14)
#define _TAMP_INITS			BIT(15)

/* _TAMP_IER bit fields */
#define _TAMP_IER_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_IER_ETAMP(id)		BIT((id))

/* _TAMP_SR bit fields */
#define _TAMP_SR_ETAMPXF_MASK		GENMASK(7, 0)
#define _TAMP_SR_ITAMPXF_MASK		GENMASK(31, 16)
#define _TAMP_SR_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_SR_ETAMP(id)		BIT((id))

/* _TAMP_SCR bit fields */
#define _TAMP_SCR_ITAMP(id)		BIT((id) + 16U)
#define _TAMP_SCR_ETAMP(id)		BIT((id))

/* _TAMP_SMCR bit fields */
#define _TAMP_SMCR_BKPRWDPROT_MASK	GENMASK(7, 0)
#define _TAMP_SMCR_BKPRWDPROT_SHIFT	U(0)
#define _TAMP_SMCR_BKPWDPROT_MASK	GENMASK(23, 16)
#define _TAMP_SMCR_BKPWDPROT_SHIFT	U(16)
#define _TAMP_SMCR_DPROT		BIT(31)

/* _TAMP_OR bit fields */
#define _TAMP_OR_OUT3RMP_PI8		0U
#define _TAMP_OR_OUT3RMP_PC13		BIT(0)

/* _TAMP_HWCFGR2 bit fields */
#define _TAMP_HWCFGR2_TZ		GENMASK(11, 8)
#define _TAMP_HWCFGR2_OR		GENMASK(7, 0)

/* _TAMP_HWCFGR1 bit fields */
#define _TAMP_HWCFGR1_BKPREG		GENMASK(7, 0)
#define _TAMP_HWCFGR1_TAMPER		GENMASK(11, 8)
#define _TAMP_HWCFGR1_ACTIVE		GENMASK(15, 12)
#define _TAMP_HWCFGR1_INTERN		GENMASK(31, 16)
#define _TAMP_HWCFGR1_ITAMP_MAX_ID	16U
#define _TAMP_HWCFGR1_ITAMP(id)		BIT((id) + 16U)

/* _TAMP_VERR bit fields */
#define _TAMP_VERR_MINREV		GENMASK(3, 0)
#define _TAMP_VERR_MAJREV		GENMASK(7, 4)

#define _TAMP_NB_MONOTONIC_COUNTER	0x1

/*
 * Macro to manage bit manipulation when we work on local variable
 * before writing only once to the real register
 */

#define CLRBITS(v, bits)		(v) &= ~(bits)
#define SETBITS(v, bits)		(v) |= (bits)
#define CLRSETBITS(v, mask, bits)	(v) = ((v) & ~(mask)) | (bits)

struct stm32_tamp_int {
	const uint32_t id;
	uint32_t mode;
	int (*func)(int id);
};

struct stm32_tamp_ext {
	const uint32_t id;
	uint32_t mode;
	uint8_t out_pin;
	int (*func)(int id);
};

struct stm32_tamp_instance {
	uintptr_t base;
	uint32_t clock;
	uint32_t hwconf1;
	uint32_t hwconf2;
	uint32_t secret_list_conf;
	uint32_t privilege_conf;
	uint32_t secure_conf;
	uint32_t passive_conf;
	uint32_t active_conf;
	struct stm32_tamp_int int_tamp[PLAT_MAX_TAMP_INT];
	struct stm32_tamp_ext ext_tamp[PLAT_MAX_TAMP_EXT];
};

/* 0 is the expected initial values for all fields but .id */
static struct stm32_tamp_instance stm32_tamp = {
	.int_tamp = {
		{
			.id = INT_TAMP1,
		},
		{
			.id = INT_TAMP2,
		},
		{
			.id = INT_TAMP3,
		},
		{
			.id = INT_TAMP4,
		},
		{
			.id = INT_TAMP5,
		},
		{
			.id = INT_TAMP8,
		},
	},
	.ext_tamp = {
		{
			.id = EXT_TAMP1,
		},
		{
			.id = EXT_TAMP2,
		},
		{
			.id = EXT_TAMP3,
		},
	}
};

static void stm32_tamp_set_secure(unsigned long base, uint32_t mode)
{
	if (mode & TAMP_REGS_IT_SECURE) {
		mmio_clrbits_32(base + _TAMP_SMCR, _TAMP_SMCR_DPROT);
	} else {
		mmio_setbits_32(base + _TAMP_SMCR, _TAMP_SMCR_DPROT);
	}
}

static void stm32_tamp_set_privilege(unsigned long base __unused, uint32_t mode __unused)
{
}

static void stm32_tamp_set_output_pin(unsigned long base, uint32_t out)
{
	mmio_setbits_32(base + _TAMP_OR, out);
}

static int stm32_tamp_set_seed(unsigned long base)
{
	/* Need RNG access. */
	uint32_t timeout = 100;
	uint8_t idx;

	for (idx = 0; idx < 4U; idx++) {
		uint32_t rnd;

		if (stm32_rng_read((uint8_t *)&rnd, sizeof(uint32_t)) != 0) {
			return -1;
		}

		VERBOSE("Seed init %u\n", rnd);
		mmio_write_32(base + _TAMP_ATSEEDR, rnd);
	}

	while (((mmio_read_32(base + _TAMP_ATOR) & _TAMP_SEEDF) != 0U) &&
	       (timeout != 0U)) {
		timeout--;
	}

	if (timeout == 0U) {
		return -1;
	}

	return 0;
}

static bool is_int_tamp_id_valid(uint32_t id)
{
	if (id > _TAMP_HWCFGR1_ITAMP_MAX_ID) {
		return false;
	}

	return (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_ITAMP(id))
		== _TAMP_HWCFGR1_ITAMP(id);
}

static bool is_ext_tamp_id_valid(uint32_t id)
{
	if (id > PLAT_MAX_TAMP_EXT) {
		return false;
	}

	return true;
}

static int stm32_tamp_set_int_config(struct stm32_tamp_int *tamp_int,
				     uint32_t *cr1, uint32_t *cr3, uint32_t *ier)
{
	uint32_t id;

	if (tamp_int == NULL) {
		return -EINVAL;
	}

	id = tamp_int->id;

	if (!is_int_tamp_id_valid(id)) {
		return -EINVAL;
	}

	/* If etamp is disabled */
	if ((tamp_int->mode & TAMP_ENABLE) != TAMP_ENABLE) {
		CLRBITS(*cr1, _TAMP_CR1_ITAMP(id));
		CLRBITS(*ier, _TAMP_IER_ITAMP(id));
		return 0;
	}

	SETBITS(*cr1, _TAMP_CR1_ITAMP(id));
	SETBITS(*ier, _TAMP_IER_ITAMP(id));

	return 0;
}

static int stm32_tamp_set_ext_config(struct stm32_tamp_ext *tamp_ext,
				     uint32_t *cr1, uint32_t *cr2,
				     uint32_t *atcr1, uint32_t *atcr2, uint32_t *ier)
{
	uint32_t id;

	if (tamp_ext == NULL) {
		return -EINVAL;
	}

	id = tamp_ext->id;

	/* Exit if not a valid TAMP_ID */
	if (!is_ext_tamp_id_valid(id)) {
		return -EINVAL;
	}

	/* If etamp is disabled */
	if ((tamp_ext->mode & TAMP_ENABLE) != TAMP_ENABLE) {
		CLRBITS(*cr1, _TAMP_CR1_ETAMP(id));
		CLRBITS(*cr2, _TAMP_CR2_ETAMPMSK(id));
		CLRBITS(*cr2, _TAMP_CR2_ETAMPTRG(id));
		CLRBITS(*cr2, _TAMP_CR2_ETAMPNOER(id));
		CLRBITS(*ier, _TAMP_IER_ETAMP(id));
		return 0;
	}

	SETBITS(*cr1, _TAMP_CR1_ETAMP(id));

	if ((tamp_ext->mode & TAMP_TRIG_ON) == TAMP_TRIG_ON) {
		SETBITS(*cr2, _TAMP_CR2_ETAMPTRG(id));
	} else {
		CLRBITS(*cr2, _TAMP_CR2_ETAMPTRG(id));
	}

	if ((tamp_ext->mode & TAMP_ACTIVE) == TAMP_ACTIVE) {
		SETBITS(*cr1, _TAMP_ATCR1_ETAMPAM(id));
	} else {
		CLRBITS(*cr1, _TAMP_ATCR1_ETAMPAM(id));
	}

	if ((tamp_ext->mode & TAMP_NOERASE) == TAMP_NOERASE) {
		SETBITS(*cr2, _TAMP_CR2_ETAMPNOER(id));
	} else {
		CLRBITS(*cr2, _TAMP_CR2_ETAMPNOER(id));
	}

	/* Configure output pin:
	 * For the case out_pin = 0, we select same output pin than the input one.
	 */
	if (tamp_ext->out_pin == TAMPOUTSEL_SAME_AS_INPUT) {
		tamp_ext->out_pin = id + 1;
	}

	if (tamp_ext->out_pin < PLAT_MAX_TAMP_EXT + 1U) {
		CLRSETBITS(*atcr1, _TAMP_ATCR1_ATOSEL_MASK(id),
			   _TAMP_ATCR1_ATOSEL(id, tamp_ext->out_pin));
	}

	if (id < _TAMP_CR2_ETAMPMSK_MAX_ID) {
		/*
		 * Only external TAMP 1, 2 and 3 can be masked
		 */
		if ((tamp_ext->mode & TAMP_EVT_MASK) == TAMP_EVT_MASK) {
			/*
			 * ETAMP(id) event generates a trigger event. This ETAMP(id) is masked
			 * and internally cleared by hardware. The backup registers are not erased.
			 */
			CLRBITS(*ier, _TAMP_IER_ETAMP(id));
			CLRBITS(*cr2, _TAMP_CR2_ETAMPMSK(id));
		} else {
			/*
			 * normal ETAMP interrupt:
			 * ETAMP(id) event generates a trigger event and TAMP(id)F must be cleared
			 * by software to * allow next tamper event detection.
			 */
			CLRBITS(*cr2, _TAMP_CR2_ETAMPMSK(id));
			CLRBITS(*ier, _TAMP_IER_ETAMP(id));
		}
	} else {
		/* other than 1,2,3 external TAMP, we want its interruption */
		CLRBITS(*ier, _TAMP_IER_ETAMP(id));
	}

	return 0;
}

int stm32_tamp_set_secure_bkpregs(struct bkpregs_conf *bkpregs_conf)
{
	uint32_t first_z2;
	uint32_t first_z3;

	if (bkpregs_conf == NULL) {
		return -EINVAL;
	}

	first_z2 = bkpregs_conf->nb_zone1_regs;
	first_z3 = bkpregs_conf->nb_zone1_regs + bkpregs_conf->nb_zone2_regs;

	if ((first_z2 > (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_BKPREG)) ||
	    (first_z3 > (stm32_tamp.hwconf1 & _TAMP_HWCFGR1_BKPREG))) {
		return -ENODEV;
	}

	mmio_clrsetbits_32(stm32_tamp.base + _TAMP_SMCR,
			   _TAMP_SMCR_BKPRWDPROT_MASK,
			   (first_z2 << _TAMP_SMCR_BKPRWDPROT_SHIFT) &
			   _TAMP_SMCR_BKPRWDPROT_MASK);

	mmio_clrsetbits_32(stm32_tamp.base + _TAMP_SMCR,
			   _TAMP_SMCR_BKPWDPROT_MASK,
			   (first_z3 << _TAMP_SMCR_BKPWDPROT_SHIFT) &
			   _TAMP_SMCR_BKPWDPROT_MASK);
	return 0;
}

int stm32_tamp_set_config(void)
{
	int ret;
	uint32_t i;
	uint32_t cr1 = 0, cr2 = 0, cr3 = 0;
	uint32_t atcr1 = 0, atcr2 = 0;
	uint32_t fltcr = 0;
	uint32_t ier = 0;

	/* Select access in secure or unsecure */
	stm32_tamp_set_secure(stm32_tamp.base, stm32_tamp.secure_conf);

	/* Select acces in privileged mode or unprivileged mode */
	stm32_tamp_set_privilege(stm32_tamp.base, stm32_tamp.privilege_conf);

	if (stm32_tamp.passive_conf != 0U) {
		/* Filter mode register set */
		fltcr = stm32_tamp.passive_conf;
	}

	if (stm32_tamp.active_conf != 0U) {
		/* Active mode configuration */
		CLRSETBITS(atcr1, _TAMP_ATCR1_COMMON_MASK,
			   (stm32_tamp.active_conf & _TAMP_ATCR1_COMMON_MASK));
	}

	for (i = 0U; i < PLAT_MAX_TAMP_INT; i++) {
		ret = stm32_tamp_set_int_config(&(stm32_tamp.int_tamp[i]),
						&cr1, &cr3, &ier);
		if (ret != 0) {
			return ret;
		}
	}

	for (i = 0U; i < PLAT_MAX_TAMP_EXT; i++) {
		ret = stm32_tamp_set_ext_config(&(stm32_tamp.ext_tamp[i]),
						&cr1, &cr2, &atcr1, &atcr2, &ier);
		if (ret != 0) {
			return ret;
		}
	}

	/*
	 * We apply configuration all in a row:
	 * As for active ext tamper "all the needed tampers must be enabled in the same write
	 * access".
	 */
	mmio_write_32(stm32_tamp.base + _TAMP_FLTCR, fltcr);
	/* Active filter configuration applied only if not already done. */
	if (((mmio_read_32(stm32_tamp.base + _TAMP_ATOR) & _TAMP_INITS) != _TAMP_INITS)) {
		mmio_write_32(stm32_tamp.base + _TAMP_ATCR1, atcr1);
	}

	mmio_write_32(stm32_tamp.base + _TAMP_CR1, cr1);
	mmio_write_32(stm32_tamp.base + _TAMP_CR2, cr2);
	/* If active tamper we reinit the seed. */
	if (stm32_tamp.active_conf != 0U) {
		if (stm32_tamp_set_seed(stm32_tamp.base) != 0) {
			ERROR("Active tamper: SEED not initialized\n");
			return -EPERM;
		}
	}

	/* Ack all pending interrupt */
	mmio_write_32(stm32_tamp.base + _TAMP_SCR, ~0U);
	/* Enable interrupts. */
	mmio_write_32(stm32_tamp.base + _TAMP_IER, ier);

	return 0;
}

int stm32_tamp_write_mcounter(int counter_idx)
{
	mmio_write_32(stm32_tamp.base + _TAMP_COUNTR, 1U);

	return 0;
}

uint32_t stm32_tamp_read_mcounter(int counter_idx)
{
	return mmio_read_32(stm32_tamp.base + _TAMP_COUNTR);
}

void stm32_tamp_configure_secret_list(uint32_t secret_list_conf)
{
	stm32_tamp.secret_list_conf = secret_list_conf;
}

void stm32_tamp_configure_privilege_access(uint32_t privilege_conf)
{
	stm32_tamp.privilege_conf = privilege_conf;
}

void stm32_tamp_configure_secure_access(uint32_t secure_conf)
{
	stm32_tamp.secure_conf = secure_conf;
}

void stm32_tamp_configure_passive(uint32_t passive_conf)
{
	stm32_tamp.passive_conf = passive_conf;
}

void stm32_tamp_configure_active(uint32_t active_conf)
{
	stm32_tamp.active_conf = active_conf;
}

int stm32_tamp_configure_internal(enum stm32_tamp_int_id id, uint32_t mode, int (*callback)(int id))
{
	uint32_t i;
	uint32_t itamp_id = id;
	struct stm32_tamp_int *tamp_int = NULL;

	/* Find internal Tamp struct*/
	for (i = 0U; i < PLAT_MAX_TAMP_INT; i++) {
		if (stm32_tamp.int_tamp[i].id == itamp_id) {
			tamp_int = &(stm32_tamp.int_tamp[i]);
			break;
		}
	}

	if (tamp_int == NULL) {
		return -EINVAL;
	}

	tamp_int->mode = mode;
	tamp_int->func = callback;

	return 0;
}

int stm32_tamp_configure_external(enum stm32_tamp_ext_id id, uint32_t mode,
				  enum stm32_tamp_ext_out_id out_pin, int (*callback)(int id))
{
	uint32_t i;
	uint32_t etamp_id = id;
	struct stm32_tamp_ext *tamp_ext = NULL;

	/* Find external Tamp struct */
	for (i = 0U; i < PLAT_MAX_TAMP_EXT; i++) {
		if (stm32_tamp.ext_tamp[i].id == etamp_id) {
			tamp_ext = &(stm32_tamp.ext_tamp[i]);
			break;
		}
	}

	if (tamp_ext == NULL) {
		return -EINVAL;
	}

	tamp_ext->mode = mode;
	tamp_ext->out_pin = out_pin;
	tamp_ext->func = callback;

	return 0;
}

void stm32_tamp_it_handler(void)
{
	uint32_t it = mmio_read_32(stm32_tamp.base + _TAMP_SR);
	uint32_t int_it = it & _TAMP_SR_ITAMPXF_MASK;
	uint32_t ext_it = it & _TAMP_SR_ETAMPXF_MASK;
	uint8_t tamp = 0;
	struct stm32_rtc_time tamp_ts;

	if (stm32_rtc_is_timestamp_enable()) {
		stm32_rtc_get_timestamp(&tamp_ts);
		INFO("Tamper Event Occurred\n");
		INFO("Date : %u/%u\n \t Time : %u:%u:%u\n",
		     tamp_ts.day, tamp_ts.month, tamp_ts.hour,
		     tamp_ts.min, tamp_ts.sec);
	}

	while ((int_it != 0U) && (tamp < PLAT_MAX_TAMP_INT)) {
		int ret = -1;
		uint32_t int_id = stm32_tamp.int_tamp[tamp].id;

		if ((it & _TAMP_SR_ITAMP(int_id)) != 0U) {
			if (stm32_tamp.int_tamp[tamp].func != NULL) {
				ret = stm32_tamp.int_tamp[tamp].func(int_id);
			}

			if (ret >= 0) {
				mmio_setbits_32(stm32_tamp.base + _TAMP_SCR,
						_TAMP_SR_ITAMP(int_id));
				ext_it &= ~_TAMP_SR_ITAMP(int_id);

				if (ret > 0) {
					stm32mp_system_reset();
				}
			}
		}
		tamp++;
	}

	tamp = 0;
	/* External tamper interrupt */
	while ((ext_it != 0U) && (tamp < PLAT_MAX_TAMP_EXT)) {
		int ret = -1;
		uint32_t ext_id = stm32_tamp.ext_tamp[tamp].id;

		if ((ext_it & _TAMP_SR_ETAMP(ext_id)) != 0U) {
			if (stm32_tamp.ext_tamp[tamp].func != NULL) {
				ret = stm32_tamp.ext_tamp[tamp].func(ext_id);
			}

			if (ret >= 0) {
				mmio_setbits_32(stm32_tamp.base + _TAMP_SCR,
						_TAMP_SCR_ETAMP(ext_id));
				ext_it &= ~_TAMP_SR_ETAMP(ext_id);

				if (ret > 0) {
					stm32mp_system_reset();
				}
			}
		}
		tamp++;
	}

	gicv2_end_of_interrupt(STM32MP1_IRQ_TAMPSERRS);
}

int stm32_tamp_init(void)
{
	int node;
	struct dt_node_info dt_tamp;
	void *fdt;
	uint32_t rev __unused;

	if (fdt_get_address(&fdt) == 0) {
		return -EPERM;
	}

	node = dt_get_node(&dt_tamp, -1, DT_TAMP_COMPAT);
	if (node < 0) {
		return -EINVAL;
	}

	assert(dt_tamp.base != 0U);
	assert(dt_tamp.clock != -1);

	stm32_tamp.base = dt_tamp.base;
	stm32_tamp.clock = (uint32_t)dt_tamp.clock;

	/* Init Tamp clock */
	clk_enable(stm32_tamp.clock);

	/* Check if TAMP is enabled */
	if ((dt_tamp.status != DT_SECURE) &&
	    (dt_tamp.status != DT_SHARED)) {
		return 0;
	}

	stm32_tamp.hwconf1 = mmio_read_32(stm32_tamp.base + _TAMP_HWCFGR1);
	stm32_tamp.hwconf2 = mmio_read_32(stm32_tamp.base + _TAMP_HWCFGR2);

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
	rev = mmio_read_32(stm32_tamp.base + _TAMP_VERR);
	VERBOSE("STM32 TAMPER V%u.%u\n", (rev & _TAMP_VERR_MAJREV) >> 4,
		rev & _TAMP_VERR_MINREV);
#endif

	if ((stm32_tamp.hwconf2 & _TAMP_HWCFGR2_TZ) == 0U) {
		ERROR("Tamper IP doesn't support trustzone");
		return -EPERM;
	}

	if (dt_set_pinctrl_config(node) != -FDT_ERR_NOTFOUND) {
		if (fdt_getprop(fdt, node, "st,out3-pc13", NULL) != NULL) {
			stm32_tamp_set_output_pin(stm32_tamp.base, _TAMP_OR_OUT3RMP_PC13);
		}
	}

	if (stm32_gic_enable_spi(node, NULL) < 0) {
		return -EPERM;
	}

	if (fdt_getprop(fdt, node, "wakeup-source", NULL) != NULL) {
		mmio_setbits_32(EXTI_BASE + EXTI_TZENR1, EXTI_TZENR1_TZEN18);
		mmio_setbits_32(EXTI_BASE + EXTI_C1IMR1, EXTI_IMR1_IM18);
	}

	return 1;
}
