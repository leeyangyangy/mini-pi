/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef REGULATOR_H
#define REGULATOR_H

#include <platform_def.h>

#ifndef PLAT_NB_RDEVS
#error "Missing PLAT_NB_RDEVS"
#endif

#ifndef PLAT_NB_SUSPEND_MODES
#error "Missing PLAT_NB_SUSPEND_MODES"
#endif

const char *plat_get_lp_mode_name(int mode);

/*
 * Consumer interface
 */

/* regulator-always-on : regulator should never be disabled */
#define REGUL_ALWAYS_ON		BIT(0)
/*
 * regulator-boot-on:
 * It's expected that this regulator was left on by the bootloader.
 * The core shouldn't prevent it from being turned off later.
 * The regulator is needed to exit from suspend so it is turned on during suspend entry.
 */
#define REGUL_BOOT_ON		BIT(1)
/* regulator-over-current-protection: Enable over current protection. */
#define REGUL_OCP		BIT(2)
/* regulator-active-discharge: enable active discharge. */
#define REGUL_ACTIVE_DISCHARGE	BIT(3)
/* regulator-pull-down: Enable pull down resistor when the regulator is disabled. */
#define REGUL_PULL_DOWN		BIT(4)
/*
 * st,mask-reset: set mask reset for the regulator, meaning that the regulator
 * setting is maintained during pmic reset.
 */
#define REGUL_MASK_RESET	BIT(5)
/* st,regulator-sink-source: set the regulator in sink source mode */
#define REGUL_SINK_SOURCE	BIT(6)
/* st,regulator-bypass: set the regulator in bypass mode */
#define REGUL_ENABLE_BYPASS	BIT(7)

struct rdev *regulator_get_by_name(const char *node_name);

#if defined(IMAGE_BL32)
struct rdev *regulator_get_by_regulator_name(const char *reg_name);
#endif

struct rdev *regulator_get_by_supply_name(const void *fdt, int node, const char *name);

int regulator_enable(struct rdev *rdev);
int regulator_disable(struct rdev *rdev);
int regulator_is_enabled(const struct rdev *rdev);

int regulator_set_voltage(struct rdev *rdev, uint16_t volt);
int regulator_set_min_voltage(struct rdev *rdev);
int regulator_get_voltage(const struct rdev *rdev);

int regulator_list_voltages(const struct rdev *rdev, const uint16_t **levels, size_t *count);
void regulator_get_range(const struct rdev *rdev, uint16_t *min_mv, uint16_t *max_mv);
int regulator_set_flag(struct rdev *rdev, uint16_t flag);

/*
 * Driver Interface
 */

/* set_state() arguments */
#define STATE_DISABLE		false
#define STATE_ENABLE		true

/* suspend() arguments */
#define LP_STATE_OFF		BIT(0)
#define LP_STATE_ON		BIT(1)
#define LP_STATE_UNCHANGED	BIT(2)
#define LP_STATE_SET_VOLT	BIT(3)

struct regul_description {
	const char *node_name;
	const struct regul_ops *ops;
	const void *driver_data;
	const char *supply_name;
	const uint32_t enable_ramp_delay;
};

struct regul_ops {
	int (*set_state)(const struct regul_description *desc, bool state);
	int (*get_state)(const struct regul_description *desc);
	int (*set_voltage)(const struct regul_description *desc, uint16_t mv);
	int (*get_voltage)(const struct regul_description *desc);
	int (*list_voltages)(const struct regul_description *desc,
			     const uint16_t **levels, size_t *count);
	int (*set_flag)(const struct regul_description *desc, uint16_t flag);
#if defined(IMAGE_BL32)
	void (*lock)(const struct regul_description *desc);
	void (*unlock)(const struct regul_description *desc);
	int (*suspend)(const struct regul_description *desc, uint8_t state,
		       uint16_t mv);
#endif
};

int regulator_register(const struct regul_description *desc, int node);

/*
 * Internal regulator structure
 * The structure is internal to the core, and the content should not be used
 * by a consumer nor a driver.
 */
struct rdev {
	const struct regul_description *desc;

	int32_t phandle;

	uint16_t min_mv;
	uint16_t max_mv;

	uint16_t flags;

	uint32_t enable_ramp_delay;
#if defined(IMAGE_BL32)
	const char *reg_name;

	uint8_t use_count;

	int32_t supply_phandle;
	struct rdev *supply_dev;

	uint8_t lp_state[PLAT_NB_SUSPEND_MODES];
	uint16_t lp_mv[PLAT_NB_SUSPEND_MODES];
#endif
};

#if defined(IMAGE_BL32)

/* Boot and init */
int regulator_core_config(void);
int regulator_core_cleanup(void);

/* Suspend resume operations */
#define PLAT_BACKUP_REGULATOR_SIZE (sizeof(int8_t) * PLAT_NB_RDEVS)

int regulator_core_suspend(int mode);
int regulator_core_resume(void);

void regulator_core_backup_context(void *backup_area, size_t backup_size);
void regulator_core_restore_context(void *backup_area, size_t backup_size);

#if LOG_LEVEL >= LOG_LEVEL_VERBOSE
void regulator_core_dump(void);
#endif

#endif

#endif /* REGULATOR_H */
