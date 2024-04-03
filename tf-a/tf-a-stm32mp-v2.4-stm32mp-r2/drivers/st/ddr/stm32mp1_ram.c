/*
 * Copyright (C) 2018-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <common/fdt_wrappers.h>
#include <drivers/clk.h>
#include <drivers/st/stm32mp_ddr_test.h>
#include <drivers/st/stm32mp1_ddr.h>
#include <drivers/st/stm32mp1_ddr_helpers.h>
#include <drivers/st/stm32mp1_ram.h>
#include <lib/mmio.h>

static struct ddr_info ddr_priv_data;
static bool ddr_self_refresh;

int stm32mp1_ddr_clk_enable(struct ddr_info *priv, uint32_t mem_speed)
{
	unsigned long ddrphy_clk, ddr_clk, mem_speed_hz;

	ddr_enable_clock();

	ddrphy_clk = clk_get_rate(DDRPHYC);

	VERBOSE("DDR: mem_speed (%d kHz), RCC %ld kHz\n",
		mem_speed, ddrphy_clk / 1000U);

	mem_speed_hz = mem_speed * 1000U;

	/* Max 10% frequency delta */
	if (ddrphy_clk > mem_speed_hz) {
		ddr_clk = ddrphy_clk - mem_speed_hz;
	} else {
		ddr_clk = mem_speed_hz - ddrphy_clk;
	}
	if (ddr_clk > (mem_speed_hz / 10)) {
		ERROR("DDR expected freq %d kHz, current is %ld kHz\n",
		      mem_speed, ddrphy_clk / 1000U);
		return -1;
	}
	return 0;
}

static int stm32mp1_ddr_setup(void)
{
	struct ddr_info *priv = &ddr_priv_data;
	int ret;
	struct stm32mp1_ddr_config config;
	int node, len;
	uint32_t uret, idx;
	void *fdt;

#define PARAM(x, y, z)							\
	{								\
		.name = x,						\
		.offset = offsetof(struct stm32mp1_ddr_config, y),	\
		.size = sizeof(config.y) / sizeof(uint32_t),		\
		.present = z						\
	}

#define CTL_PARAM(x) PARAM("st,ctl-"#x, c_##x, NULL)
#define PHY_PARAM(x) PARAM("st,phy-"#x, p_##x, NULL)
#define PHY_PARAM_OPT(x) PARAM("st,phy-"#x, p_##x, &config.p_##x##_present)

	const struct {
		const char *name; /* Name in DT */
		const uint32_t offset; /* Offset in config struct */
		const uint32_t size;   /* Size of parameters */
		bool * const present;  /* presence indication for opt */
	} param[] = {
		CTL_PARAM(reg),
		CTL_PARAM(timing),
		CTL_PARAM(map),
		CTL_PARAM(perf),
		PHY_PARAM(reg),
		PHY_PARAM(timing),
		PHY_PARAM_OPT(cal)
	};

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node = fdt_node_offset_by_compatible(fdt, -1, DT_DDR_COMPAT);
	if (node < 0) {
		ERROR("%s: Cannot read DDR node in DT\n", __func__);
		return -EINVAL;
	}

	ret = fdt_read_uint32(fdt, node, "st,mem-speed", &config.info.speed);
	if (ret < 0) {
		VERBOSE("%s: no st,mem-speed\n", __func__);
		return -EINVAL;
	}
	ret = fdt_read_uint32(fdt, node, "st,mem-size", &config.info.size);
	if (ret < 0) {
		VERBOSE("%s: no st,mem-size\n", __func__);
		return -EINVAL;
	}
	config.info.name = fdt_getprop(fdt, node, "st,mem-name", &len);
	if (config.info.name == NULL) {
		VERBOSE("%s: no st,mem-name\n", __func__);
		return -EINVAL;
	}
	INFO("RAM: %s\n", config.info.name);

	for (idx = 0; idx < ARRAY_SIZE(param); idx++) {
		/* Check present field to avoid warning for optional properties */
		if (param[idx].present != NULL) {
			if (fdt_getprop(fdt, node, param[idx].name, NULL) == NULL) {
				*(param[idx].present) = false;
				VERBOSE("%s: %s[0x%x] = absent\n", __func__,
					param[idx].name, param[idx].size);
				continue;
			}
			/* Save presence of optional parameters */
			*(param[idx].present) = true;
		}
		ret = fdt_read_uint32_array(fdt, node, param[idx].name,
					    param[idx].size,
					    (void *)((uintptr_t)&config +
						     param[idx].offset));

		VERBOSE("%s: %s[0x%x] = %d\n", __func__,
			param[idx].name, param[idx].size, ret);
		if (ret != 0) {
			ERROR("%s: Cannot read %s, error=%d\n",
			      __func__, param[idx].name, ret);
			return -EINVAL;
		}
	}

	config.self_refresh = false;

	if (stm32mp1_is_wakeup_from_standby()) {
		config.self_refresh = true;
		config.zdata = stm32_get_zdata_from_context();
	}

	/* Disable axidcg clock gating during init */
	mmio_clrbits_32(priv->rcc + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	stm32mp1_ddr_init(priv, &config);

	/* Enable axidcg clock gating */
	mmio_setbits_32(priv->rcc + RCC_DDRITFCR, RCC_DDRITFCR_AXIDCGEN);

	priv->info.size = config.info.size;

	VERBOSE("%s : ram size(%x, %x)\n", __func__,
		(uint32_t)priv->info.base, (uint32_t)priv->info.size);

	if (stm32mp_map_ddr_non_cacheable() != 0) {
		panic();
	}

	if (config.self_refresh) {
		uret = stm32mp_ddr_test_rw_access();
		if (uret != 0U) {
			ERROR("DDR rw test: Can't access memory @ 0x%x\n",
			      uret);
			panic();
		}

		/* Restore area overwritten by training */
		stm32_restore_ddr_training_area();
	} else {
		uret = stm32mp_ddr_test_data_bus();
		if (uret != 0U) {
			ERROR("DDR data bus test: can't access memory @ 0x%x\n",
			      uret);
			panic();
		}

		uret = stm32mp_ddr_test_addr_bus(config.info.size);
		if (uret != 0U) {
			ERROR("DDR addr bus test: can't access memory @ 0x%x\n",
			      uret);
			panic();
		}

		uret = stm32mp_ddr_check_size();
		if (uret < config.info.size) {
			ERROR("DDR size: 0x%x does not match DT config: 0x%x\n",
			      uret, config.info.size);
			panic();
		}
	}

	/*
	 * Initialization sequence has configured DDR registers with settings.
	 * The Self Refresh (SR) mode corresponding to these settings has now
	 * to be set.
	 */
	ddr_set_sr_mode(ddr_read_sr_mode());

	if (stm32mp_unmap_ddr() != 0) {
		panic();
	}

	/* Save DDR self_refresh state */
	ddr_self_refresh = config.self_refresh;

	return 0;
}

bool stm32mp1_ddr_is_restored(void)
{
	return ddr_self_refresh;
}

int stm32mp1_ddr_probe(void)
{
	struct ddr_info *priv = &ddr_priv_data;

	VERBOSE("STM32MP DDR probe\n");

	priv->ctl = (struct stm32mp1_ddrctl *)stm32mp_ddrctrl_base();
	priv->phy = (struct stm32mp1_ddrphy *)stm32mp_ddrphyc_base();
	priv->pwr = stm32mp_pwr_base();
	priv->rcc = stm32mp_rcc_base();

	priv->info.base = STM32MP_DDR_BASE;
	priv->info.size = 0;

	return stm32mp1_ddr_setup();
}
