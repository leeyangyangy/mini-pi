/*
 * Copyright (c) 2016-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/bl_common.h>
#include <common/desc_image_load.h>
#include <drivers/clk.h>
#include <lib/mmio.h>
#include <plat/common/platform.h>

/*******************************************************************************
 * This function flushes the data structures so that they are visible
 * in memory for the next BL image.
 ******************************************************************************/
void plat_flush_next_bl_params(void)
{
	flush_bl_params_desc();
}

/*******************************************************************************
 * This function returns the list of loadable images.
 ******************************************************************************/
bl_load_info_t *plat_get_bl_image_load_info(void)
{
#if STM32MP_USE_STM32IMAGE
	bl_mem_params_node_t *bl33 = get_bl_mem_params_node(BL33_IMAGE_ID);
	uint32_t ddr_ns_size = stm32mp_get_ddr_ns_size();

	/*
	 * If going back from CSTANDBY / STANDBY and DDR was in Self-Refresh,
	 * BL33 must not be loaded as it would overwrite the code already
	 * in DDR. For this, the BL33 part of the bl_mem_params_desc_ptr
	 * struct should be modified to skip its loading
	 */
	if (stm32mp1_is_wakeup_from_standby()) {
		bl_mem_params_node_t *bl32;

		bl33->image_info.h.attr |= IMAGE_ATTRIB_SKIP_LOADING;
		bl32 = get_bl_mem_params_node(BL32_IMAGE_ID);
		bl32->image_info.h.attr |= IMAGE_ATTRIB_SKIP_LOADING;
#if defined(AARCH32_SP_OPTEE)
		bl32->ep_info.pc = stm32_pm_get_optee_ep();

		if (stm32mp1_addr_inside_backupsram(bl32->ep_info.pc)) {
			clk_enable(BKPSRAM);
		}
#else
		/* Set ep_info PC to 0, to inform BL32 it is a reset after STANDBY */
		bl33->ep_info.pc = 0;
#endif
	}

	/* Max size is non-secure DDR end address minus image_base */
	bl33->image_info.image_max_size = STM32MP_DDR_BASE + ddr_ns_size -
					  bl33->image_info.image_base;
#endif /* STM32MP_USE_STM32IMAGE */

	return get_bl_load_info_from_mem_params_desc();
}

/*******************************************************************************
 * This function returns the list of executable images.
 ******************************************************************************/
bl_params_t *plat_get_next_bl_params(void)
{
	bl_params_t *bl_params = get_next_bl_params_from_mem_params_desc();

	populate_next_bl_params_config(bl_params);

	return bl_params;
}
