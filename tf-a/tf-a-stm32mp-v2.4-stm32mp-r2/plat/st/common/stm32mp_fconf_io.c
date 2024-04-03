/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <common/debug.h>
#include <common/fdt_wrappers.h>
#include <drivers/io/io_storage.h>
#include <drivers/mmc.h>
#include <lib/fconf/fconf.h>
#include <lib/object_pool.h>
#include <libfdt.h>
#include <tools_share/firmware_image_package.h>

#include <platform_def.h>

#include <stm32mp_io_storage.h>
#include <stm32mp_fconf_getter.h>

#if STM32MP_SDMMC || STM32MP_EMMC
static io_block_spec_t gpt_block_spec = {
	.offset = 0,
	.length = 34 * MMC_BLOCK_SIZE, /* Size of GPT table */
};
#endif

/* By default, STM32 platforms load images from the FIP */
struct plat_io_policy policies[MAX_NUMBER_IDS] = {
	[FIP_IMAGE_ID] = {
		&storage_dev_handle,
		(uintptr_t)&image_block_spec,
		open_storage
	},
#if STM32MP_SDMMC || STM32MP_EMMC
	[GPT_IMAGE_ID] = {
		&storage_dev_handle,
		(uintptr_t)&gpt_block_spec,
		open_storage
	},
#endif
};

#if TRUSTED_BOARD_BOOT
#define FCONF_ST_IO_UUID_NUMBER	U(14)
#else
#define FCONF_ST_IO_UUID_NUMBER	U(8)
#endif

static io_uuid_spec_t fconf_stm32mp_uuids[FCONF_ST_IO_UUID_NUMBER];
static OBJECT_POOL_ARRAY(fconf_stm32mp_uuids_pool, fconf_stm32mp_uuids);

struct policies_load_info {
	unsigned int image_id;
	const char *name;
};

/* image id to property name table */
static const struct policies_load_info load_info[FCONF_ST_IO_UUID_NUMBER] = {
	{FW_CONFIG_ID, "fw_cfg_uuid"},
	{BL32_IMAGE_ID, "bl32_uuid"},
	{BL32_EXTRA1_IMAGE_ID, "bl32_extra1_uuid"},
	{BL32_EXTRA2_IMAGE_ID, "bl32_extra2_uuid"},
	{BL33_IMAGE_ID, "bl33_uuid"},
	{HW_CONFIG_ID, "hw_cfg_uuid"},
	{TOS_FW_CONFIG_ID, "tos_fw_cfg_uuid"},
	{NT_FW_CONFIG_ID, "nt_fw_cfg_uuid"},
#if TRUSTED_BOARD_BOOT
	{TRUSTED_BOOT_FW_CERT_ID, "t_boot_fw_cert_uuid"},
	{TRUSTED_KEY_CERT_ID, "t_key_cert_uuid"},
	{TRUSTED_OS_FW_KEY_CERT_ID, "tos_fw_key_cert_uuid"},
	{NON_TRUSTED_FW_KEY_CERT_ID, "nt_fw_key_cert_uuid"},
	{TRUSTED_OS_FW_CONTENT_CERT_ID, "tos_fw_content_cert_uuid"},
	{NON_TRUSTED_FW_CONTENT_CERT_ID, "nt_fw_content_cert_uuid"},
#endif /* TRUSTED_BOARD_BOOT */
};

int fconf_populate_stm32mp_io_policies(uintptr_t config)
{
	int node;
	unsigned int i;

	/* As libfdt uses void *, we can't avoid this cast */
	const void *dtb = (void *)config;

	/* Assert the node offset point to "st,io-fip-handle" compatible property */
	const char *compatible_str = "st,io-fip-handle";

	node = fdt_node_offset_by_compatible(dtb, -1, compatible_str);
	if (node < 0) {
		ERROR("FCONF: Can't find %s compatible in dtb\n", compatible_str);
		return node;
	}

	/* Locate the uuid cells and read the value for all the load info uuid */
	for (i = 0U; i < FCONF_ST_IO_UUID_NUMBER; i++) {
		union uuid_helper_t uuid_helper;
		io_uuid_spec_t *uuid_ptr;
		int err;
		unsigned int j;

		uuid_ptr = pool_alloc(&fconf_stm32mp_uuids_pool);
		err = fdt_read_uint32_array(dtb, node, load_info[i].name,
					    4, uuid_helper.word);
		if (err < 0) {
			WARN("FCONF: Read cell failed for %s\n", load_info[i].name);
			return err;
		}

		/* Convert uuid from big endian to little endian */
		for (j = 0U; j < 4U; j++) {
			uuid_helper.word[j] =
				((uuid_helper.word[j] >> 24U) & 0xff) |
				((uuid_helper.word[j] << 8U) & 0xff0000) |
				((uuid_helper.word[j] >> 8U) & 0xff00) |
				((uuid_helper.word[j] << 24U) & 0xff000000);
		}

		VERBOSE("FCONF: stm32mp-io_policies.%s found with value = 0x%x 0x%x 0x%x 0x%x\n",
			load_info[i].name,
			uuid_helper.word[0], uuid_helper.word[1],
			uuid_helper.word[2], uuid_helper.word[3]);

		uuid_ptr->uuid = uuid_helper.uuid_struct;
		policies[load_info[i].image_id].image_spec = (uintptr_t)uuid_ptr;
		policies[load_info[i].image_id].dev_handle = &fip_dev_handle;
		policies[load_info[i].image_id].check = open_fip;
	}

	return 0;
}

FCONF_REGISTER_POPULATOR(TB_FW, stm32mp_io, fconf_populate_stm32mp_io_policies);
