/*
 * Copyright (c) 2017-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <common/debug.h>
#include <common/fdt_wrappers.h>
#include <drivers/regulator.h>
#include <drivers/st/stm32_gpio.h>

#include <stm32mp_dt.h>

static void *fdt;

/*******************************************************************************
 * This function checks device tree file with its header.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_open_and_check(uintptr_t dt_addr)
{
	int ret;

	ret = fdt_check_header((void *)dt_addr);
	if (ret == 0) {
		fdt = (void *)dt_addr;
	}

	return ret;
}

/*******************************************************************************
 * This function gets the address of the DT.
 * If DT is OK, fdt_addr is filled with DT address.
 * Returns 1 if success, 0 otherwise.
 ******************************************************************************/
int fdt_get_address(void **fdt_addr)
{
	if (fdt == NULL) {
		return 0;
	}

	*fdt_addr = fdt;

	return 1;
}

/*******************************************************************************
 * This function check the presence of a node (generic use of fdt library).
 * Returns true if present, else return false.
 ******************************************************************************/
bool fdt_check_node(int node)
{
	int len;
	const char *cchar;

	cchar = fdt_get_name(fdt, node, &len);

	return (cchar != NULL) && (len >= 0);
}

/*******************************************************************************
 * This function return global node status (generic use of fdt library).
 ******************************************************************************/
uint8_t fdt_get_status(int node)
{
	uint8_t status = DT_DISABLED;
	const char *cchar;

	cchar = fdt_getprop(fdt, node, "status", NULL);
	if ((cchar == NULL) ||
	    (strncmp(cchar, "okay", strlen("okay")) == 0)) {
		status |= DT_NON_SECURE;
	}

	cchar = fdt_getprop(fdt, node, "secure-status", NULL);
	if (cchar == NULL) {
		if (status == DT_NON_SECURE) {
			status |= DT_SECURE;
		}
	} else if (strncmp(cchar, "okay", strlen("okay")) == 0) {
		status |= DT_SECURE;
	}

	return status;
}

#if ENABLE_ASSERTIONS
/*******************************************************************************
 * This function returns the address cells from the node parent.
 * Returns:
 * - #address-cells value if success.
 * - invalid value if error.
 * - a default value if undefined #address-cells property as per libfdt
 *   implementation.
 ******************************************************************************/
static int fdt_get_node_parent_address_cells(int node)
{
	int parent;

	parent = fdt_parent_offset(fdt, node);
	if (parent < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return fdt_address_cells(fdt, parent);
}
#endif

/*******************************************************************************
 * This function return interrupts from node.
 ******************************************************************************/
int fdt_get_interrupt(int node, const fdt32_t **array, int *len, bool *extended)
{
	uint8_t status = fdt_get_status(node);

	*extended = false;

	switch (status) {
	case DT_SECURE:
		*array = fdt_getprop(fdt, node, "interrupts-extended", len);
		if (*array == NULL) {
			*array = fdt_getprop(fdt, node, "interrupts", len);
		} else {
			*extended = true;
		}
		break;

	default:
		*array = fdt_getprop(fdt, node, "secure-interrupts", len);
		break;
	}

	if (*array == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	return 0;
}

/*******************************************************************************
 * This function gets the stdout pin configuration information from the DT.
 * And then calls the sub-function to treat it and set GPIO registers.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_set_stdout_pinctrl(void)
{
	int node;

	node = fdt_get_stdout_node_offset(fdt);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return dt_set_pinctrl_config(node);
}

/*******************************************************************************
 * This function fills the generic information from a given node.
 ******************************************************************************/
void dt_fill_device_info(struct dt_node_info *info, int node)
{
	const fdt32_t *cuint;

	assert(fdt_get_node_parent_address_cells(node) == 1);

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint != NULL) {
		info->base = fdt32_to_cpu(*cuint);
	} else {
		info->base = 0;
	}

	cuint = fdt_getprop(fdt, node, "clocks", NULL);
	if (cuint != NULL) {
		cuint++;
		info->clock = (int)fdt32_to_cpu(*cuint);
	} else {
		info->clock = -1;
	}

	cuint = fdt_getprop(fdt, node, "resets", NULL);
	if (cuint != NULL) {
		cuint++;
		info->reset = (int)fdt32_to_cpu(*cuint);
	} else {
		info->reset = -1;
	}

	info->status = fdt_get_status(node);
}

/*******************************************************************************
 * This function retrieve the generic information from DT.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_node(struct dt_node_info *info, int offset, const char *compat)
{
	int node;

	node = fdt_node_offset_by_compatible(fdt, offset, compat);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	dt_fill_device_info(info, node);

	return node;
}

/*******************************************************************************
 * This function gets the UART instance info of stdout from the DT.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_stdout_uart_info(struct dt_node_info *info)
{
	int node;

	node = fdt_get_stdout_node_offset(fdt);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	dt_fill_device_info(info, node);

	return node;
}

/*******************************************************************************
 * This function returns the node offset matching compatible string in the DT,
 * and also matching the reg property with the given address.
 * Returns value on success, and error value on failure.
 ******************************************************************************/
int dt_match_instance_by_compatible(const char *compatible, uintptr_t address)
{
	int node;

	for (node = fdt_node_offset_by_compatible(fdt, -1, compatible);
	     node != -FDT_ERR_NOTFOUND;
	     node = fdt_node_offset_by_compatible(fdt, node, compatible)) {
		const fdt32_t *cuint;

		assert(fdt_get_node_parent_address_cells(node) == 1);

		cuint = fdt_getprop(fdt, node, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		if ((uintptr_t)fdt32_to_cpu(*cuint) == address) {
			return node;
		}
	}

	return -FDT_ERR_NOTFOUND;
}

/*******************************************************************************
 * This function gets DDR size information from the DT.
 * Returns value in bytes on success, and 0 on failure.
 ******************************************************************************/
uint32_t dt_get_ddr_size(void)
{
	static uint32_t size;
	int node;

	if (size != 0U) {
		return size;
	}

	node = fdt_node_offset_by_compatible(fdt, -1, DT_DDR_COMPAT);
	if (node < 0) {
		return 0;
	}

	size = fdt_read_uint32_default(fdt, node, "st,mem-size", 0U);

	flush_dcache_range((uintptr_t)&size, sizeof(uint32_t));

	return size;
}

/*******************************************************************************
 * This function gets OPP table node from the DT.
 * Returns node offset on success and a negative FDT error code on failure.
 ******************************************************************************/
static int dt_get_opp_table_node(void)
{
	return fdt_node_offset_by_compatible(fdt, -1, DT_OPP_COMPAT);
}

/*******************************************************************************
 * This function gets OPP parameters (frequency in KHz and voltage in mV) from
 * an OPP table subnode. Platform HW support capabilities are also checked.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
static int dt_get_opp_freqvolt_from_subnode(int subnode, uint32_t *freq_khz,
					    uint32_t *voltage_mv)
{
	const fdt64_t *cuint64;
	const fdt32_t *cuint32;
	uint64_t read_freq_64;
	uint32_t read_voltage_32;

	assert(freq_khz != NULL);
	assert(voltage_mv != NULL);

	cuint32 = fdt_getprop(fdt, subnode, "opp-supported-hw", NULL);
	if (cuint32 != NULL) {
		if (!stm32mp_supports_cpu_opp(fdt32_to_cpu(*cuint32))) {
			VERBOSE("Invalid opp-supported-hw 0x%x\n",
				fdt32_to_cpu(*cuint32));
			return -FDT_ERR_BADVALUE;
		}
	}

	cuint64 = fdt_getprop(fdt, subnode, "opp-hz", NULL);
	if (cuint64 == NULL) {
		VERBOSE("Missing opp-hz\n");
		return -FDT_ERR_NOTFOUND;
	}

	/* Frequency value expressed in KHz must fit on 32 bits */
	read_freq_64 = fdt64_to_cpu(*cuint64) / 1000ULL;
	if (read_freq_64 > (uint64_t)UINT32_MAX) {
		VERBOSE("Invalid opp-hz %llu\n", read_freq_64);
		return -FDT_ERR_BADVALUE;
	}

	cuint32 = fdt_getprop(fdt, subnode, "opp-microvolt", NULL);
	if (cuint32 == NULL) {
		VERBOSE("Missing opp-microvolt\n");
		return -FDT_ERR_NOTFOUND;
	}

	/* Millivolt value must fit on 16 bits */
	read_voltage_32 = fdt32_to_cpu(*cuint32) / 1000U;
	if (read_voltage_32 > (uint32_t)UINT16_MAX) {
		VERBOSE("Invalid opp-microvolt %u\n", read_voltage_32);
		return -FDT_ERR_BADVALUE;
	}

	*freq_khz = (uint32_t)read_freq_64;

	*voltage_mv = read_voltage_32;

	return 0;
}

/*******************************************************************************
 * This function parses OPP table in DT and finds the parameters for the
 * highest frequency supported by the HW platform.
 * If found, the new frequency and voltage values override the original ones.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_max_opp_freqvolt(uint32_t *freq_khz, uint32_t *voltage_mv)
{
	int node;
	int subnode;
	uint32_t freq = 0U;
	uint32_t voltage = 0U;

	assert(freq_khz != NULL);
	assert(voltage_mv != NULL);

	node = dt_get_opp_table_node();
	if (node < 0) {
		return node;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		uint32_t read_freq;
		uint32_t read_voltage;

		if (dt_get_opp_freqvolt_from_subnode(subnode, &read_freq,
						     &read_voltage) != 0) {
			continue;
		}

		if (read_freq > freq) {
			freq = read_freq;
			voltage = read_voltage;
		}
	}

	if ((freq == 0U) || (voltage == 0U)) {
		return -FDT_ERR_NOTFOUND;
	}

	*freq_khz = freq;
	*voltage_mv = voltage;

	return 0;
}

/*******************************************************************************
 * This function parses OPP table in DT and finds all parameters supported by
 * the HW platform.
 * If found, the corresponding frequency and voltage values are respectively
 * stored in @*freq_khz_array and @*voltage_mv_array.
 * Note that @*count has to be set by caller to the effective size allocated
 * for both tables. Its value is then replaced by the number of filled elements.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_all_opp_freqvolt(uint32_t *count, uint32_t *freq_khz_array,
			    uint32_t *voltage_mv_array)
{
	int node;
	int subnode;
	uint32_t idx = 0U;

	assert(count != NULL);
	assert(freq_khz_array != NULL);
	assert(voltage_mv_array != NULL);

	node = dt_get_opp_table_node();
	if (node < 0) {
		return node;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		uint32_t read_freq;
		uint32_t read_voltage;

		if (dt_get_opp_freqvolt_from_subnode(subnode, &read_freq,
						     &read_voltage) != 0) {
			continue;
		}

		if (idx >= *count) {
			return -FDT_ERR_NOSPACE;
		}

		freq_khz_array[idx] = read_freq;
		voltage_mv_array[idx] = read_voltage;
		idx++;
	}

	if (idx == 0U) {
		return -FDT_ERR_NOTFOUND;
	}

	*count = idx;

	return 0;
}

/*******************************************************************************
 * This function gets PWR VDD regulator voltage information from the DT.
 * Returns value in microvolts on success, and 0 on failure.
 ******************************************************************************/
uint32_t dt_get_pwr_vdd_voltage(void)
{
	struct rdev *regul = dt_get_vdd_regulator();
	uint16_t min;

	if (regul == NULL) {
		return 0;
	}

	regulator_get_range(regul, &min, NULL);

	return (uint32_t)min * 1000U;
}

/*******************************************************************************
 * This function retrieves VDD supply regulator from DT.
 * Returns an rdev taken from supply node, NULL otherwise.
 ******************************************************************************/
struct rdev *dt_get_vdd_regulator(void)
{
	int node = fdt_node_offset_by_compatible(fdt, -1, DT_PWR_COMPAT);

	if (node < 0) {
		return NULL;
	}

	return regulator_get_by_supply_name(fdt, node, "vdd");
}

/*******************************************************************************
 * This function retrieves CPU supply regulator from DT.
 * Returns an rdev taken from supply node, NULL otherwise.
 ******************************************************************************/
struct rdev *dt_get_cpu_regulator(void)
{
	int node = fdt_path_offset(fdt, "/cpus/cpu@0");

	if (node < 0) {
		return NULL;
	}

	return regulator_get_by_supply_name(fdt, node, "cpu");
}

/*******************************************************************************
 * This function retrieves USB phy regulator name from DT.
 * Returns string taken from supply node, NULL otherwise.
 ******************************************************************************/
struct rdev *dt_get_usb_phy_regulator(void)
{
	int node = fdt_node_offset_by_compatible(fdt, -1, DT_USBPHYC_COMPAT);
	int subnode;

	if (node < 0) {
		return NULL;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		struct rdev *supply = regulator_get_by_supply_name(fdt, node, "phy");

		if (supply != NULL) {
			return supply;
		}
	}

	return NULL;
}

/*******************************************************************************
 * This function retrieves board model from DT
 * Returns string taken from model node, NULL otherwise
 ******************************************************************************/
const char *dt_get_board_model(void)
{
	int node = fdt_path_offset(fdt, "/");

	if (node < 0) {
		return NULL;
	}

	return (const char *)fdt_getprop(fdt, node, "model", NULL);
}

/*******************************************************************************
 * This function gets the pin count for a GPIO bank based from the FDT.
 * It also checks node consistency.
 ******************************************************************************/
int fdt_get_gpio_bank_pin_count(unsigned int bank)
{
	int pinctrl_node;
	int node;
	uint32_t bank_offset;

	pinctrl_node = stm32_get_gpio_bank_pinctrl_node(fdt, bank);
	if (pinctrl_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	bank_offset = stm32_get_gpio_bank_offset(bank);

	fdt_for_each_subnode(node, fdt, pinctrl_node) {
		const fdt32_t *cuint;

		if (fdt_getprop(fdt, node, "gpio-controller", NULL) == NULL) {
			continue;
		}

		cuint = fdt_getprop(fdt, node, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		if (fdt32_to_cpu(*cuint) != bank_offset) {
			continue;
		}

		if (fdt_get_status(node) == DT_DISABLED) {
			return 0;
		}

		cuint = fdt_getprop(fdt, node, "ngpios", NULL);
		if (cuint == NULL) {
			return -FDT_ERR_NOTFOUND;
		}

		return (int)fdt32_to_cpu(*cuint);
	}

	return 0;
}
