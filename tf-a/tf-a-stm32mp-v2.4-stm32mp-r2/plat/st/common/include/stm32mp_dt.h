/*
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 * Copyright (c) 2017-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP_DT_H
#define STM32MP_DT_H

#include <stdbool.h>
#include <stdint.h>

#include <libfdt.h>

#define DT_DISABLED		U(0)
#define DT_NON_SECURE		U(1)
#define DT_SECURE		U(2)
#define DT_SHARED		(DT_NON_SECURE | DT_SECURE)

struct dt_node_info {
	uint32_t base;
	int32_t clock;
	int32_t reset;
	uint32_t status;
};

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/
int dt_open_and_check(uintptr_t dt_addr);
int fdt_get_address(void **fdt_addr);
bool fdt_check_node(int node);
uint8_t fdt_get_status(int node);
int fdt_get_interrupt(int node, const fdt32_t **array, int *len,
		      bool *extended);
int dt_set_stdout_pinctrl(void);
void dt_fill_device_info(struct dt_node_info *info, int node);
int dt_get_node(struct dt_node_info *info, int offset, const char *compat);
int dt_get_stdout_uart_info(struct dt_node_info *info);
int dt_match_instance_by_compatible(const char *compatible, uintptr_t address);
uint32_t dt_get_ddr_size(void);
int dt_get_max_opp_freqvolt(uint32_t *freq_khz, uint32_t *voltage_mv);
int dt_get_all_opp_freqvolt(uint32_t *count, uint32_t *freq_khz_array,
			    uint32_t *voltage_mv_array);
uint32_t dt_get_pwr_vdd_voltage(void);
struct rdev *dt_get_vdd_regulator(void);
struct rdev *dt_get_cpu_regulator(void);
struct rdev *dt_get_usb_phy_regulator(void);
const char *dt_get_board_model(void);
int fdt_get_gpio_bank_pin_count(unsigned int bank);

#endif /* STM32MP_DT_H */
