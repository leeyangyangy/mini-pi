/*
 * Copyright (C) 2019-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <arch.h>
#include <asm_macros.S>
#include <common/bl_common.h>
#include <drivers/st/stm32_gpio.h>
#include <smccc_helpers.h>

	.global	stm32_pwr_down_wfi_load
	.global	stm32_pwr_down_wfi_wrapper
	.global	bl2_vector_table
	.global	disable_mmu_secure
	.global	__STACKS_END__

func stm32_pwr_down_wfi_wrapper
	push	{r4,r5,r6,lr}

	# Save r0 and r1 in  r2 and r3, as they are used in disable_mmu_secure
	mov	r2, r0
	mov	r3, r1
	# Save current sp in r4
	mov	r4, sp
	# Save current VBAR in r5
	ldcopr	r5, VBAR
	# Save current MVBAR in r6
	ldcopr	r6, MVBAR

	# Reuse BL2 vector table for VBAR and MVBAR
	ldr	r1, =bl2_vector_table
	stcopr	r1, VBAR
	stcopr	r1, MVBAR

	# Set sp to BL2 STACK (as BL2 is not using it anymore)
	ldr	sp, =__STACKS_END__

	# Disable MMU as TLB are still stored in DDR,
	# and in few instructions DDR won't be readable
	bl	disable_mmu_secure

	# dsb is done in disable mmu
	# isb is done in disable mmu

	mov	r0, r2
	mov	r1, r3
	bl	stm32_pwr_down_wfi_load

	# Restore stack
	mov	sp, r4
	# Restore VBAR
	stcopr	r5, VBAR
	# Restore MVBAR
	stcopr	r6, MVBAR

	# Synchronize on memory access and instruction
	# after resetting stack/IT handler
	dsb
	isb

	pop	{r4,r5,r6,pc}
endfunc stm32_pwr_down_wfi_wrapper
