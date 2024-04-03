#
# Copyright (c) 2017-2020, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

SP_MIN_WITH_SECURE_FIQ	:=	1

# Allow SP_min to be placed in DDR
STM32MP_SP_MIN_IN_DDR	?=	0

$(eval $(call assert_booleans, STM32MP_SP_MIN_IN_DDR))

$(eval $(call add_defines, STM32MP_SP_MIN_IN_DDR))

BL32_CFLAGS		+=	-DSTM32MP_SHARED_RESOURCES

BL32_SOURCES		+=	drivers/st/clk/stm32mp1_calib.c			\
				drivers/st/etzpc/etzpc.c			\
				drivers/st/regulator/regulator_fixed.c		\
				drivers/st/rng/stm32_rng.c			\
				drivers/st/rtc/stm32_rtc.c			\
				drivers/st/tamper/stm32_tamp.c			\
				drivers/st/timer/stm32_timer.c 			\
				plat/common/aarch32/platform_mp_stack.S		\
				plat/st/stm32mp1/sp_min/sp_min_setup.c		\
				plat/st/stm32mp1/stm32mp1_low_power.c		\
				plat/st/stm32mp1/stm32mp1_pm.c			\
				plat/st/stm32mp1/stm32mp1_power_config.c	\
				plat/st/stm32mp1/stm32mp1_shared_resources.c	\
				plat/st/stm32mp1/stm32mp1_topology.c

# Generic GIC v2
include drivers/arm/gic/v2/gicv2.mk

BL32_SOURCES		+=	${GICV2_SOURCES}			\
				plat/common/plat_gicv2.c		\
				plat/st/common/stm32_gic.c

# Generic PSCI
BL32_SOURCES		+=	plat/common/plat_psci_common.c

# Generic FDT
BL32_SOURCES		+=	common/fdt_fixup.c

# SCMI server drivers
BL32_SOURCES		+=	drivers/st/scmi-msg/base.c		\
				drivers/st/scmi-msg/clock.c		\
				drivers/st/scmi-msg/entry.c		\
				drivers/st/scmi-msg/reset_domain.c	\
				drivers/st/scmi-msg/smt.c

# stm32mp1 specific services
BL32_SOURCES		+=	plat/st/stm32mp1/services/bsec_svc.c		\
				plat/st/stm32mp1/services/low_power_svc.c	\
				plat/st/stm32mp1/services/pwr_svc.c		\
				plat/st/stm32mp1/services/rcc_svc.c		\
				plat/st/stm32mp1/services/stm32mp1_svc_setup.c	\
				plat/st/stm32mp1/stm32mp1_scmi.c

# Arm Archtecture services
BL32_SOURCES		+=	services/arm_arch_svc/arm_arch_svc_setup.c

ifneq ($(STM32MP_SP_MIN_IN_DDR),1)
BL32_SOURCES		+=	plat/st/stm32mp1/stm32mp1_critic_power.c
endif
