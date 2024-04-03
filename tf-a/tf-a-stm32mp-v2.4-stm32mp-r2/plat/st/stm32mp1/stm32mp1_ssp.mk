#
# Copyright (c) 2015-2021, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

ST_VERSION 		:=	r1.0-ssp
VERSION_STRING		:=	v${VERSION_MAJOR}.${VERSION_MINOR}-${ST_VERSION}(${BUILD_TYPE}):${BUILD_STRING}

# Required to use BL2_IN_XIP_MEM
BL2_IN_XIP_MEM 		:= 	1

SEPARATE_CODE_AND_RODATA :=	1

TRUSTED_BOARD_BOOT	:=	0

# Macros and rules to build TF-A binary
STM32_TF_STM32		:=	$(addprefix ${BUILD_PLAT}/tf-a-ssp-, $(patsubst %.dtb,%.stm32,$(DTB_FILE_NAME)))

PLAT_BL_COMMON_SOURCES	:=	common/fdt_wrappers.c					\
				plat/st/common/stm32mp_common.c				\
				plat/st/stm32mp1/stm32mp1_private.c

PLAT_BL_COMMON_SOURCES	+=	${XLAT_TABLES_LIB_SRCS}

PLAT_BL_COMMON_SOURCES	+=	lib/cpus/aarch32/cortex_a7.S

PLAT_BL_COMMON_SOURCES	+=	drivers/st/uart/aarch32/stm32_console.S

PLAT_BL_COMMON_SOURCES	+=	drivers/arm/tzc/tzc400.c				\
				drivers/clk/clk.c					\
				drivers/delay_timer/delay_timer.c			\
				drivers/delay_timer/generic_delay_timer.c		\
				drivers/st/bsec/bsec2.c					\
				drivers/st/clk/stm32mp_clkfunc.c			\
				drivers/st/gpio/stm32_gpio.c				\
				drivers/st/i2c/stm32_i2c.c				\
				drivers/st/iwdg/stm32_iwdg.c				\
				drivers/st/pmic/stm32mp_pmic.c				\
				drivers/st/pmic/stpmic1.c				\
				drivers/st/regulator/stm32mp_dummy_regulator.c		\
				drivers/st/regulator/stm32mp_regulator.c		\
				drivers/st/reset/stm32mp1_reset.c			\
				plat/st/common/stm32mp_dt.c				\
				plat/st/common/stm32mp_shres_helpers.c			\
				plat/st/stm32mp1/stm32mp1_dbgmcu.c			\
				plat/st/stm32mp1/stm32mp1_helper.S			\
				plat/st/stm32mp1/stm32mp1_syscfg.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/clk/stm32mp1_clk.c

BL2_SOURCES		:=	drivers/io/io_storage.c					\
				drivers/st/crypto/stm32_hash.c				\
				plat/st/stm32mp1/stm32mp1_ssp.c

ifeq (${STM32MP_UART_PROGRAMMER},1)
BL2_SOURCES		+=	drivers/st/uart/stm32_uart.c				\
				plat/st/common/stm32cubeprogrammer_uart.c
endif

ifeq (${STM32MP_USB_PROGRAMMER},1)
BL2_SOURCES		+=	drivers/st/usb_dwc2/usb_dwc2.c				\
				lib/usb/usb_core.c					\
				lib/usb/usb_st_dfu.c					\
				plat/st/common/stm32cubeprogrammer_usb.c		\
				plat/st/stm32mp1/stm32mp1_usb.c
endif

BL2_DTSI		:=	stm32mp15-ssp-bl2.dtsi

check_boot_ssp:
	@if ([ ${STM32MP_UART_PROGRAMMER} = 1 ] && [ ${STM32MP_USB_PROGRAMMER} = 1 ]) || \
	([ ${STM32MP_UART_PROGRAMMER} = 0 ] && [ ${STM32MP_USB_PROGRAMMER} = 0 ]); then \
		echo "Error selecting serial boot device"; \
		false; \
	fi

bl2: check_boot_ssp

${BUILD_PLAT}/stm32mp1-ssp-%.o: ${BUILD_PLAT}/fdts/%-bl2.dtb plat/st/stm32mp1/stm32mp1.S bl2
	@echo "  SSP AS      stm32mp1.S"
	${Q}${AS} ${ASFLAGS} ${TF_CFLAGS} \
		-DDTB_BIN_PATH=\"$<\" \
		-c plat/st/stm32mp1/stm32mp1.S -o $@
