/*
 * Copyright (c) 2016-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_SMC_H
#define STM32MP1_SMC_H

/*
 * SMC function IDs for STM32 Service queries
 * STM32 SMC services use the space between 0x82000000 and 0x8200FFFF
 * like this is defined in SMC calling Convention by ARM
 * for SiP (silicon Partner)
 * https://developer.arm.com/docs/den0028/latest
 */

/* Secure Service access from Non-secure */

/*
 * SMC function STM32_SMC_RCC.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_REG_xxx).
 * Argument a2: (input) Register offset or physical address.
 *		(output) Register read value, if applicable.
 * Argument a3: (input) Register target value if applicable.
 */
#define STM32_SMC_RCC			0x82001000

/*
 * SMC function STM32_SMC_PWR.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_REG_xxx).
 * Argument a2: (input) Register offset or physical address.
 *		(output) Register read value, if applicable.
 * Argument a3: (input) Register target value if applicable.
 */
#define STM32_SMC_PWR			0x82001001

/*
 * SMC functions STM32_SMC_RCC_CAL.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Clock ID (from DT clock bindings).
 */
#define STM32_SMC_RCC_CAL		0x82001002

/*
 * STM32_SMC_BSEC call API
 *
 * Argument a0: (input) SMCC ID
 *		(output) status return code
 * Argument a1: (input) Service ID (STM32_SMC_BSEC_xxx)
 * Argument a2: (input) OTP index
 *		(output) OTP read value, if applicable
 * Argument a3: (input) OTP value if applicable
 */
#define STM32_SMC_BSEC			0x82001003

/* Low Power services */

/*
 * SIP function STM32_SMC_PD_DOMAIN.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a2: (index) ID of target power domain to be enabled/disabled.
 * Argument a3: (input) 0 to disable, 1 to enable target domain.
 */
#define STM32_SMC_PD_DOMAIN		0x82001008

/*
 * SIP function STM32_SMC_RCC_OPP.
 *
 * Argument a0: (input) SMCC ID.
 *		(output) Status return code.
 * Argument a1: (input) Service ID (STM32_SMC_RCC_OPP_xxx).
 *		(output) Rounded frequency, if applicable.
 * Argument a2: (input) Requested frequency.
 */
#define STM32_SMC_RCC_OPP		0x82001009

/*
 * SIP function STM32_SMC_AUTO_STOP - CPU auto stop for OS driver suspend
 *
 * Argument a0: (input) This SMCC ID: STM32_SMC_AUTO_STOP
 *		(output) Status return code.
 */
#define STM32_SMC_AUTO_STOP		0x8200100a

/*
 * STM32_SIP_SMC_SCMI_AGENT0
 * STM32_SIP_SMC_SCMI_AGENT1
 * Process SCMI message pending in SCMI shared memory buffer.
 *
 * Argument a0: (input) SMCC ID
 */
#define STM32_SIP_SMC_SCMI_AGENT0	0x82002000
#define STM32_SIP_SMC_SCMI_AGENT1	0x82002001

/* SMC function IDs for SiP Service queries */
#define STM32_SIP_SVC_CALL_COUNT	0x8200ff00
#define STM32_SIP_SVC_UID		0x8200ff01
/*					0x8200ff02 is reserved */
#define STM32_SIP_SVC_VERSION		0x8200ff03

/* STM32 SiP Service Calls version numbers */
#define STM32_SIP_SVC_VERSION_MAJOR	0x0
#define STM32_SIP_SVC_VERSION_MINOR	0x1

/* Number of STM32 SiP Calls implemented */
#define STM32_COMMON_SIP_NUM_CALLS	9

/* Service ID for STM32_SMC_RCC/_PWR */
#define STM32_SMC_REG_READ		0x0
#define STM32_SMC_REG_WRITE		0x1
#define STM32_SMC_REG_SET		0x2
#define STM32_SMC_REG_CLEAR		0x3

/* Service for BSEC */
#define STM32_SMC_READ_SHADOW		0x01
#define STM32_SMC_PROG_OTP		0x02
#define STM32_SMC_WRITE_SHADOW		0x03
#define STM32_SMC_READ_OTP		0x04
#define STM32_SMC_READ_ALL		0x05
#define STM32_SMC_WRITE_ALL		0x06
#define STM32_SMC_WRLOCK_OTP		0x07

/* SMC error codes */
#define STM32_SMC_OK			0x00000000U
#define STM32_SMC_NOT_SUPPORTED		0xFFFFFFFFU
#define STM32_SMC_FAILED		0xFFFFFFFEU
#define STM32_SMC_INVALID_PARAMS	0xFFFFFFFDU

/* Service ID for STM32_SMC_RCC_OPP */
#define STM32_SMC_RCC_OPP_SET		0x0
#define STM32_SMC_RCC_OPP_ROUND		0x1

#endif /* STM32MP1_SMC_H */
