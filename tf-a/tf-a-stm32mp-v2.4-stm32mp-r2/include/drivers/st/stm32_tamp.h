/*
 * Copyright (c) 2014-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32_TAMP_H
#define STM32_TAMP_H

/* Internal Tamper */
enum stm32_tamp_int_id {
	INT_TAMP1 = 0,
	INT_TAMP2,
	INT_TAMP3,
	INT_TAMP4,
	INT_TAMP5,
	INT_TAMP6,
	INT_TAMP7,
	INT_TAMP8,
	INT_TAMP9,
	INT_TAMP10,
	INT_TAMP11,
	INT_TAMP12,
	INT_TAMP13,
	INT_TAMP14,
	INT_TAMP15,
	INT_TAMP16
};

/* External Tamper */
enum stm32_tamp_ext_id {
	EXT_TAMP1 = 0,
	EXT_TAMP2,
	EXT_TAMP3,
	EXT_TAMP4,
	EXT_TAMP5,
	EXT_TAMP6,
	EXT_TAMP7,
	EXT_TAMP8
};

/* Out pin to compare for external Tamper */
enum stm32_tamp_ext_out_id {
	TAMPOUTSEL_SAME_AS_INPUT = 0,
	TAMPOUTSEL1 = 1,
	TAMPOUTSEL2,
	TAMPOUTSEL3,
	TAMPOUTSEL4,
	TAMPOUTSEL5,
	TAMPOUTSEL6,
	TAMPOUTSEL7,
	TAMPOUTSEL8,
};

/* Define number of backup registers in zone 1 and zone 2 (remaining are in
 * zone 3)
 *
 * backup registers in zone 1 : read/write only in secure mode
 *                     zone 2 : write only in secure mode, read in secure
 *                              and non-secure mode
 *                     zone 3 : read/write in secure and non-secure mode
 *
 * Protection zone 1 if nb_zone1_regs == 0 no backup register are in zone 1
 *                   else backup registers from TAMP_BKP0R to TAMP_BKPxR
 *                   with x = nb_zone1_regs - 1 are in zone 1.
 * Protection zone 2 if nb_zone2_regs == 0 no backup register are in zone 2
 *                   else backup registers from TAMP_BKPyR with y = nb_zone1_regs
 *                   to TAMP_BKPzR with z = (nb_zone1_regs1 + nb_zone2_regs - 1)
 *                   are in zone 2.
 * Protection zone 3 backup registers from TAMP_BKPtR
 *                   with t = nb_zone1_regs1 + nb_zone2_regs to last backup
 *                   register are in zone 3.
 */
struct bkpregs_conf {
	uint32_t nb_zone1_regs;
	uint32_t nb_zone2_regs;
};

/* Define TAMPER modes */
#define TAMP_DISABLE		0x0U
#define TAMP_ENABLE		0x1U
#define TAMP_TRIG_OFF		0x0U
#define TAMP_TRIG_ON		0x2U
#define TAMP_ACTIVE		0x4U
#define TAMP_ERASE		0x0U
#define TAMP_NOERASE		0x8U
#define TAMP_NO_EVT_MASK	0x0U
#define TAMP_EVT_MASK		0x10U

/* Define Passive FILTER mode */
#define TAMP_FILTER_TAMPPUDIS_OFFSET	7U
#define TAMP_FILTER_PRECHARGE		(0x0U << TAMP_FILTER_TAMPPUDIS_OFFSET)
#define TAMP_FILTER_PULL_UP_DISABLE	(0x1U << TAMP_FILTER_TAMPPUDIS_OFFSET)
#define TAMP_FILTER_TAMPPRCH_OFFSET	5U
#define TAMP_FILTER_DURATION_1_CYCLE	(0x0U << TAMP_FILTER_TAMPPRCH_OFFSET)
#define TAMP_FILTER_DURATION_2_CYCLES	(0x1U << TAMP_FILTER_TAMPPRCH_OFFSET)
#define TAMP_FILTER_DURATION_4_CYCLES	(0x2U << TAMP_FILTER_TAMPPRCH_OFFSET)
#define TAMP_FILTER_DURATION_8_CYCLES	(0x3U << TAMP_FILTER_TAMPPRCH_OFFSET)
#define TAMP_FILTER_TAMPFLT_OFFSET	3U
#define TAMP_FILTER_COUNT_1		(0x0U << TAMP_FILTER_TAMPFLT_OFFSET)
#define TAMP_FILTER_COUNT_2		(0x1U << TAMP_FILTER_TAMPFLT_OFFSET)
#define TAMP_FILTER_COUNT_4		(0x2U << TAMP_FILTER_TAMPFLT_OFFSET)
#define TAMP_FILTER_COUNT_8		(0x3U << TAMP_FILTER_TAMPFLT_OFFSET)
#define TAMP_FILTER_TAMPFREQ_OFFSET	0U
#define TAMP_FILTER_SAMPLING_32768	(0x0U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_16384	(0x1U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_8192	(0x2U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_4096	(0x3U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_2048	(0x4U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_1024	(0x5U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_512	(0x6U << TAMP_FILTER_TAMPFREQ_OFFSET)
#define TAMP_FILTER_SAMPLING_256	(0x7U << TAMP_FILTER_TAMPFREQ_OFFSET)

/*  Define active filter */
#define TAMP_ACTIVE_FLTEN_OFFSET	31U
#define TAMP_ACTIVE_FILTER_OFF		(0x0U << TAMP_ACTIVE_FLTEN_OFFSET)
#define TAMP_ACTIVE_FILTER_ON		(0x1U << TAMP_ACTIVE_FLTEN_OFFSET)
#define TAMP_FILTER_ATOSHARE_OFFSET	30U
#define TAMP_FILTER_USE_DEDICATED_OUT	(0x0U << TAMP_FILTER_ATOSHARE_OFFSET)
#define TAMP_FILTER_SELECT_OUT		(0x1U << TAMP_FILTER_ATOSHARE_OFFSET)
#define TAMP_ACTIVE_ATPER_OFFSET	24U
#define TAMP_ACTIVE_ATPER_1_OUTPUT	(0x0U << TAMP_ACTIVE_ATPER_OFFSET)
#define TAMP_ACTIVE_ATPER_2_OUTPUTS	(0x1U << TAMP_ACTIVE_ATPER_OFFSET)
#define TAMP_ACTIVE_ATPER_3_4_OUTPUTS	(0x2U << TAMP_ACTIVE_ATPER_OFFSET)
#define TAMP_ACTIVE_ATPER_5_OUTPUTS	(0x3U << TAMP_ACTIVE_ATPER_OFFSET)
#define TAMP_ACTIVE_ATCKSEL_OFFSET	16U
#define TAMP_ACTIVE_CKSEL_DIV_0		(0x0U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_2		(0x1U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_4		(0x2U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_8		(0x3U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_16	(0x4U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_32	(0x5U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_64	(0x6U << TAMP_ACTIVE_ATCKSEL_OFFSET)
#define TAMP_ACTIVE_CKSEL_DIV_128	(0x7U << TAMP_ACTIVE_ATCKSEL_OFFSET)

/* Define secure mode acces */
/* Tamper configuration and interrupt can be written when the APB access is secure or nonsecure.*/
#define TAMP_REGS_IT_UNSECURE		0U
/* Tamper configuration and interrupt can be written only when the APB access is secure.*/
#define TAMP_REGS_IT_SECURE		BIT(31)

/*
 * stm32_tamp_write_mcounter : Increase monotonic counter[counter_idx]
 */
int stm32_tamp_write_mcounter(int counter_idx);
uint32_t stm32_tamp_read_mcounter(int counter_idx);

/*
 * stm32_tamp_it_handler : Interrupt handler
 */
void stm32_tamp_it_handler(void);

/*
 * stm32_tamp_configure_secure_access: Configure which registers can be
 * read/write from unsecure world
 * secure_conf is a bit field from TAMP_.*_{UN,}SECURE define
 */
void stm32_tamp_configure_secure_access(uint32_t secure_conf);

/*
 * stm32_tamp_configure_privilege_access: Configure which registers can be
 * read/write from unpriviliged world
 * privilege_conf is a bit field from TAMP_.*_{UN,}PRIVILEGE define
 */
void stm32_tamp_configure_privilege_access(uint32_t privilege_conf);

/*
 * stm32_tamp_configure_passive: Configure passive mode
 * passive_conf is a bit field from TAMP_FILTER_* define
 */
void stm32_tamp_configure_passive(uint32_t passive_conf);

/*
 * stm32_tamp_configure_ctive: Configure active mode
 * passive_conf is a bit field from TAMP_ACTIVE_* define
 */
void stm32_tamp_configure_active(uint32_t active_conf);

/*
 * stm32_tamp_configure_internal: Configure one internal tamper
 * id: internal tamper id
 * mode: bitmask from TAMPER modes define
 * callback: function to call when tamper is raised (can be NULL),
 *           called in interrupt context,
 *           if callback returns negative value, blocked secrets stay blocked
 *           (driver doesn't release this specific tamper).
 *           if callback returns 0 this specific tamp is ack (in case of no-erase
 *           tamper, blocked secret are unblocked)
 *           if callback returns positive value, this specific tamp is ack (in
 *           case of no-erase tamper, blocked secret are unblocked) and system is
 *           rebooted).
 *
 * return: -EINVAL if 'id' is not a valid internal tamp id, else 0
 */
int stm32_tamp_configure_internal(enum stm32_tamp_int_id id, uint32_t mode,
				  int (*callback)(int id));

/*
 * stm32_tamp_configure_external: Configure one external tamper
 * id: external tamper id
 * mode: bitmask from TAMPER modes define
 * pin_out; output pin connected to input pin (linekd with selected ext tamp id)
 * callback: function to call when tamper is raised (can be NULL),
 *           called in interrupt context,
 *           if callback returns negative value, blocked secrets stay blocked
 *           (driver doesn't release this specific tamper).
 *           if callback returns 0 this specific tamp is ack (in case of no-erase
 *           tamper, blocked secret are unblocked)
 *           if callback returns positive value, this specific tamp is ack (in
 *           case of no-erase tamper, blocked secret are unblocked) and system is
 *           rebooted).
 *
 * return: -EINVAL if 'id' is not a valid external tamp id, else 0
 */
int stm32_tamp_configure_external(enum stm32_tamp_ext_id id, uint32_t mode,
				  enum stm32_tamp_ext_out_id out_pin, int (*callback)(int id));

/*
 * stm32_tamp_init: Initialize tamper from DT
 * return 0 if disabled, 1 if enabled, else < 0
 */
int stm32_tamp_init(void);

/*
 * stm32_tamp_set_secure_bkprwregs : Configure backup registers zone.
 * registers in zone 1 : read/write only in secure mode
 *              zone 2 : write only in secure mode, read in secure and non-secure mode
 *              zone 3 : read/write in secure and non-secure mode
 *
 * bkpregs_conf : a pointer to struct bkpregs_conf that define the number of registers in zone 1
 * and zone 2 (remaining backup registers will be in zone 3).
 *
 * return 0 if OK, -ENODEV if zone 1 and/or zone 2 definition are out of range.
 */
int stm32_tamp_set_secure_bkpregs(struct bkpregs_conf *bkpregs_conf);

/*
 * stm32_tamp_set_config: apply configuration
 * default one if no previous call to any of :
 * stm32_tamp_configure_passive()
 * stm32_tamp_configure_active()
 * stm32_tamp_configure_internal()
 * stm32_tamp_configure_external()
 * stm32_tamp_configure_secret_list()
 * stm32_tamp_configure_secure_access()
 * stm32_tamp_configure_privilige_access()
 *
 * return: < 0 if unable to apply configuration, else 0
 */
int stm32_tamp_set_config(void);

#endif /* STM32_TAMP_H */
