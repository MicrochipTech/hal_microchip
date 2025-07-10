/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ECIA_API_H
#define _MEC_ECIA_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mec_defs.h"

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Globally disables maskable interrupts in CPU.
 * Configure MEC5 ECIA.
 * Disables and clears all external NVIC enables.
 * Sets all external NVIC priorities to dflt_priority.
 * Leaves interrupts globally masked off.
 * No flags defined at this time
 */
#define MEC_ECIA_INIT_FLAG_NO_CLEAR 0x01 /* do not clear GIRQ status or NVIC pending */

void mec_hal_ecia_init(uint32_t direct_bitmap, uint8_t dflt_priority, uint32_t flags);

/*
 * The following routines do not globally mask interrupts and
 * restore interrupts in the CPU.
 * It is the applications responsiblity to use these routines safely.
 * If a device's GIRQ was configured for Aggregated mode then changes
 * to priority, etc. will affect all peripheral sources connected to
 * the aggregated GIRQ.
 */
void mec_hal_girq_ctrl(uint32_t devi, int enable);

uint32_t mec_hal_girq_src(uint32_t devi);
uint32_t mec_hal_girq_result(uint32_t devi);

void mec_hal_girq_clr_src(uint32_t devi);

int mec_hal_girq_bm_en(uint32_t girq_num, uint32_t bitmap, uint8_t enable);

int mec_hal_girq_bm_clr_src(uint32_t girq, uint32_t clr_src_bitmap);

uint32_t mec_hal_girq_source_get(uint32_t girq_num);
uint32_t mec_hal_girq_result_get(uint32_t girq);

uint32_t mec_hal_girq_result_test(uint32_t girq, uint32_t bitpos);

int mec_hal_ecia_girq_aggr_enable(uint32_t girq_num, uint8_t enable);

/* Check if a peripheral interrupt source is direct capable.
 * This is not a simple question.
 * This routine only works if Method 1 initialization
 * is implement in mec_ecia_init() and the application
 * developer calls mec_ecia_init() or duplicates its
 * functionality.
 */
int mec_hal_ecia_is_direct(uint32_t devi);

/* NVIC external interrupt API
 * !!!! WARNING: Touching NVIC registers requires CPU in privileged mode !!!!
 */
void mec_hal_ecia_nvic_enable(uint32_t devi);
void mec_hal_ecia_nvic_disable(uint32_t devi);
/* Clear NVIC interrupt pending for a specific device source */
void mec_hal_ecia_nvic_clr_pend(uint32_t devi);
/* Get NVIC interrupt pending status for a specific device source. */
uint32_t mec_hal_ecia_nvic_get_pending(uint32_t devi);
/* Return peripheral source NVIC priority 0=highest,...,7=lowest */
uint8_t mec_hal_ecia_nvic_get_pri(uint32_t devi);
/* Set peripheral's NVIC priority 0=highest,...,7=lowest */
void mec_hal_ecia_nvic_set_pri(uint32_t devi, uint8_t priority);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_ECIA_H */
