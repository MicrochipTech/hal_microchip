/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include <mec_ecs_regs.h>
#include <mec_ecia_regs.h>

#include "mec_pcfg.h"
#include "mec_defs.h"
#include "mec_ecia_api.h"
#include "mec_ecs_api.h"
#include "mec_pcr_api.h"
#include "mec_mmcr.h"
#include "mec_retval.h"

/* #define MEC_ECIA_ENABLE_CHECKS */
#ifdef MEC_ECIA_ENABLE_CHECKS
#include <assert.h>
#endif

/*
 * EC Interrupt Aggregator and NVIC configuration.
 * Implementation:
 * All NVIC enables and priorities will be set at initialization based
 * upon the supplied direct bitmap and NVIC priority.
 * Peripheral level code will use the ECIA GIRQ and any peripheral interrupt
 * enable/status registers.
 * Reason:
 * NVIC interrupt registers may only be accessed when the Cortex-M4 is in
 * privileged mode.
 */

static void nvic_extirq_enable(uint32_t extn)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return;
    }

    NVIC_EnableIRQ(extn);
}

static void nvic_extirq_disable(uint32_t extn)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return;
    }

    NVIC_DisableIRQ(extn);
}

static void nvic_extirq_pend_clear(uint32_t extn)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return;
    }

    NVIC_ClearPendingIRQ(extn);
}

static uint32_t nvic_extirq_pend_get(uint32_t extn)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return 0;
    }

    return NVIC_GetPendingIRQ(extn);
}

/* priority = Cortex-M4 NVIC for this SoC. 3-bits where 0 = highest and 7 is
 * lowest priority. Each external interrupt priority is an 8-bit register
 * where the priority is stored in the upper 3 bits.
 */
static void nvic_extirq_priority_set(uint32_t extn, uint8_t priority)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return;
    }

    NVIC_SetPriority(extn, priority);
}

static uint32_t nvic_extirq_priority_get(uint32_t extn)
{
    if (extn >= MEC5_MAX_NVIC_EXT_INPUTS) {
        return 0; /* NVIC POR default priority */
    }

    return NVIC_GetPriority(extn);
}

/* !!! Touches NVIC registers. Caller must be Privileged !!!
 * NOTE: Cortex-M4 NVIC encodes priority where 0 = highest and lowests
 * is equal to all bits set. The number of bits implemented is set at
 * hardware design time (MEC_NVIC_NUM_PRI_BITS).
 */
static void set_all_pri(uint8_t dflt_priority)
{
    uint32_t pri32 = ((uint32_t)(dflt_priority & 0x7U) << (8U - __NVIC_PRIO_BITS)) & 0xffu;
    uintptr_t nvic_ip_addr = (uintptr_t)&NVIC->IP[0];

    pri32 |= (pri32 << 8);
    pri32 |= (pri32 << 16);

    for (uint32_t n = 0; n < MEC5_NVIC_NUM_IP_REGS; n++) {
        MEC_MMCR32(nvic_ip_addr) = pri32;
        nvic_ip_addr += 4U;
    }
}

static void clear_aggr_enables(uint32_t clear_bitmap)
{
    mmcr32_wr(clear_bitmap, MEC_ECIA_BASE + MEC_ECIA_AGGR_CLREN_OFS);
}

static void set_aggr_enables(uint32_t aggr_bitmap)
{
    mmcr32_wr(aggr_bitmap, MEC_ECIA_BASE + MEC_ECIA_AGGR_SETEN_OFS);
}

static void clear_all_girqs(uint32_t ecia_base)
{
    for (uint32_t n = 0; n < MEC_ECIA_GIRQZ_MAX; n++) {
        mmcr32_wr(UINT32_MAX, ecia_base + MEC_ECIA_GIRQ_ENCLR_OFS);
        mmcr32_wr(UINT32_MAX, ecia_base + MEC_ECIA_GIRQ_STS_OFS);
    }
}

static void clear_all_nvic(void)
{
    for (uint32_t n = 0; n < MEC5_NVIC_NUM_REGS; n++) {
        NVIC->ICER[n] = UINT32_MAX;
        NVIC->ICPR[n] = UINT32_MAX;
    }
}

/**
 * mec_ecia_init - Initial EC Interrupt Aggregator and NVIC
 * external interrupt registers.
 *
 * @param direct_bitmap Bit map of GIRQ's caller wants to use direct NVIC
 * connection.
 *
 * @param dflt_priority - Priority 0=highest to 7=lowest to set all
 * external peripheral NVIC priority to.
 *
 * @note - Touches NVIC registers. Caller must be Privileged.
 * Restrictions on GIRQ's:
 * Aggregated only: 8-12 and 24-26
 * Direct or Aggregated: 13-21 and 23
 * No connection to NVIC: GIRQ22 used to wake AHB fabric only.
 * The function parameters will be masked with allowed bitmaps.
 *
 * Sets all NVIC enables for direct and aggregated interrupt
 * sources.
 * TODO - change to set all GIRQ enables for direct. Do not
 * set any NVIC enables. Drivers will handle NVIC enables.
 * How to handle Aggregated only GIRQ's?
 * GIRQ's 8 - 12, and 26 are GPIO's. The GPIO control registers
 * have interrupt enable fields but ROM enable some. Enabling
 * the aggregated GIRQ's may cause an issue.
 * GIRQ's 24 and 25 are for eSPI Controller to Target virtual wires.
 * Each C2T VWire has an interrupt enable field. We should be able
 * to enable these.
 */

/* #define MEC_ECIA_INIT_FLAG_NO_CLEAR */

void mec_hal_ecia_init(uint32_t direct_bitmap, uint8_t dflt_priority, uint32_t flags)
{
    uint32_t aggr_bitmap = 0;

    /* Clear ECIA PCR sleep enable */
    mec_hal_pcr_clr_blk_slp_en(MEC_PCR_ECIA);

    /* Disconnect all direct capable GIRQ sources from the NVIC
     * allowing us to clear direct NVIC pending bits
     */
    mec_hal_ecs_ictrl(0);

    /* disconnect all GIRQ aggregated block outputs from NVIC */
    clear_aggr_enables(UINT32_MAX);

    /* clear all ECIA GIRQ individual enables and status(source) bits */
    clear_all_girqs(MEC_ECIA_BASE);

    /* clear all NVIC enables and pending status */
    clear_all_nvic();

    set_all_pri(dflt_priority);

    /* mask out GIRQ's that cannot do direct */
    direct_bitmap &= MEC5_ECIA_DIRECT_BITMAP;
    aggr_bitmap = MEC5_ECIA_ALL_BITMAP & ~(direct_bitmap);

    /* Route all aggregated GIRQn outputs to NVIC */
    set_aggr_enables(aggr_bitmap);

    /* enable any direct connections? */
    if (direct_bitmap) {
        /* Disconnect aggregated GIRQ output for direct mapped */
        clear_aggr_enables(direct_bitmap);

        /* enable direct mode */
        mec_hal_ecs_ictrl(1);
    }
}

/* Enable/disable multiple GIRQ sources */
int mec_hal_girq_bm_en(uint32_t girq_num, uint32_t bitmap, uint8_t enable)
{
    uint32_t gidx = 0, gbase = 0, ofs = 0;

    if ((girq_num < MEC_ECIA_GIRQ_FIRST) || (girq_num > MEC_ECIA_GIRQ_LAST)) {
        return MEC_RET_ERR_INVAL;
    }

    if (!bitmap) {
        return MEC_RET_OK;
    }

    gidx = girq_num - MEC_ECIA_GIRQ_FIRST;
    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);
    ofs = MEC_ECIA_GIRQ_ENSET_OFS;

    if (enable == 0) {
        ofs = MEC_ECIA_GIRQ_ENCLR_OFS;
    }

    mmcr32_wr(bitmap, gbase + ofs);

    return MEC_RET_OK;
}

/* Clear multiple GIRQ source(status) bits */
int mec_hal_girq_bm_clr_src(uint32_t girq_num, uint32_t bitmap)
{
    uint32_t gidx = 0, gbase = 0;

    if ((girq_num < MEC_ECIA_GIRQ_FIRST) || (girq_num > MEC_ECIA_GIRQ_LAST)) {
        return MEC_RET_ERR_INVAL;
    }

    if (!bitmap) {
        return MEC_RET_OK;
    }

    gidx = girq_num - MEC_ECIA_GIRQ_FIRST;
    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    mmcr32_wr(bitmap, gbase + MEC_ECIA_GIRQ_STS_OFS);
    
    return MEC_RET_OK;
}

uint32_t mec_hal_girq_source_get(uint32_t girq_num)
{
    uint32_t gidx = 0, gbase = 0;

    if ((girq_num < MEC_ECIA_GIRQ_FIRST) || (girq_num > MEC_ECIA_GIRQ_LAST)) {
        return 0u;
    }

    gidx = girq_num - MEC_ECIA_GIRQ_FIRST;
    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    return mmcr32_rd(gbase + MEC_ECIA_GIRQ_STS_OFS);
}

uint32_t mec_hal_girq_result_get(uint32_t girq_num)
{
    uint32_t gidx = 0, gbase = 0;

    if ((girq_num < MEC_ECIA_GIRQ_FIRST) || (girq_num > MEC_ECIA_GIRQ_LAST)) {
        return 0u;
    }

    gidx = girq_num - MEC_ECIA_GIRQ_FIRST;
    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    return mmcr32_rd(gbase + MEC_ECIA_GIRQ_RESULT_OFS);
}

uint32_t mec_hal_girq_result_test(uint32_t girq_num, uint32_t bitpos)
{
    uint32_t result = mec_hal_girq_result_get(girq_num);

    return (result & MEC_BIT(bitpos));
}

/* Set or clear GIRQ enable for a peripheral source. */
void mec_hal_girq_ctrl(uint32_t devi, int enable)
{
    uint32_t gidx = MEC_ECIA_INFO_GIRQZ(devi);
    uint32_t gpos = MEC_ECIA_INFO_GIRQ_POS(devi);
    uint32_t gbase = 0, ofs = 0;

    if (gidx >= MEC5_ECIA_NUM_GIRQS) {
        return;
    }

    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    ofs = MEC_ECIA_GIRQ_ENSET_OFS;
    if (enable == 0) {
        ofs = MEC_ECIA_GIRQ_ENCLR_OFS;
    }

    mmcr32_wr(MEC_BIT(gpos), gbase + ofs);
}

/*
 * Return 0 if source not set else non-zero
 * (single bit should be set in word)
 */
uint32_t mec_hal_girq_src(uint32_t devi)
{
    uint32_t gidx = MEC_ECIA_INFO_GIRQZ(devi);
    uint32_t gpos = MEC_ECIA_INFO_GIRQ_POS(devi);
    uint32_t gbase = 0;

    if (gidx >= MEC5_ECIA_NUM_GIRQS) {
        return 0;
    }

    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    return mmcr32_rd(gbase + MEC_ECIA_GIRQ_STS_OFS) & MEC_BIT(gpos);
}

uint32_t mec_hal_girq_result(uint32_t devi)
{
    uint32_t gidx = MEC_ECIA_INFO_GIRQZ(devi);
    uint32_t gpos = MEC_ECIA_INFO_GIRQ_POS(devi);
    uint32_t gbase = 0;

    if (gidx >= MEC5_ECIA_NUM_GIRQS) {
        return 0;
    }

    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);

    return mmcr32_rd(gbase + MEC_ECIA_GIRQ_RESULT_OFS) & MEC_BIT(gpos);
}

/* Clear GIRQ source(status) for a peripheral source. */
void mec_hal_girq_clr_src(uint32_t devi)
{
    uint32_t gidx = MEC_ECIA_INFO_GIRQZ(devi);
    uint32_t gpos = MEC_ECIA_INFO_GIRQ_POS(devi);
    uint32_t gbase = 0;

    if (gidx >= MEC5_ECIA_NUM_GIRQS) {
        return;
    }

    gbase = MEC_ECIA_GIRQZ_BASE(MEC_ECIA_BASE, gidx);
    
    mmcr32_wr(MEC_BIT(gpos), gbase + MEC_ECIA_GIRQ_STS_OFS);
}

int mec_hal_ecia_girq_aggr_enable(uint32_t girq_num, uint8_t enable)
{
    uint32_t ofs = MEC_ECIA_AGGR_SETEN_OFS;
    uint8_t gid = (uint8_t)(girq_num & 0xffu);

    if ((gid < MEC_ECIA_GIRQ_FIRST) || (girq_num > MEC_ECIA_GIRQ_LAST)) {
        return MEC_RET_ERR_INVAL;
    }

    if (enable == 0) {
        ofs = MEC_ECIA_AGGR_CLREN_OFS;
    }

    mmcr32_wr(MEC_BIT(gid), MEC_ECIA_BASE + ofs);

    return MEC_RET_OK;
}

/*
 * This is not a simple question.
 * This routine only works if Method 1 initialization
 * is implemented in mec_ecia_init() and the application
 * developer calls mec_ecia_init() or duplicates its
 * functionality.
 */
int mec_hal_ecia_is_direct(uint32_t devi)
{
    uint32_t girq_num = 0;

    /* Is direct mode disabled? */
    if (mec_hal_ecs_is_idirect() == 0) {
        return 0;
    }

    /* bitmap and aggregator set/clear enable and active use non-zero numbering */
    girq_num = MEC_ECIA_INFO_GIRQZ(devi) + MEC_ECIA_GIRQ_FIRST;

    if (!(MEC_BIT(girq_num) & MEC5_ECIA_DIRECT_BITMAP)) {
        return 0;
    }

    /* Is the GIRQ's aggregated output routed to NVIC?
     * If yes, then we can't enable direct connections of sources in this GIRQ
     */
    if (mmcr32_test_bit(MEC_ECIA_BASE + MEC_ECIA_AGGR_SETEN_OFS, girq_num)) {
        return 0;
    }

    return 1;
}

/* !!! Touches NVIC register(s). Caller must be Privileged !!! */

void mec_hal_ecia_nvic_enable(uint32_t devi)
{
    uint32_t nvic_extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        nvic_extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    nvic_extirq_enable(nvic_extn);
}

void mec_hal_ecia_nvic_disable(uint32_t devi)
{
    uint32_t nvic_extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        nvic_extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    nvic_extirq_disable(nvic_extn);
}

void mec_hal_ecia_nvic_clr_pend(uint32_t devi)
{
    uint32_t nvic_extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        nvic_extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    nvic_extirq_pend_clear(nvic_extn);
}

uint32_t mec_hal_ecia_nvic_get_pending(uint32_t devi)
{
    uint32_t extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    return nvic_extirq_pend_get(extn);
}

uint8_t mec_hal_ecia_nvic_get_pri(uint32_t devi)
{
    uint32_t extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    return (uint8_t)(nvic_extirq_priority_get(extn));
}

void mec_hal_ecia_nvic_set_pri(uint32_t devi, uint8_t priority)
{
    uint32_t extn = MEC_ECIA_INFO_NVIC_AGGR(devi);

    if (mec_hal_ecia_is_direct(devi)) {
        extn = MEC_ECIA_INFO_NVIC_DIRECT(devi);
    }

    if (priority > MEC5_NVIC_PRI_LO_VAL) {
        priority = MEC5_NVIC_PRI_LO_VAL;
    }

    nvic_extirq_priority_set(extn, priority);
}

/* end mec_ecia.c */
