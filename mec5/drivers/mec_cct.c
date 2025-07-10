/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include "mec_pcfg.h"
#include "mec_defs.h"
#include "mec_cct_regs.h"
#include "mec_ecia_api.h"
#include "mec_pcr_api.h"
#include "mec_cct_api.h"
#include "mec_retval.h"

#define MEC_CCT_CR_SET_CLR_MSK \
    (MEC_BIT(MEC_CCT_CR_CMP1_SET_POS) | MEC_BIT(MEC_CCT_CR_CMP0_SET_POS) |\
     MEC_BIT(MEC_CCT_CR_CMP1_CLR_POS) | MEC_BIT(MEC_CCT_CR_CMP0_CLR_POS))

#define MEC_CCT_CCR0_EDGE_DIS_ALL \
        (MEC_CCT_CCR0_EDG0_SET(MEC_CCT_CAP_EDGE_DIS_VAL) |\
         MEC_CCT_CCR0_EDG1_SET(MEC_CCT_CAP_EDGE_DIS_VAL) |\
         MEC_CCT_CCR0_EDG2_SET(MEC_CCT_CAP_EDGE_DIS_VAL) |\
         MEC_CCT_CCR0_EDG3_SET(MEC_CCT_CAP_EDGE_DIS_VAL))

#define MEC_CCT_CCR1_EDGE_DIS_ALL \
        (MEC_CCT_CCR1_EDG4_SET(MEC_CCT_CAP_EDGE_DIS_VAL) |\
         MEC_CCT_CCR1_EDG5_SET(MEC_CCT_CAP_EDGE_DIS_VAL))

#define MEC_CCT_GIRQ          18
#define MEC_CCT_FRC_GIRQ_POS  20
#define MEC_CCT_CAP0_GIRQ_POS 21
#define MEC_CCT_CAP1_GIRQ_POS 22
#define MEC_CCT_CAP2_GIRQ_POS 23
#define MEC_CCT_CAP3_GIRQ_POS 24
#define MEC_CCT_CAP4_GIRQ_POS 25
#define MEC_CCT_CAP5_GIRQ_POS 26
#define MEC_CCT_CMP0_GIRQ_POS 27
#define MEC_CCT_CMP1_GIRQ_POS 28

#define MEC_CCT_GIRQ_NVIC   10
#define MEC_CCT_FRC_NVIC    146
#define MEC_CCT_CAP0_NVIC   147
#define MEC_CCT_CAP1_NVIC   148
#define MEC_CCT_CAP2_NVIC   149
#define MEC_CCT_CAP3_NVIC   150
#define MEC_CCT_CAP4_NVIC   151
#define MEC_CCT_CAP5_NVIC   152
#define MEC_CCT_CMP0_NVIC   153
#define MEC_CCT_CMP1_NVIC   154

#define MEC_CCT_FRC_ECIA_INFO                                                                      \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_FRC_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_FRC_NVIC)

#define MEC_CCT_CAP0_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP0_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP0_NVIC)

#define MEC_CCT_CAP1_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP1_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP1_NVIC)

#define MEC_CCT_CAP2_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP2_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP2_NVIC)

#define MEC_CCT_CAP3_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP3_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP3_NVIC)

#define MEC_CCT_CAP4_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP4_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP4_NVIC)

#define MEC_CCT_CAP5_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP5_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP5_NVIC)

#define MEC_CCT_CMP0_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CMP0_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CMP0_NVIC)

#define MEC_CCT_CMP1_ECIA_INFO                                                                     \
    MEC_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CMP1_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CMP1_NVIC)

static const uint32_t cct_ecia_tbl[] = {
    MEC_CCT_FRC_ECIA_INFO, MEC_CCT_CAP0_ECIA_INFO, MEC_CCT_CAP1_ECIA_INFO,
    MEC_CCT_CAP2_ECIA_INFO, MEC_CCT_CAP3_ECIA_INFO, MEC_CCT_CAP4_ECIA_INFO,
    MEC_CCT_CAP5_ECIA_INFO, MEC_CCT_CMP0_ECIA_INFO, MEC_CCT_CMP1_ECIA_INFO,
};

/* CCT control register contains Compare 0/1 set and clear bits.
 * These bits have a side-effect. Read returns the current HW status.
 * Writing 1 to any of these bits will clear or set the comparator status.
 * Read-modify-write of the control register could cause an uninteded side-effect.
 * We must mask the comparator set/clear bits when performing any read-modify-write
 * of the control register.
 */
static inline void cct_cr_rmw(uintptr_t cct_base, uint32_t val, uint32_t mask)
{
    uint32_t r = mmcr32_rd(cct_base + MEC_CCT_CR_OFS);

    r = (r & (uint32_t)~(mask | MEC_CCT_CR_SET_CLR_MSK)) | (val & mask);
    mmcr32_wr(r, cct_base + MEC_CCT_CR_OFS);
}

union mec_cct_cap_config_u {
    struct mec_cct_cap_config cap_fields;
    uint32_t cap_cfg_all;
};

int mec_hal_cct_cap_cfg(uintptr_t cct_base, uint8_t capid, struct mec_cct_cap_config *capcfg)
{
    uint32_t ccr_msk = 0, ccr_ofs = 0, mux_msk = 0, v = 0;
    uint8_t ccr_pos = 0, mux_pos = 0;

    if ((cct_base == 0) || (capcfg == NULL) || (capid >= MEC_CCT_CAP_ID_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    union mec_cct_cap_config_u *p = (union mec_cct_cap_config_u *)capcfg;

    if (capid < MEC_CCT_CAP4_ID) {
        ccr_ofs = MEC_CCT_CCR0_OFS;
        ccr_pos = capid;
    } else {
        ccr_ofs = MEC_CCT_CCR1_OFS;
        ccr_pos = (capid - MEC_CCT_CAP4_ID);
    }

    ccr_pos *= 8u;
    ccr_msk = MEC_CCT_CAP_EDGE_MSK0 << ccr_pos;

    /* disable */
    mmcr32_update_field(cct_base + ccr_ofs, (MEC_CCT_CAP_EDGE_DIS_VAL << ccr_pos), ccr_msk);

    /* clear status */
    mec_hal_cct_clear_irq(cct_base, MEC_BIT(MEC_CCT_CAP0_GIRQ_POS + capid));

    /* MUX */
    if (capcfg->prog_mux != 0) {
        mux_pos = capid * 4u;
        mux_msk = 0xfu << mux_pos;
        v = capcfg->mux_sel;
        v <<= mux_pos;
        mmcr32_update_field(cct_base + MEC_CCT_MUX_OFS, v, mux_msk);
    }

    v = p->cap_cfg_all & 0xffu;
    if (capcfg->enable == 0) {
        v &= (uint32_t)~MEC_CCT_CAP_EDGE_MSK0;
        v |= MEC_CCT_CAP_EDGE_DIS_VAL;
    }

    v <<= ccr_pos;
    mmcr32_update_field(cct_base + ccr_ofs, v, ccr_msk);

    return MEC_RET_OK;
}

void mec_hal_cct_clear_irq(uintptr_t cct_base, uint32_t imsk)
{
    uint32_t girq_bm = (imsk << 20);

    if (cct_base == 0) {
        return;
    }

    mec_hal_girq_bm_clr_src(MEC_CCT_GIRQ, girq_bm);
}

void mec_hal_cct_cap_ctrl(uintptr_t cct_base, uint8_t capid, uint8_t edge)
{
    uint32_t msk = 0, ofs = 0, r = 0;
    uint8_t pos = 0;

    if ((cct_base == 0u) || (capid >= MEC_CCT_CAP_ID_MAX)) {
        return;
    }

    if (edge < MEC_CCT_CAP_IDET_DIS) {
        r = edge;
    } else {
        r = MEC_CCT_CAP_EDGE_DIS_VAL;
    }

    if (capid < MEC_CCT_CAP4_ID) {
        ofs = MEC_CCT_CCR0_OFS;
        pos = capid;
    } else {
        ofs = MEC_CCT_CCR1_OFS;
        pos = capid - MEC_CCT_CAP4_ID;
    }

    pos *= 8u;
    r <<= pos;
    msk = MEC_CCT_CAP_EDGE_MSK0 << pos;
    mmcr32_update_field(cct_base + ofs, r, msk);
}

uint8_t mec_hal_cct_fr_clkdiv_get(uintptr_t cct_base)
{
    uint32_t r = 0;

    if (cct_base == 0u) {
        return 0xffu;
    }

    r = mmcr32_rd(cct_base + MEC_CCT_CR_OFS);

    return MEC_CCT_CR_TCLK_GET(r);
}

void mec_hal_cct_enable_irq(uintptr_t cct_base, uint32_t ienbm)
{
    uint32_t girqbm = (ienbm & 0x1ffu) << MEC_CCT_FRC_GIRQ_POS;
    (void)cct_base;

    mec_hal_girq_bm_en(MEC_CCT_GIRQ, girqbm, 1u);
}

void mec_hal_cct_disable_irq(uintptr_t cct_base, uint32_t ienbm)
{
    uint32_t girqbm = (ienbm & 0x1ffu) << MEC_CCT_FRC_GIRQ_POS;
    (void)cct_base;

    mec_hal_girq_bm_en(MEC_CCT_GIRQ, girqbm, 0);
}

void mec_hal_cct_fr_enable(uintptr_t cct_base)
{
    cct_cr_rmw(cct_base, MEC_BIT(MEC_CCT_CR_FR_EN_POS), MEC_BIT(MEC_CCT_CR_FR_EN_POS));
}

void mec_hal_cct_fr_disable(uintptr_t cct_base)
{
    cct_cr_rmw(cct_base, 0, MEC_BIT(MEC_CCT_CR_FR_EN_POS));
}

void mec_hal_cct_fr_reset(uintptr_t cct_base)
{
    cct_cr_rmw(cct_base, MEC_BIT(MEC_CCT_CR_FR_RST_POS), MEC_BIT(MEC_CCT_CR_FR_RST_POS));
}

int mec_hal_cct_init(uintptr_t cct_base, struct mec_cct_config *cfg)
{
    uint32_t i = 0, ofs = 0, msk = 0, r = 0;

    if ((cct_base == 0) || (cfg == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_pcr_clr_blk_slp_en(MEC_PCR_CCT0);
    mec_hal_pcr_blk_reset(MEC_PCR_CCT0);

    /* disable all edge detection */
    mmcr32_wr(MEC_CCT_CCR0_EDGE_DIS_ALL, cct_base + MEC_CCT_CCR0_OFS);
    mmcr32_wr(MEC_CCT_CCR1_EDGE_DIS_ALL, cct_base + MEC_CCT_CCR1_OFS);

    /* disable and clear all CCT GIRQ bits */
    for (i = 0; i < MEC_ARRAY_SIZE(cct_ecia_tbl); i++) {
        mec_hal_girq_ctrl(cct_ecia_tbl[i], 0);
        mec_hal_girq_clr_src(cct_ecia_tbl[i]);
    }

    /* ungate clocks in the CCT */
    cct_cr_rmw(cct_base, MEC_BIT(MEC_CCT_CR_ACTV_POS), MEC_BIT(MEC_CCT_CR_ACTV_POS));

    /* set free run counter clock divider */
    r = MEC_CCT_CR_TCLK_SET(cfg->fr_clk_div);
    cct_cr_rmw(cct_base, r, MEC_CCT_CR_TCLK_MSK);

    /* program comparator match values */
    mmcr32_wr(cfg->comp0, cct_base + MEC_CCT_CMP0_OFS);
    mmcr32_wr(cfg->comp1, cct_base + MEC_CCT_CMP1_OFS);

    /* program capture controls */
    for (i = 0; i < MEC_CCT_CAP_ID_MAX; i++) {
        struct mec_cct_cap_config *capcfg = &cfg->cap_cfg[i];

        if (capcfg->enable != 0) {
            if (capcfg->prog_mux != 0) {
                r = (uint32_t)capcfg->mux_sel << (i * 4u);
                msk = 0xfu << (i * 4u);
                mmcr32_update_field(cct_base + MEC_CCT_MUX_OFS, r, msk);
            }

            union mec_cct_cap_config_u *p = (union mec_cct_cap_config_u *)capcfg;

            r = p->cap_cfg_all & 0xffu;
            msk = 0xffu;

            if (i < MEC_CCT_CAP4_ID) {
                r <<= (i * 8u);
                msk <<= (i * 8u);
                ofs = MEC_CCT_CCR0_OFS;
            } else {
                r <<= ((i - MEC_CCT_CAP4_ID) * 8u);
                msk <<= ((i - MEC_CCT_CAP4_ID) * 8u);
                ofs = MEC_CCT_CCR1_OFS;
            }

            mmcr32_update_field(cct_base + ofs, r, 0xffu);
        }
    }

    if (cfg->flags & MEC_CCT_CFG_CMP0_EN) {
        cct_cr_rmw(cct_base, MEC_BIT(MEC_CCT_CR_CMP0_EN_POS), MEC_BIT(MEC_CCT_CR_CMP0_EN_POS));
    }

    if (cfg->flags & MEC_CCT_CFG_CMP1_EN) {
        cct_cr_rmw(cct_base, MEC_BIT(MEC_CCT_CR_CMP1_EN_POS), MEC_BIT(MEC_CCT_CR_CMP1_EN_POS));
    }

    if (cfg->flags & MEC_CCT_CFG_FR_EN) {
        mec_hal_cct_fr_enable(cct_base);
    }

    return MEC_RET_OK;
}

/* end mec_cct.c */
