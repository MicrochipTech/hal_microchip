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
#include "mec_ecia_api.h"
#include "mec_pcr_api.h"
#include "mec_cct_api.h"
#include "mec_retval.h"

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
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_FRC_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_FRC_NVIC)

#define MEC_CCT_CAP0_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP0_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP0_NVIC)

#define MEC_CCT_CAP1_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP1_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP1_NVIC)

#define MEC_CCT_CAP2_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP2_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP2_NVIC)

#define MEC_CCT_CAP3_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP3_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP3_NVIC)

#define MEC_CCT_CAP4_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP4_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP4_NVIC)

#define MEC_CCT_CAP5_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CAP5_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CAP5_NVIC)

#define MEC_CCT_CMP0_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CMP0_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CMP0_NVIC)

#define MEC_CCT_CMP1_ECIA_INFO                                                                     \
    MEC5_ECIA_INFO(MEC_CCT_GIRQ, MEC_CCT_CMP1_GIRQ_POS, MEC_CCT_GIRQ_NVIC, MEC_CCT_CMP1_NVIC)

static const uint32_t cct_ecia_tbl[] = {
    MEC_CCT_FRC_ECIA_INFO, MEC_CCT_CAP0_ECIA_INFO, MEC_CCT_CAP1_ECIA_INFO,
    MEC_CCT_CAP2_ECIA_INFO, MEC_CCT_CAP3_ECIA_INFO, MEC_CCT_CAP4_ECIA_INFO,
    MEC_CCT_CAP5_ECIA_INFO, MEC_CCT_CMP0_ECIA_INFO, MEC_CCT_CMP1_ECIA_INFO,
};

static uint32_t set_cap_ctrl_edge(uint32_t cap_ctrl, uint8_t capid, uint8_t edge)
{
    uint32_t cc = 0;
    uint8_t pos = 0;

    if (capid < MEC_CCT_CAP4_ID) {
        pos = capid * 8u;
    } else {
        pos = (capid - MEC_CCT_CAP4_ID) * 8u;
    }

    cc = (cap_ctrl & (uint32_t)~(0x3u << pos)) | ((edge & 0x3u) << pos);

    return cc;
}

static uint32_t set_cap_filt(uint32_t cap_ctrl, uint8_t capid, uint8_t filt)
{
    uint32_t cc = 0;
    uint8_t pos = 0;

    if (capid < MEC_CCT_CAP4_ID) {
        pos = capid * 8u;
    } else {
        pos = (capid - MEC_CCT_CAP4_ID) * 8u;
    }

    if (filt >= MEC_CCT_FCLK_DIS) {
        cc = cap_ctrl | MEC_BIT(2 + pos);
    } else {
        cc = cap_ctrl &= (uint32_t)~MEC_BIT(2 + pos);
        pos += 5u;
        cc = cap_ctrl & (uint32_t)~(0x7u << pos);
        cc |= ((filt & 0x7u) << pos);
    }

    return cc;
}

static uint32_t set_cap_mux(uint32_t cct_mux, uint8_t capid, uint8_t muxval)
{
    uint8_t pos = (capid * 4u);
    uint32_t cm = cct_mux & (uint32_t)~(0xfu << pos);

    cm |= ((uint32_t)(muxval & 0xfu) << pos);

    return cm;
}

int mec_hal_cct_cap_cfg(struct mec_cct_regs *regs, uint32_t capcfg)
{
    uint32_t cc = 0, cmux = 0;
    uint8_t capid = 0, cval = 0;

    if (!regs) {
        return MEC_RET_ERR_INVAL;
    }

    capid = (uint8_t)MEC_CCT_CAP_ID_FROM_INFO(capcfg);
    if (capid < MEC_CCT_CAP4_ID) {
        cc = regs->CAP_CTRL0;
        cc = set_cap_ctrl_edge(cc, capid, MEC_CCT_CAP_IDET_DIS);
        regs->CAP_CTRL0 = cc;
    } else {
        cc = regs->CAP_CTRL1;
        cc = set_cap_ctrl_edge(cc, capid, MEC_CCT_CAP_IDET_DIS);
        regs->CAP_CTRL1 = cc;
    }

    mec_hal_cct_clear_irq(regs, MEC_BIT(MEC_CCT_CAP0_GIRQ_POS + capid));

    cval = (uint8_t)MEC_CCT_CAP_FCLK_FROM_INFO(capcfg);
    cc = set_cap_filt(cc, capid, cval);

    cval = (uint8_t)MEC_CCT_CAP_MUX_FROM_INFO(capcfg);
    cmux = set_cap_mux(regs->MUX_SEL, capid, cval);

    cval = (uint8_t)MEC_CCT_CAP_EDGE_FROM_INFO(capcfg);
    cc = set_cap_ctrl_edge(cc, capid, cval);

    regs->MUX_SEL = cmux;
    if (capid < MEC_CCT_CAP4_ID) {
        regs->CAP_CTRL0 = cc;
    } else {
        regs->CAP_CTRL1 = cc;
    }

    return MEC_RET_OK;
}

void mec_hal_cct_clear_irq(struct mec_cct_regs *regs, uint32_t imsk)
{
    uint32_t girq_bm = (imsk << 20);

    if (!regs) {
        return;
    }

    mec_hal_girq_bm_clr_src(MEC_CCT_GIRQ, girq_bm);
}

void mec_hal_cct_cap_ctrl(struct mec_cct_regs *regs, uint8_t capid, uint8_t edge)
{
    uint32_t edge_enc = 0;
    uint8_t pos = 0;

    if (!regs || (capid >= MEC_CCT_CAP_ID_MAX)) {
        return;
    }

    if (edge < MEC_CCT_CAP_IDET_DIS) {
        edge_enc = edge;
    } else {
        edge_enc = MEC_CCT_CAP_IDET_DIS;
    }

    if (capid < MEC_CCT_CAP4_ID) {
        pos = capid * 8u;
        edge_enc <<= pos;
        regs->CAP_CTRL0 = ((regs->CAP_CTRL0 & (uint32_t)~(0x3u << pos)) | edge_enc);
    } else {
        pos = (capid - MEC_CCT_CAP4_ID) * 8u;
        edge_enc <<= pos;
        regs->CAP_CTRL1 = ((regs->CAP_CTRL1 & (uint32_t)~(0x3u << pos)) | edge_enc);
    }
}

uint8_t mec_hal_cct_fr_clkdiv_get(struct mec_cct_regs *regs)
{
    if (!regs) {
        return 0xffu;
    }

    return (uint8_t)((regs->TCTRL & MEC_CCT_TCTRL_TCLK_FREQ_Msk) >> MEC_CCT_TCTRL_TCLK_FREQ_Pos);
}

void mec_hal_cct_enable_irq(struct mec_cct_regs *regs, uint32_t ienbm)
{
    uint32_t girqbm = (ienbm & 0x1ffu) << MEC_CCT_FRC_GIRQ_POS;

    mec_hal_girq_bm_en(MEC_CCT_GIRQ, girqbm, 1u);
}

void mec_hal_cct_disable_irq(struct mec_cct_regs *regs, uint32_t ienbm)
{
    uint32_t girqbm = (ienbm & 0x1ffu) << MEC_CCT_FRC_GIRQ_POS;

    mec_hal_girq_bm_en(MEC_CCT_GIRQ, girqbm, 0);
}

int mec_hal_cct_init(struct mec_cct_regs *regs, struct mec_cct_config *cfg)
{
    uint32_t capcfg = 0;
    size_t i = 0;

    if (!regs || !cfg) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_pcr_clr_blk_slp_en(MEC_PCR_CCT0);
    mec_hal_pcr_blk_reset(MEC_PCR_CCT0);

    /* disable all edge detection */
    regs->CAP_CTRL0 = 0x03030303u;
    regs->CAP_CTRL1 = 0x03030303u;

    /* disable and clear all CCT GIRQ bits */
    for (i = 0; i < MEC_ARRAY_SIZE(cct_ecia_tbl); i++) {
        mec_hal_girq_ctrl(cct_ecia_tbl[i], 0);
        mec_hal_girq_clr_src(cct_ecia_tbl[i]);
    }

    /* ungate clocks in the CCT */
    regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_ACTV_Pos);

    /* set free run counter clock divider */
    regs->TCTRL |= (((uint32_t)cfg->fr_clk_div << MEC_CCT_TCTRL_TCLK_FREQ_Pos) &
                    MEC_CCT_TCTRL_TCLK_FREQ_Msk);

    /* program capture controls */
    for (i = 0; i < MEC_CCT_CAP_ID_MAX; i++) {
        if (cfg->cap_cfg[i] & MEC_CCT_CAP_VALID) {
            capcfg = (uint32_t)cfg->cap_cfg[i];
            mec_hal_cct_cap_cfg(regs, cfg->cap_cfg[i]);
        }
    }

    if (cfg->flags & MEC_CCT_CFG_CMP0_EN) {
        regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_COMP0_CLR_Pos);
        regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_COMP0_EN_Pos);
    }

    if (cfg->flags & MEC_CCT_CFG_CMP1_EN) {
        regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_COMP1_CLR_Pos);
        regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_COMP1_EN_Pos);
    }

    if (cfg->flags & MEC_CCT_CFG_FR_EN) {
        regs->TCTRL |= MEC_BIT(MEC_CCT_TCTRL_FREN_Pos);
    }

    return MEC_RET_OK;
}


/* end mec_cct.c */
