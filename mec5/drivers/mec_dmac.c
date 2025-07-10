/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include <mec_dma_regs.h>

#include "mec_pcfg.h"
#include "mec_defs.h"
#include "mec_dmac_api.h"
#include "mec_ecia_api.h"
#include "mec_pcr_api.h"
#include "mec_retval.h"
#include "mec_mmcr.h"

/* #define MEC_DMAC_DEBUG_REGS */

#define MEC_DMA_CHAN_REGS_SIZE 0x40u

#define MEC_DMA_CHAN_ALL_STATUS (MEC_BIT(MEC_DMA_CHAN_ISTATUS_BERR_Pos) \
                                 | MEC_BIT(MEC_DMA_CHAN_ISTATUS_HFCREQ_Pos) \
                                 | MEC_BIT(MEC_DMA_CHAN_ISTATUS_DONE_Pos) \
                                 | MEC_BIT(MEC_DMA_CHAN_ISTATUS_HFCTERM_Pos))

#define MEC_DMA_CHAN_STOP_WAIT 256u

#define MEC_DMAC_GIRQ           14
#define MEC_DMAC_GIRQ_IDX       6
#define MEC_DMAC_GIRQ_NVIC_NUM  6

#if MEC5_DMAC_NUM_CHANNELS != 0
#define MEC_DMAC_ALL_CHAN_MASK MEC_GENMASK((MEC5_DMAC_NUM_CHANNELS - 1u), 0)
#else
#define MEC_DMAC_ALL_CHAN_MASK 0
#endif

#define MEC_DMAC_CHAN0_ECIA_INFO  MEC_ECIA_INFO(14, 0, 6, 24)
#define MEC_DMAC_CHAN1_ECIA_INFO  MEC_ECIA_INFO(14, 1, 6, 25)
#define MEC_DMAC_CHAN2_ECIA_INFO  MEC_ECIA_INFO(14, 2, 6, 26)
#define MEC_DMAC_CHAN3_ECIA_INFO  MEC_ECIA_INFO(14, 3, 6, 27)
#define MEC_DMAC_CHAN4_ECIA_INFO  MEC_ECIA_INFO(14, 4, 6, 28)
#define MEC_DMAC_CHAN5_ECIA_INFO  MEC_ECIA_INFO(14, 5, 6, 29)
#define MEC_DMAC_CHAN6_ECIA_INFO  MEC_ECIA_INFO(14, 6, 6, 30)
#define MEC_DMAC_CHAN7_ECIA_INFO  MEC_ECIA_INFO(14, 7, 6, 31)
#define MEC_DMAC_CHAN8_ECIA_INFO  MEC_ECIA_INFO(14, 8, 6, 32)
#define MEC_DMAC_CHAN9_ECIA_INFO  MEC_ECIA_INFO(14, 9, 6, 33)
#define MEC_DMAC_CHAN10_ECIA_INFO MEC_ECIA_INFO(14, 10, 6, 34)
#define MEC_DMAC_CHAN11_ECIA_INFO MEC_ECIA_INFO(14, 11, 6, 35)
#define MEC_DMAC_CHAN12_ECIA_INFO MEC_ECIA_INFO(14, 12, 6, 36)
#define MEC_DMAC_CHAN13_ECIA_INFO MEC_ECIA_INFO(14, 13, 6, 37)
#define MEC_DMAC_CHAN14_ECIA_INFO MEC_ECIA_INFO(14, 14, 6, 38)
#define MEC_DMAC_CHAN15_ECIA_INFO MEC_ECIA_INFO(14, 15, 6, 39)
#if MEC5_DMAC_NUM_CHANNELS == 20
#define MEC_DMAC_CHAN16_ECIA_INFO MEC_ECIA_INFO(14, 16, 6, 194)
#define MEC_DMAC_CHAN17_ECIA_INFO MEC_ECIA_INFO(14, 17, 6, 195)
#define MEC_DMAC_CHAN18_ECIA_INFO MEC_ECIA_INFO(14, 18, 6, 196)
#define MEC_DMAC_CHAN19_ECIA_INFO MEC_ECIA_INFO(14, 19, 6, 197)
#endif

const uint32_t dmac_ecia_info_table[MEC5_DMAC_NUM_CHANNELS] = {
    MEC_DMAC_CHAN0_ECIA_INFO, MEC_DMAC_CHAN1_ECIA_INFO,
    MEC_DMAC_CHAN2_ECIA_INFO, MEC_DMAC_CHAN3_ECIA_INFO,
    MEC_DMAC_CHAN4_ECIA_INFO, MEC_DMAC_CHAN5_ECIA_INFO,
    MEC_DMAC_CHAN6_ECIA_INFO, MEC_DMAC_CHAN7_ECIA_INFO,
    MEC_DMAC_CHAN8_ECIA_INFO, MEC_DMAC_CHAN9_ECIA_INFO,
    MEC_DMAC_CHAN10_ECIA_INFO, MEC_DMAC_CHAN11_ECIA_INFO,
    MEC_DMAC_CHAN12_ECIA_INFO, MEC_DMAC_CHAN13_ECIA_INFO,
    MEC_DMAC_CHAN14_ECIA_INFO, MEC_DMAC_CHAN15_ECIA_INFO,
#if MEC5_DMAC_NUM_CHANNELS == 20
    MEC_DMAC_CHAN16_ECIA_INFO, MEC_DMAC_CHAN17_ECIA_INFO,
    MEC_DMAC_CHAN18_ECIA_INFO, MEC_DMAC_CHAN19_ECIA_INFO,
#endif
};

#ifdef MEC_DMAC_DEBUG_REGS
struct mec_dma_chan_regs_save {
    uint32_t  actv;
    uint32_t  mstart;
    uint32_t  mend;
    uint32_t  dstart;
    uint32_t  ctrl;
    uint32_t  istatus;
    uint32_t  ien;
    uint32_t  fsm;
};

struct mec_dma_chan_regs_save dbg_mec_dma[MEC5_DMAC_NUM_CHANNELS];
#endif

static uint32_t dmac_get_ecia_info(uint32_t channel)
{
    if (channel < MEC5_DMAC_NUM_CHANNELS) {
        return dmac_ecia_info_table[channel];
    }

    return UINT32_MAX;
}

static void dma_chan_ia_enable(uint32_t channel)
{
    uint32_t devi = dmac_get_ecia_info(channel);

    mec_hal_girq_ctrl(devi, 1u);
}

static void dma_chan_ia_disable(uint32_t channel)
{
    uint32_t devi = dmac_get_ecia_info(channel);

    mec_hal_girq_ctrl(devi, 0);
}

static void dmac_clr_ia_status(uint32_t channel)
{
    uint32_t devi = dmac_get_ecia_info(channel);

    mec_hal_girq_clr_src(devi);
}

static void dma_clr_ia_all(void)
{
    for (uint8_t chan = 0; chan < MEC5_DMAC_NUM_CHANNELS; chan++) {
        mec_hal_girq_clr_src(dmac_ecia_info_table[chan]);
    }
}

uint32_t mec_hal_dmac_girq_result(void)
{
    return mec_hal_girq_result_get(MEC_DMAC_GIRQ);
}

void mec_hal_dmac_girq_aggr(uint8_t enable)
{
    mec_hal_ecia_girq_aggr_enable(MEC_DMAC_GIRQ, enable);
}

void mec_hal_dmac_aggr_nvic_ien(uint8_t enable)
{
    if (enable) {
        NVIC_EnableIRQ(MEC_DMAC_GIRQ_NVIC_NUM);
    } else {
        NVIC_DisableIRQ(MEC_DMAC_GIRQ_NVIC_NUM);
    }
}

int mec_hal_dmac_reset(void)
{
    uintptr_t ba = MEC_DMA_BASE_ADDR;
    uint32_t retries = 1000u;

    mmcr32_set_bit(ba + MEC_DMAM_CR_OFS, MEC_DMAM_CR_SRST_POS); /* self-clearing */

    while (mmcr32_test_bit(ba + MEC_DMAM_CR_OFS, MEC_DMAM_CR_SRST_POS)) {
        if (retries == 0) {
            return MEC_RET_ERR_TIMEOUT;
        }
        retries--;
    }

    return MEC_RET_OK;
}

int mec_hal_dmac_enable(uint8_t enable)
{
    uintptr_t ba = MEC_DMA_BASE_ADDR;

    if (enable) {
        mmcr32_set_bit(ba + MEC_DMAM_CR_OFS, MEC_DMAM_CR_ACTV_POS);
    } else {
        mmcr32_clr_bit(ba + MEC_DMAM_CR_OFS, MEC_DMAM_CR_ACTV_POS);
    }

    return MEC_RET_OK;
}

bool mec_hal_dmac_is_enabled(void)
{
    uintptr_t ba = MEC_DMA_BASE_ADDR;

    if (mmcr32_test_bit(ba + MEC_DMAM_CR_OFS, MEC_DMAM_CR_ACTV_POS)) {
        return true;
    }

    return false;
}

int mec_hal_dma_chan_ia_status_clr(enum mec_dmac_channel channel)
{
    if (channel >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    dmac_clr_ia_status(channel);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ia_status_clr_mask(uint32_t chanmsk)
{
    chanmsk &= MEC_DMAC_ALL_CHAN_MASK;

    if (chanmsk == 0) {
        return MEC_RET_OK;
    }

    mec_hal_girq_bm_clr_src(MEC_DMA_GIRQ, chanmsk);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ia_enable(enum mec_dmac_channel channel)
{
    if (channel >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    dma_chan_ia_enable(channel);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ia_disable(enum mec_dmac_channel channel)
{
    if (channel >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    dma_chan_ia_disable(channel);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ia_enable_mask(uint32_t chan_mask)
{
    if (chan_mask == 0) {
        mec_hal_girq_bm_en(MEC_DMA_GIRQ, chan_mask & MEC_DMAC_ALL_CHAN_MASK, 1u);
    }

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ia_disable_mask(uint32_t chan_mask)
{
    if (chan_mask != 0) {
        mec_hal_girq_bm_en(MEC_DMA_GIRQ, chan_mask & MEC_DMAC_ALL_CHAN_MASK, 0);
    }

    return MEC_RET_OK;
}

int mec_hal_dmac_init(uint32_t chan_mask)
{
    mec_hal_pcr_clr_blk_slp_en(MEC_PCR_DMA); /* clocks gated ON */
    mec_hal_dmac_reset();
    dma_clr_ia_all();
    mec_hal_dmac_enable(1u);
    mec_hal_dma_chan_ia_enable_mask(chan_mask);

    return MEC_RET_OK;
}

uintptr_t mec_hal_dma_chan_reg_addr(enum mec_dmac_channel chan)
{
    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return 0u;
    }

    return (uintptr_t)MEC_DMA_CHAN_BASE(chan);
}

int mec_hal_dma_chan_init(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    /* write memory end address to 0 first, causing HW to see MEA <= MSA */
    mmcr32_wr(0, ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_MSA_OFS);

    mmcr32_wr(0, ba + MEC_DMACH_ACTV_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_CR_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_DSA_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_IER_OFS);
    mmcr32_wr(UINT32_MAX, ba + MEC_DMACH_SR_OFS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_intr_status(enum mec_dmac_channel chan, uint32_t *status)
{
    uintptr_t ba = 0;
    uint32_t hwsts = 0u, logical_sts = 0u;

    if ((status == NULL) || (chan >= MEC5_DMAC_NUM_CHANNELS)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    hwsts = mmcr32_rd(ba + MEC_DMACH_SR_OFS);

    if ((hwsts & MEC_BIT(MEC_DMACH_SR_BERR_POS)) != 0) {
        logical_sts |= MEC_BIT(MEC_DMA_CHAN_STS_BUS_ERR_POS);
    }

    if ((hwsts & MEC_BIT(MEC_DMACH_SR_HW_OVFL_POS)) != 0) {
        logical_sts |= MEC_BIT(MEC_DMA_CHAN_STS_HFC_OVF_POS);
    }

    if ((hwsts & MEC_BIT(MEC_DMACH_SR_DONE_POS)) != 0) {
        logical_sts |= MEC_BIT(MEC_DMA_CHAN_STS_DONE_POS);
    }

    if ((hwsts & MEC_BIT(MEC_DMACH_SR_HW_TERM_POS)) != 0) {
        logical_sts |= MEC_BIT(MEC_DMA_CHAN_STS_HFC_TERM_POS);
    }

    *status = logical_sts;

    return MEC_RET_OK;
}

int mec_hal_dma_chan_intr_status_clr(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    mmcr32_wr(UINT32_MAX, ba + MEC_DMACH_SR_OFS);
    mec_hal_girq_bm_clr_src(MEC_DMA_GIRQ, MEC_BIT(chan));

    return MEC_RET_OK;
}

int mec_hal_dma_chan_intr_en(enum mec_dmac_channel chan, uint8_t ien)
{
    uintptr_t ba = 0;
    uint32_t ien_val = 0u;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    if (ien) {
        ien_val = MEC_BIT(MEC_DMA_CHAN_IEN_DONE_Pos) | MEC_BIT(MEC_DMA_CHAN_IEN_BERR_Pos);
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    mmcr32_wr(ien_val, ba + MEC_DMACH_IER_OFS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_go(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0;
    uint8_t start_pos = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    ctrl = mmcr32_rd(ba + MEC_DMACH_CR_OFS);

    start_pos = MEC_DMACH_CR_HFC_RUN_POS;
    if (ctrl & MEC_BIT(MEC_DMACH_CR_DHFC_POS)) {
        start_pos = MEC_DMACH_CR_SFC_GO_POS;
    }

    ctrl &= (uint32_t)~(MEC_BIT(MEC_DMACH_CR_DHFC_POS) | MEC_BIT(MEC_DMACH_CR_HFC_RUN_POS));

#ifdef MEC_DMAC_DEBUG_REGS
    dbg_mec_dma[chan].actv = mmcr32_rd(ba + MEC_DMACH_ACTV_OFS);
    dbg_mec_dma[chan].mstart = mmcr32_rd(ba + MEC_DMACH_MSA_OFS);
    dbg_mec_dma[chan].mend = mmcr32_rd(ba + MEC_DMACH_MEA_OFS);
    dbg_mec_dma[chan].dstart = mmcr32_rd(ba + MEC_DMACH_DSA_OFS);
    dbg_mec_dma[chan].ctrl = mmcr32_rd(ba + MEC_DMACH_CR_OFS) | MEC_BIT(start_pos);
    dbg_mec_dma[chan].istatus = mmcr32_rd(ba + MEC_DMACH_SR_OFS);
    dbg_mec_dma[chan].ien = mmcr32_rd(ba + MEC_DMACH_IER_OFS);
    dbg_mec_dma[chan].fsm = mmcr32_rd(ba + MEC_DMACH_FSM_OFS);
#endif

    mmcr32_set_bit(ba + MEC_DMACH_CR_OFS, start_pos);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_start(enum mec_dmac_channel chan)
{
    return mec_hal_dma_chan_go(chan);
}

int mec_hal_dma_chan_start2(enum mec_dmac_channel chan, uint32_t flags)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0;
    uint8_t start_pos = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    mmcr32_wr(0, ba + MEC_DMACH_IER_OFS);
    mmcr32_wr(UINT32_MAX, ba + MEC_DMACH_SR_OFS);

    ctrl = mmcr32_rd(ba + MEC_DMACH_CR_OFS);

    start_pos = MEC_DMACH_CR_HFC_RUN_POS;
    if (ctrl & MEC_BIT(MEC_DMA_CHAN_CTRL_DHFC_Pos)) {
        start_pos = MEC_DMACH_CR_SFC_GO_POS;
    }

    ctrl &= (uint32_t)~(MEC_BIT(MEC_DMACH_CR_DHFC_POS) | MEC_BIT(MEC_DMACH_CR_HFC_RUN_POS));
    mmcr32_wr(ctrl, ba + MEC_DMACH_CR_OFS);

    mmcr32_wr(flags & 0xfu, ba + MEC_DMACH_IER_OFS);
    mmcr32_set_bit(ba + MEC_DMACH_CR_OFS, start_pos);
    mmcr32_set_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);

    return MEC_RET_OK;
}

bool mec_hal_dma_chan_is_busy(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return false;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    if (mmcr32_test_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_BUSY_POS)) {
        return true;
    }

    return false;
}

int mec_hal_dma_chan_halt(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;
    uint32_t msk = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    msk = (MEC_BIT(MEC_DMACH_CR_DHFC_POS) | MEC_BIT(MEC_DMACH_CR_HFC_RUN_POS));

    mmcr32_clr_bits(ba + MEC_DMACH_CR_OFS, msk);
    mmcr32_clr_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_stop(enum mec_dmac_channel chan)
{
    uintptr_t ba = 0;
    uint32_t mask = 0;
    uint32_t wait_cnt = MEC_DMA_CHAN_STOP_WAIT;
    int ret = MEC_RET_OK;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    if (mmcr32_test_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_BUSY_POS)) {
        mmcr32_set_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_ABORT_POS);

        /* should stop on next byte boundary */
        while (mmcr32_test_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_BUSY_POS)) {
            if (wait_cnt == 0) {
                ret = MEC_RET_ERR_TIMEOUT;
                break;
            }
            wait_cnt--;
        }
    }

    mask = (MEC_BIT(MEC_DMACH_CR_DHFC_POS) | MEC_BIT(MEC_DMACH_CR_HFC_RUN_POS));
    mmcr32_clr_bits(ba + MEC_DMACH_CR_OFS, mask);
    mmcr32_clr_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);

    return ret;
}

int mec_hal_dma_chan_ctrl_get(enum mec_dmac_channel chan, uint32_t *ctrl)
{
    uintptr_t ba = 0;

    if ((chan >= MEC5_DMAC_NUM_CHANNELS) || (ctrl == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);
    *ctrl = mmcr32_rd(ba + MEC_DMACH_CR_OFS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_ien(enum mec_dmac_channel chan, uint8_t iflags, uint8_t enable)
{
    uintptr_t ba = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    if (enable != 0) {
        mmcr32_set_bits(ba + MEC_DMACH_IER_OFS, (uint32_t)iflags);
    } else {
        mmcr32_clr_bits(ba + MEC_DMACH_IER_OFS, (uint32_t)iflags);
    }

    return MEC_RET_OK;
}

int mec_hal_chan_regs_get(enum mec_dmac_channel chan, uint32_t *regbuf, uint8_t n)
{
    uintptr_t ba = 0;

    if ((chan >= MEC5_DMAC_NUM_CHANNELS) || (regbuf == NULL) || (n < 8u)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    for (int i = 0; i < 8; i++) {
        regbuf[i] = mmcr32_rd(ba);
        ba += 4u;
    }

    return MEC_RET_OK;
}

int mec_hal_dma_chan_hwfc_set(enum mec_dmac_channel chan, enum mec_dmac_hwfc_dev_id hwfc_dev,
                              uintptr_t dev_addr)
{
    uintptr_t ba = 0;
    uint32_t val = 0;

    if ((chan >= MEC5_DMAC_NUM_CHANNELS) || (hwfc_dev >= MEC_DMAC_DEV_ID_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    mmcr32_wr(dev_addr, ba + MEC_DMACH_DSA_OFS);

    val = MEC_DMACH_CR_HFCD_SET((uint32_t)hwfc_dev);
    mmcr32_update_field(ba + MEC_DMACH_CR_OFS, val, MEC_DMACH_CR_HFCD_MSK);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_dir_set(enum mec_dmac_channel chan, enum mec_dmac_dir dir)
{
    uintptr_t ba = 0;

    if ((chan >= MEC5_DMAC_NUM_CHANNELS) || (dir >= MEC_DMAC_DIR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    if (dir == MEC_DMAC_DIR_MEM_TO_DEV) {
        mmcr32_set_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_M2D_POS);
    } else {
        mmcr32_clr_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_M2D_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_dma_chan_dir_get(enum mec_dmac_channel chan, enum mec_dmac_dir *dir)
{
    uintptr_t ba = 0;

    if ((dir == NULL) || (chan >= MEC5_DMAC_NUM_CHANNELS)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    if (mmcr32_test_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_M2D_POS)) {
        *dir = MEC_DMAC_DIR_MEM_TO_DEV;
    } else {
        *dir = MEC_DMAC_DIR_DEV_TO_MEM;
    }

    return MEC_RET_OK;
}

int mec_hal_dma_chan_mem_set(enum mec_dmac_channel chan, uintptr_t maddr, size_t nbytes)
{
    uintptr_t ba = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    mmcr32_wr(maddr, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(maddr + nbytes, ba + MEC_DMACH_MEA_OFS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_mem_units_set(enum mec_dmac_channel chan, enum mec_dmac_unit_size unitsz)
{
    uintptr_t ba = 0;
    uint32_t v = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    v = 1u;
    if ((unitsz == 4u) || (unitsz == 2u)) {
        v = unitsz;
    }

    v = MEC_DMACH_CR_UNIT_SET(v);
    mmcr32_update_field(ba + MEC_DMACH_CR_OFS, v, MEC_DMACH_CR_UNIT_MSK);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_rem_bytes(enum mec_dmac_channel chan, uint32_t *remsz)
{
    uintptr_t ba = 0;
    uint32_t mstart = 0, mend = 0, nrem = 0;

    if ((remsz == NULL) || (chan >= MEC5_DMAC_NUM_CHANNELS)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    nrem = 0u;
    mstart = mmcr32_rd(ba + MEC_DMACH_MSA_OFS);
    mend = mmcr32_rd(ba + MEC_DMACH_MEA_OFS);

    if (mend > mstart) {
        nrem = mend - mstart;
    }

    *remsz = nrem;

    return 0;
}

int mec_hal_dma_chan_reload(enum mec_dmac_channel chan, uintptr_t src, uintptr_t dest,
                            size_t nbytes)
{
    uintptr_t ba = 0;
    uint32_t v = 0, ms = 0, ds = 0;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    /* ensure HW is "done" by mstart == mend */
    v = mmcr32_rd(ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(v, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_MEA_OFS);

    if (mmcr32_test_bit(ba + MEC_DMACH_CR_OFS, MEC_DMACH_CR_M2D_POS)) {
        ms = src;
        ds = dest;
    } else { /* device to memory */
        ms = dest;
        ds = src;
    }

    mmcr32_wr(ms, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(ms + nbytes, ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(ds, ba + MEC_DMACH_DSA_OFS);

    return MEC_RET_OK;
}

/* Configure a DMA channel for transfer.
 * DMA termination is based on channel memory start address incrementing until it matches
 * the memory end address. Device has start address register but no end address.
 * Control register has a bit to select the direction Memory to Device or Device to Memory.
 * Memory to Device: source is memory, destination is device.
 *  Control direction bit = 1 (Mem2Dev)
 *  Control Increment Mem = 1
 *  Control Increment Dev is optional. Current MCHP peripherals which can use central DMA
 *  expose their data as a single data register.
 *  Memory Start address reg = source address
 *  Memory End addresss reg = source address + transfer size in bytes
 *  Device Start address reg = destination address
 *
 * Device to Memory: source is device HW register, destination is memory
 *  Control direction bit = 0 (Dev2Mem)
 *  Control Increment Mem = 1 may be 0 if writing same value to device.
 *  Control Increment Dev is optional.
 *  Memory Start address reg = destination address (memory)
 *  Memory End address regs = destination address + transfer size in bytes
 *  Device Start address reg = source address (device register address)
 */
int mec_hal_dma_chan_cfg(enum mec_dmac_channel chan, struct mec_dma_cfg *cfg)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0u; /* dir = Dev2Mem, IncrMem=0, IncrDev=0 */
    uint32_t ms = 0u, ds = 0u;
    uint32_t usz = 0u;

    if ((cfg == NULL) || (chan >= MEC5_DMAC_NUM_CHANNELS)) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_dma_chan_init(chan);

    usz = 1u;
    if ((cfg->unitsz == 4u) || (cfg->unitsz == 2u)) {
        usz = cfg->unitsz;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    ctrl = MEC_DMACH_CR_UNIT_SET(usz);
    ctrl |= MEC_DMACH_CR_HFCD_SET(cfg->hwfc_dev);

    if (cfg->flags & MEC_DMA_CFG_FLAG_SWFLC) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_DHFC_POS);
    }

    if (cfg->dir == MEC_DMAC_DIR_MEM_TO_DEV) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_M2D_POS);

        ms = cfg->src_addr;
        ds = cfg->dst_addr;

        if ((cfg->flags & MEC_DMA_CFG_FLAG_INCR_SRC_ADDR) != 0) {
            ctrl |= MEC_BIT(MEC_DMACH_CR_INCM_POS);
        }

        if ((cfg->flags & MEC_DMA_CFG_FLAG_INCR_DST_ADDR) != 0) {
            ctrl |= MEC_BIT(MEC_DMACH_CR_INCD_POS);
        }
    } else { /* device(source address) to memory(destination address) */
        ms = cfg->dst_addr;
        ds = cfg->src_addr;

        if ((cfg->flags & MEC_DMA_CFG_FLAG_INCR_SRC_ADDR) != 0) {
            ctrl |= MEC_BIT(MEC_DMACH_CR_INCD_POS);
        }

        if ((cfg->flags & MEC_DMA_CFG_FLAG_INCR_DST_ADDR) != 0) {
            ctrl |= MEC_BIT(MEC_DMACH_CR_INCM_POS);
        }
    }

    mmcr32_wr(ms, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(ms + cfg->nbytes, ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(ds, ba + MEC_DMACH_DSA_OFS);
    mmcr32_wr(ctrl, ba + MEC_DMACH_CR_OFS);
    mmcr32_set_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_cfg_get(enum mec_dmac_channel chan, struct mec_dma_cfg *cfg)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0u, dstart = 0u, mstart = 0u, mend = 0u;

    if ((cfg != NULL) || (chan >= MEC5_DMAC_NUM_CHANNELS)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    cfg->flags = 0u;

    ctrl = mmcr32_rd(ba + MEC_DMACH_CR_OFS);

    if (ctrl & MEC_BIT(MEC_DMACH_CR_DHFC_POS)) {
        cfg->flags |= MEC_DMA_CFG_FLAG_SWFLC;
    }

    cfg->hwfc_dev = MEC_DMACH_CR_HFCD_GET(ctrl);

    if (ctrl & MEC_BIT(MEC_DMACH_CR_M2D_POS)) {
        cfg->dir = MEC_DMAC_DIR_MEM_TO_DEV;

        if (ctrl & MEC_BIT(MEC_DMACH_CR_INCM_POS)) {
            cfg->flags |= MEC_DMA_CFG_FLAG_INCR_SRC_ADDR;
        }

        if (ctrl & MEC_BIT(MEC_DMACH_CR_INCD_POS)) {
            cfg->flags |= MEC_DMA_CFG_FLAG_INCR_DST_ADDR;
        }
    } else {
        cfg->dir = MEC_DMAC_DIR_DEV_TO_MEM;

        if (ctrl & MEC_BIT(MEC_DMACH_CR_INCM_POS)) {
            cfg->flags |= MEC_DMA_CFG_FLAG_INCR_DST_ADDR;
        }

        if (ctrl & MEC_BIT(MEC_DMACH_CR_INCD_POS)) {
            cfg->flags |= MEC_DMA_CFG_FLAG_INCR_SRC_ADDR;
        }
    }

    cfg->nbytes = 0u;
    mstart = mmcr32_rd(ba + MEC_DMACH_MSA_OFS);
    mend = mmcr32_rd(ba + MEC_DMACH_MEA_OFS);
    dstart = mmcr32_rd(ba + MEC_DMACH_DSA_OFS);

    if (mend > mstart) {
        cfg->nbytes = mend - mstart;
    }

    if (ctrl & MEC_BIT(MEC_DMACH_CR_M2D_POS)) {
        cfg->src_addr = mstart;
        cfg->dst_addr = dstart;
    } else {
        cfg->src_addr = dstart;
        cfg->dst_addr = mstart;
    }

    return MEC_RET_OK;
}

/* chan_cfg
 * b[0] = direction: 0=dev2Mem, 1=Mem2Dev
 * b[7:1] = 7-bit HWFlowCtrl Device ID
 * b[8] = 0 do not increment memory address, 1 increment memory address
 * b[9] = 0 do not increment device address, 1 increment device address
 * b[10] = 0 do not lock channel in arbiter, 1 lock channel in arbiter
 * b[11] = 0 HW flow control, 1 = SW flow control
 * b[15:12] = transfer size units: 1, 2, or 4
 */
int mec_hal_dma_chan_cfg2(enum mec_dmac_channel chan, uint32_t nbytes,
                          uint32_t maddr, uint32_t daddr, uint32_t chan_cfg)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0u;
    uint32_t temp = 0u;
    uint8_t ien = 0u;

    if (chan >= MEC5_DMAC_NUM_CHANNELS) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    mmcr32_clr_bits(ba + MEC_DMACH_CR_OFS,
                    (MEC_BIT(MEC_DMACH_CR_HFC_RUN_POS) | MEC_BIT(MEC_DMACH_CR_SFC_GO_POS)));
    
    mmcr32_wr(0, ba + MEC_DMACH_IER_OFS);
    mmcr32_wr(UINT32_MAX, ba + MEC_DMACH_SR_OFS);

    temp = MEC_HAL_DMA_CHAN_CFG_GET_HWDEV(chan_cfg);
    ctrl = MEC_DMACH_CR_HFCD_SET(temp); 

    temp = MEC_HAL_DMA_CHAN_CFG_GET_UNITSZ(chan_cfg);
    ctrl |= MEC_DMACH_CR_UNIT_SET(temp);

    if (MEC_HAL_DMA_CHAN_CFG_IS_INCRM(chan_cfg)) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_INCM_POS);
    }

    if (MEC_HAL_DMA_CHAN_CFG_IS_INCRD(chan_cfg)) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_INCD_POS);
    }

    if (MEC_HAL_DMA_CHAN_CFG_IS_LOCK(chan_cfg)) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_LOCK_ARB_POS);
    }

    if (MEC_HAL_DMA_CHAN_CFG_IS_MEM2DEV(chan_cfg)) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_M2D_POS);
    }

    if (MEC_HAL_DMA_CHAN_CFG_IS_SW_FLCM(chan_cfg)) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_DHFC_POS);
    }

    if (chan_cfg & MEC_HAL_DMA_CHAN_CFG_DONE_IEN) {
        ien |= MEC_BIT(MEC_DMACH_IER_DONE_POS);
    }

    if (chan_cfg & MEC_HAL_DMA_CHAN_CFG_BERR_IEN) {
        ien |= MEC_BIT(MEC_DMACH_IER_BERR_POS);
    }

    mmcr32_wr(maddr, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(maddr + nbytes, ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(daddr, ba + MEC_DMACH_DSA_OFS);

    mmcr32_wr(ien, ba + MEC_DMACH_IER_OFS);
    mmcr32_wr(ctrl, ba + MEC_DMACH_CR_OFS);
    mmcr32_set_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);

    return MEC_RET_OK;
}

int mec_hal_dma_chan_cfg3(enum mec_dmac_channel chan, struct mec_dma_cfg3 *cfg3)
{
    uintptr_t ba = 0;
    uint32_t ctrl = 0;
    uint8_t ien = 0;

    if ((chan >= MEC5_DMAC_NUM_CHANNELS) || (cfg3 == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    ba = MEC_DMA_CHAN_BASE(chan);

    mmcr32_wr(0, ba + MEC_DMACH_CR_OFS);
    mmcr32_wr(0, ba + MEC_DMACH_IER_OFS);
    mmcr32_set_bit(ba + MEC_DMACH_ACTV_OFS, MEC_DMACH_ACTV_EN_POS);
    mmcr32_wr(UINT32_MAX, ba + MEC_DMACH_SR_OFS);

    mmcr32_wr(cfg3->mem_addr, ba + MEC_DMACH_MSA_OFS);
    mmcr32_wr(cfg3->mem_addr + cfg3->nbytes, ba + MEC_DMACH_MEA_OFS);
    mmcr32_wr(cfg3->dev_addr, ba + MEC_DMACH_DSA_OFS);

    ctrl = MEC_DMACH_CR_HFCD_SET(cfg3->hwfc_dev);
    ctrl |= MEC_DMACH_CR_UNIT_SET(cfg3->unitsz);

    if (cfg3->dir == MEC_DMAC_DIR_MEM_TO_DEV) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_M2D_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_INCR_MEM_ADDR) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_INCM_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_INCR_DEV_ADDR) {
        ctrl |= MEC_BIT(MEC_DMACH_CR_INCD_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_DONE_IEN) {
        ien |= MEC_BIT(MEC_DMACH_IER_DONE_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_BERR_IEN) {
        ien |= MEC_BIT(MEC_DMACH_IER_BERR_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_HWFLC_ERR) {
        ien |= MEC_BIT(MEC_DMACH_IER_HW_OVFL_POS);
    }

    if (cfg3->flags & MEC_DMA_CFG3_FLAG_HWFLC_TERM) {
        ien |= MEC_BIT(MEC_DMACH_IER_HW_TERM_POS);
    }

    mmcr32_wr(ctrl, ba + MEC_DMACH_CR_OFS);
    mmcr32_wr(ien, ba + MEC_DMACH_IER_OFS);

    return MEC_RET_OK;
}

/* end mec_dmac.c */
