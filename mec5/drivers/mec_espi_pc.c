/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include "mec_defs.h"
#include "mec_ecia_api.h"
#include "mec_espi_pc.h"
#include "mec_mmcr.h"
#include "mec_espi_regs.h"

/* ---- eSPI Peripheral Channel ---- */
#define MEC_ESPI_PC_ECIA_INFO  MEC_ECIA_INFO(19, 0, 11, 103)
#define MEC_ESPI_LTR_ECIA_INFO MEC_ECIA_INFO(19, 3, 11, 106)

static uint32_t xlat_intr_to_hw(uint32_t bitmap)
{
    uint32_t hwbm = 0;

    if (bitmap & MEC_BIT(MEC_ESPI_PC_INTR_CHEN_CHG_POS)) {
        hwbm |= MEC_BIT(MEC_ESPI_PC_CHEN_CHG_POS);
    }

    if (bitmap & MEC_BIT(MEC_ESPI_PC_INTR_BMEN_CHG_POS)) {
        hwbm |= MEC_BIT(MEC_ESPI_PC_BMEN_CHG_POS);
    }

    if (bitmap & MEC_BIT(MEC_ESPI_PC_INTR_BERR_POS)) {
        hwbm |= MEC_BIT(MEC_ESPI_PC_ABERR_POS);
    }

    return hwbm;
}

/* ---- Public API ---- */

void mec_hal_espi_pc_ready_set(uintptr_t iobase)
{
    mmcr8_set_bit(iobase + MEC_ESPI_PC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

int mec_hal_espi_pc_is_ready(uintptr_t iobase)
{
    return mmcr8_test_bit(iobase + MEC_ESPI_PC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

/* Peripheral channel enable state is status register bit[24]; move to bit[0].
 * Enable change is status register bit[25]; move to bit[1].
 */
uint32_t mec_hal_espi_pc_en_status(uintptr_t iobase)
{
    uint32_t v = mmcr32_rd(iobase + MEC_ESPI_PC_SR_OFS);

    v &= (MEC_BIT(MEC_ESPI_PC_CHEN_STATE_POS) | MEC_BIT(MEC_ESPI_PC_CHEN_CHG_POS));
    v >>= MEC_ESPI_PC_CHEN_STATE_POS;
    return v;
}

/* Peripheral channel bus master enable state is status register bit[27]; move to bit[0].
 * BM enable change is bit[28]; move to bit[1].
 * the enable's current state.
 */
uint32_t mec_hal_espi_pc_bm_status(uintptr_t iobase)
{
    uint32_t v = mmcr32_rd(iobase + MEC_ESPI_PC_SR_OFS);

    v &= (MEC_BIT(MEC_ESPI_PC_BMEN_STATE_POS) | MEC_BIT(MEC_ESPI_PC_BMEN_CHG_POS));
    v >>= MEC_ESPI_PC_BMEN_STATE_POS;
    return v;
}

uint32_t mec_hal_espi_pc_status(uintptr_t iobase)
{
    uint32_t v = mmcr32_rd(iobase + MEC_ESPI_PC_SR_OFS);
    uint32_t sts = 0;

    if ((v & MEC_BIT(MEC_ESPI_PC_ABERR_POS)) != 0) {
        sts |= MEC_BIT(MEC_ESPI_PC_ISTS_BERR_POS);
    }
    if ((v & MEC_BIT(MEC_ESPI_PC_CHEN_CHG_POS)) != 0) {
        sts |= MEC_BIT(MEC_ESPI_PC_ISTS_CHEN_CHG_POS);
    }
    if ((v & MEC_BIT(MEC_ESPI_PC_CHEN_STATE_POS)) != 0) {
        sts |= MEC_BIT(MEC_ESPI_PC_ISTS_CHEN_STATE_POS);
    }
    if ((v & MEC_BIT(MEC_ESPI_PC_BMEN_STATE_POS)) != 0) {
        sts |= MEC_BIT(MEC_ESPI_PC_ISTS_BMEN_STATE_POS);
    }
    if ((v & MEC_BIT(MEC_ESPI_PC_BMEN_CHG_POS)) != 0) {
        sts |= MEC_BIT(MEC_ESPI_PC_ISTS_BMEN_CHG_POS);
    }

    return sts;
}

void mec_hal_espi_pc_status_clr(uintptr_t iobase, uint32_t bitmap)
{
    uint32_t regval = xlat_intr_to_hw(bitmap);

    mmcr32_wr(regval, iobase + MEC_ESPI_PC_SR_OFS);
}

void mec_hal_espi_pc_intr_en(uintptr_t iobase, uint32_t bitmap)
{
    uint32_t regval = xlat_intr_to_hw(bitmap);

    mmcr32_wr(regval, iobase + MEC_ESPI_PC_IER_OFS);
}

void mec_hal_espi_pc_intr_dis(uintptr_t iobase, uint32_t bitmap)
{
    uint32_t mask = xlat_intr_to_hw(bitmap);

    mmcr32_clr_bits(iobase + MEC_ESPI_PC_IER_OFS, mask);
}

void mec_hal_espi_pc_status_clr_all(uintptr_t iobase)
{
    mmcr32_clr_bits(iobase + MEC_ESPI_PC_IER_OFS, MEC_ESPI_PC_IER_ALL_MSK);
}

uint64_t mec_hal_espi_pc_error_addr(uintptr_t iobase)
{
    union {
        uint64_t lw;
        uint32_t w[2];
    } err_addr;

    err_addr.w[0] = mmcr32_rd(iobase + MEC_ESPI_PC_ERA_LSW_OFS);
    err_addr.w[1] = mmcr32_rd(iobase + MEC_ESPI_PC_ERA_MSW_OFS);

    return err_addr.lw;
}

void mec_hal_espi_pc_last_cycle(uintptr_t iobase, struct mec_espi_pc_last_cycle *lc)
{
    uint32_t v = 0;

    if (lc == NULL) {
        return;
    }

    lc->host_pc_addr_lsw = mmcr32_rd(iobase + MEC_ESPI_PC_LC_ADDR_LSW_OFS);
    lc->host_pc_addr_msw = mmcr32_rd(iobase + MEC_ESPI_PC_LC_ADDR_MSW_OFS);
    v = mmcr32_rd(iobase + MEC_ESPI_PC_LC_LTT_OFS);
    lc->len = (uint16_t)((v & MEC_ESPI_PC_LC_LTT_LEN_MSK) >> MEC_ESPI_PC_LC_LTT_LEN_POS);
    lc->cycle_type = (uint8_t)((v & MEC_ESPI_PC_LC_LTT_TYPE_MSK) >> MEC_ESPI_PC_LC_LTT_TYPE_POS);
    lc->tag = (uint8_t)((v & MEC_ESPI_PC_LC_LTT_TAG_MSK) >> MEC_ESPI_PC_LC_LTT_TAG_POS);
}

/* PC GIRQ */
void mec_hal_espi_pc_girq_ctrl(uint8_t enable)
{
    mec_hal_girq_ctrl(MEC_ESPI_PC_ECIA_INFO, (int)enable);
}

void mec_hal_espi_pc_girq_status_clr(void)
{
    mec_hal_girq_clr_src(MEC_ESPI_PC_ECIA_INFO);
}

uint32_t mec_hal_espi_pc_girq_status(void)
{
    return mec_hal_girq_src(MEC_ESPI_PC_ECIA_INFO);
}

uint32_t mec_hal_espi_pc_girq_result(void)
{
    return mec_hal_girq_result(MEC_ESPI_PC_ECIA_INFO);
}

/* Peripheral Channel LTR */

uint32_t mec_hal_espi_pc_ltr_status(uintptr_t regbase)
{
    return mmcr32_rd(regbase + MEC_ESPI_PC_LTR_SR_OFS);
}

void mec_hal_espi_pc_ltr_intr_en(uintptr_t regbase, uint32_t enmask)
{
    mmcr32_wr(enmask, regbase + MEC_ESPI_PC_LTR_IER_OFS);
}

void mec_hal_espi_pc_ltr_ctrl(uintptr_t regbase, uint8_t tag, uint8_t start)
{
    uint32_t rb = regbase + MEC_ESPI_PC_LTR_CR;
    uint32_t ctrl = MEC_ESPI_PC_LTR_CR_TAG_OUT_SET((uint32_t)tag);

    mmcr32_update_field(rb, ctrl, MEC_ESPI_PC_LTR_CR_TAG_OUT_MSK);

    if (start != 0) {
        mmcr32_set_bit(rb, MEC_ESPI_PC_LTR_CR_STA_POS);
    }
}

void mec_hal_espi_pc_ltr_msg(uintptr_t regbase, uint16_t nunits, uint8_t time_unit,
                             uint8_t rsvd_bits, uint8_t max_lat)
{
    uint32_t rb = regbase + MEC_ESPI_PC_LTR_MSG_OFS;
    uint32_t msg = (((uint32_t)nunits << MEC_ESPI_IO_PCLTRM_VALUE_Pos)
                    & MEC_ESPI_IO_PCLTRM_VALUE_Msk);

    msg = MEC_ESPI_PC_LTR_MSG_TV_SET((uint32_t)nunits);
    msg |= MEC_ESPI_PC_LTR_MSG_SC_SET((uint32_t)time_unit);
    msg |= MEC_ESPI_PC_LTR_MSG_RTXB_SET((uint32_t)rsvd_bits);

    if (max_lat != 0) {
        msg |= MEC_BIT(MEC_ESPI_PC_LTR_MSG_REQ_POS);
    }

    mmcr32_update_field(rb, msg, MEC_ESPI_PC_LTR_MSG_ALL_MSK);
}

void mec_hal_espi_pc_ltr_girq_ctrl(uint8_t enable)
{
    mec_hal_girq_ctrl(MEC_ESPI_LTR_ECIA_INFO, (int)enable);
}

void mec_hal_espi_pc_ltr_girq_status_clr(void)
{
    mec_hal_girq_clr_src(MEC_ESPI_LTR_ECIA_INFO);
}

uint32_t mec_hal_espi_pc_ltr_girq_status(void)
{
    return mec_hal_girq_src(MEC_ESPI_LTR_ECIA_INFO);
}

uint32_t mec_hal_espi_pc_ltr_girq_result(void)
{
    return mec_hal_girq_result(MEC_ESPI_LTR_ECIA_INFO);
}

/* end mec_espi_pc.c */
