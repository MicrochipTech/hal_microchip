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
#include "mec_espi_api.h"
#include "mec_retval.h"

/* ---- eSPI Peripheral Channel ---- */
#define MEC_ESPI_PC_ECIA_INFO MEC5_ECIA_INFO(19, 0, 11, 103)
#define MEC_ESPI_LTR_ECIA_INFO MEC5_ECIA_INFO(19, 3, 11, 106)

static uint32_t xlat_intr_to_hw(uint32_t bitmap)
{
    uint32_t hwbm = 0;

    if (bitmap & BIT(MEC_ESPI_PC_INTR_BERR_POS)) {
        hwbm |= BIT(MEC_ESPI_PC_INTR_BERR_POS);
    }
    if (bitmap & BIT(ESPI_IO_PCSTS_PCEN_CHG_Pos)) {
        hwbm |= BIT(MEC_ESPI_PC_INTR_CHEN_CHG_POS);
    }
    if (bitmap & BIT(ESPI_IO_PCSTS_PCBMEN_CHG_Pos)) {
        hwbm |= BIT(MEC_ESPI_PC_INTR_BMEN_CHG_POS);
    }

    return hwbm;
}

/* ---- Public API ---- */

void mec_espi_pc_ready_set(struct espi_io_regs *iobase)
{
    iobase->PCRDY = BIT(ESPI_IO_PCRDY_PC_READY_Pos);
}

int mec_espi_pc_is_ready(struct espi_io_regs *iobase)
{
    if (iobase->PCRDY & BIT(ESPI_IO_PCRDY_PC_READY_Pos)) {
        return 1u;
    }

    return 0;
}

/* Peripheral channel enable state is status register bit[24]; move to bit[0].
 * Enable change is status register bit[25]; move to bit[1].
 */
uint32_t mec_espi_pc_en_status(struct espi_io_regs *iobase)
{
    uint32_t temp = iobase->PCSTS & (ESPI_IO_PCSTS_PCEN_VAL_Msk | ESPI_IO_PCSTS_PCEN_CHG_Msk);

    return (temp >> ESPI_IO_PCSTS_PCEN_VAL_Pos);
}

/* Peripheral channel bus master enable state is status register bit[27]; move to bit[0].
 * BM enable change is bit[28]; move to bit[1].
 * the enable's current state.
 */
uint32_t mec_espi_pc_bm_status(struct espi_io_regs *iobase)
{
    uint32_t temp = iobase->PCSTS & (ESPI_IO_PCSTS_PCBM_VAL_Msk | ESPI_IO_PCSTS_PCBMEN_CHG_Msk);

    return (temp >> ESPI_IO_PCSTS_PCBM_VAL_Pos);
}

uint32_t mec_espi_pc_status(struct espi_io_regs *iobase)
{
    uint32_t sts = 0;
    uint32_t temp = iobase->PCSTS;

    if (temp & BIT(ESPI_IO_PCSTS_EC_BUS_ERR_Pos)) {
        sts |= BIT(MEC_ESPI_PC_ISTS_BERR_POS);
    }
    if (temp & BIT(ESPI_IO_PCSTS_PCEN_VAL_Pos)) {
        sts |= BIT(MEC_ESPI_PC_ISTS_CHEN_CHG_POS);
    }
    if (temp & BIT(ESPI_IO_PCSTS_PCEN_CHG_Pos)) {
        sts |= BIT(MEC_ESPI_PC_ISTS_CHEN_STATE_POS);
    }
    if (temp & BIT(ESPI_IO_PCSTS_PCBM_VAL_Pos)) {
        sts |= BIT(MEC_ESPI_PC_ISTS_BMEN_STATE_POS);
    }
    if (temp & BIT(ESPI_IO_PCSTS_PCBMEN_CHG_Pos)) {
        sts |= BIT(MEC_ESPI_PC_ISTS_BMEN_CHG_POS);
    }

    return sts;
}

void mec_espi_pc_status_clr(struct espi_io_regs *iobase, uint32_t bitmap)
{
    uint32_t regval = 0;

    regval = xlat_intr_to_hw(bitmap);
    iobase->PCSTS = regval;
}

void mec_espi_pc_intr_en(struct espi_io_regs *iobase, uint32_t bitmap)
{
    uint32_t regval = 0;

    regval = xlat_intr_to_hw(bitmap);
    iobase->PCIEN |= regval;
}

void mec_espi_pc_intr_dis(struct espi_io_regs *iobase, uint32_t bitmap)
{
    uint32_t regval = 0;

    regval = xlat_intr_to_hw(bitmap);
    iobase->PCIEN &= ~regval;
}

void mec_espi_pc_status_clr_all(struct espi_io_regs *iobase)
{
    iobase->PCSTS = (ESPI_IO_PCSTS_EC_BUS_ERR_Msk
                     | ESPI_IO_PCSTS_PCEN_CHG_Msk
                     | ESPI_IO_PCSTS_PCBMEN_CHG_Msk);
}

uint64_t mec_espi_pc_error_addr(struct espi_io_regs *iobase)
{
    union {
        uint64_t lw;
        uint32_t w[2];
    } err_addr;

    err_addr.w[0] = iobase->PCERR[0];
    err_addr.w[1] = iobase->PCERR[1];

    return err_addr.lw;
}

void mec_espi_pc_last_cycle(struct espi_io_regs *iobase,
                            struct mec_espi_pc_last_cycle *lc)
{
    uint32_t temp;

    lc->host_pc_addr_lsw = iobase->PCLC[0];
    lc->host_pc_addr_msw = iobase->PCLC[1];
    temp = iobase->PCLC[2];
    lc->len = (uint16_t)(temp & 0xfffu);
    lc->cycle_type = (uint8_t)((temp >> 12) & 0xffu);
    lc->tag = (uint8_t)((temp >> 20) & 0xfu);
}

/* PC GIRQ */
void mec_espi_pc_girq_ctrl(uint8_t enable)
{
    mec_girq_ctrl(MEC_ESPI_PC_ECIA_INFO, (int)enable);
}

void mec_espi_pc_girq_status_clr(void)
{
    mec_girq_clr_src(MEC_ESPI_PC_ECIA_INFO);
}

uint32_t mec_espi_pc_girq_status(void)
{
    return mec_girq_src(MEC_ESPI_PC_ECIA_INFO);
}

uint32_t mec_espi_pc_girq_result(void)
{
    return mec_girq_result(MEC_ESPI_PC_ECIA_INFO);
}

/* Peripheral Channel LTR */

uint32_t mec_espi_pc_ltr_status(struct espi_io_regs *iobase)
{
    return iobase->PCLTRSTS;
}

void mec_espi_pc_ltr_intr_en(struct espi_io_regs *iobase, uint32_t enmask)
{
    iobase->PCLTREN = enmask;
}

void mec_espi_pc_ltr_ctrl(struct espi_io_regs *iobase, uint8_t tag, uint8_t start)
{
    uint32_t ctrl = iobase->PCLTRCTL & ~(ESPI_IO_PCLTRCTL_LTR_TX_TAG_Msk);

    ctrl |= (((uint32_t)tag << ESPI_IO_PCLTRCTL_LTR_TX_TAG_Pos)
             & ESPI_IO_PCLTRCTL_LTR_TX_TAG_Msk);

    iobase->PCLTRCTL = ctrl;

    if (start) {
        iobase->PCLTRCTL |= BIT(ESPI_IO_PCLTRCTL_START_Pos);
    }
}

void mec_espi_pc_ltr_msg(struct espi_io_regs *iobase, uint16_t nunits, uint8_t time_unit,
                         uint8_t rsvd_bits, uint8_t max_lat)
{
    uint32_t msg = ((uint32_t)nunits << ESPI_IO_PCLTRM_VALUE_Pos) & ESPI_IO_PCLTRM_VALUE_Msk;

    msg |= (((uint32_t)time_unit << ESPI_IO_PCLTRM_SCALE_Pos) & ESPI_IO_PCLTRM_SCALE_Msk);
    msg |= (((uint32_t)rsvd_bits << ESPI_IO_PCLTRM_TX_RSVD_BITS_Pos)
            & ESPI_IO_PCLTRM_TX_RSVD_BITS_Msk);
    if (max_lat) {
        msg |= BIT(ESPI_IO_PCLTRM_SEL_MAX_LAT_Pos);
    }

    iobase->PCLTRM = (iobase->PCLTRM & ~(0xffffu)) | msg;
}

void mec_espi_pc_ltr_girq_ctrl(uint8_t enable)
{
    mec_girq_ctrl(MEC_ESPI_LTR_ECIA_INFO, (int)enable);
}

void mec_espi_pc_ltr_girq_status_clr(void)
{
    mec_girq_clr_src(MEC_ESPI_LTR_ECIA_INFO);
}

uint32_t mec_espi_pc_ltr_girq_status(void)
{
    return mec_girq_src(MEC_ESPI_LTR_ECIA_INFO);
}

uint32_t mec_espi_pc_ltr_girq_result(void)
{
    return mec_girq_result(MEC_ESPI_LTR_ECIA_INFO);
}

/* end mec_espi_pc.c */