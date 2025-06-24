/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include "mec_defs.h"
#include "mec_espi_vw.h"
#include "mec_retval.h"
#include "mec_mmcr.h"
#include "mec_espi_regs.h"

/* ---- eSPI Virtual Wire channel ----
 * The eSPI design implements virtual wires in two directions:
 * eSPI Host Controller to eSPI Target (EC)
 * eSPI Target (EC) to eSPI Host Controller.
 * Virtual wires are grouped into 4 VWires and transmitted in
 * packet marked as the VWire channel. The grouping of the VWires
 * is defined by the eSPI specification. The standard assigns a
 * host index number to each group.
 * MEC5 implements:
 *   MEC5_ESPI_NUM_CTVW eSPI Host Controller To Target VWire 96-bit registers.
 *   MEC5_ESPI_NUM_TCVW Target(EC) to eSPI Host Controller VWire 64-bit registers.
 *
 * CTVW hardware can generate an interrupt to the EC when any individual VWire
 * changes state.
 * TCVW hardware includes a transmit bit to send a group of VWires to the Host
 * and a bit indicating the packet has been transmitted. No interrupt can be
 * generated.
 *
 */

#define MEC_ESPI_VW_CHEN_CHG_GIRQ19_POS 8

static const uint8_t vw_ct_ien_xlat_tbl[] = {
    [MEC_ESPI_VW_CT_IEN_DIS] = 0x4u,
    [MEC_ESPI_VW_CT_IEN_LL] = 0u,
    [MEC_ESPI_VW_CT_IEN_LH] = 0x1u,
    [MEC_ESPI_VW_CT_IEN_RE] = 0xdu,
    [MEC_ESPI_VW_CT_IEN_FE] = 0xeu,
    [MEC_ESPI_VW_CT_IEN_BE] = 0xfu,
};
#define MEC_VW_CT_IXLAT_TBL_ENTRIES \
    (sizeof(vw_ct_ien_xlat_tbl) / sizeof(uint8_t))

static uint32_t xlat_isel(uint32_t logical_isel)
{
    uint32_t isel = 0x4u; /* default to disabled */

    if (logical_isel < MEC_VW_CT_IXLAT_TBL_ENTRIES) {
        isel = vw_ct_ien_xlat_tbl[logical_isel];
    }

    return isel;
}

/* ---- Public API ---- */

int mec_hal_espi_vw_is_enabled(uintptr_t iorb)
{
    if (iorb == 0) {
        return 0;
    }

    return mmcr8_test_bit(iorb + MEC_ESPI_VW_SR_OFS, MEC_ESPI_VW_SR_CHEN_POS);
}

/* VWire channel enable current state is in the VW Status register located in
 * the eSPI I/O component. VWire channel enable change does not have a bit
 * in the eSPI register spaces. Instead VW channel enable change signal is
 * connected to GIRQ19 bit[8].
 */
uint32_t mec_hal_espi_vw_en_status(uintptr_t iorb)
{
    uint32_t ensts = mmcr8_rd(iorb + MEC_ESPI_VW_SR_OFS) & MEC_BIT(MEC_ESPI_VW_SR_CHEN_POS);

    if (mmcr32_test_bit(MEC_ESPI_GIRQ_STS_ADDR, MEC_ESPI_GIRQ_VW_CHEN_POS) != 0) {
        ensts |= MEC_BIT(1);
    }

    return ensts;
}

void mec_hal_espi_vw_en_status_clr(void)
{
    mmcr32_wr(MEC_ESPI_GIRQ_STS_ADDR, MEC_BIT(MEC_ESPI_GIRQ_VW_CHEN_POS));
}

/* Returns non-zero if VW Enable Change interrupt is enabled and asserted else 0 */
uint32_t mec_hal_espi_vw_en_result(void)
{
    return (uint32_t)mmcr32_test_bit(MEC_ESPI_GIRQ_RESULT_ADDR, MEC_ESPI_GIRQ_VW_CHEN_POS);
}

void mec_hal_espi_vw_en_ien(uint8_t enable)
{
    if (enable != 0) {
        mmcr32_wr(MEC_ESPI_GIRQ_ENSET_ADDR, MEC_BIT(MEC_ESPI_GIRQ_VW_CHEN_POS));
    } else {
        mmcr32_wr(MEC_ESPI_GIRQ_ENCLR_ADDR, MEC_BIT(MEC_ESPI_GIRQ_VW_CHEN_POS));
    }
}

void mec_hal_espi_vw_ready_set(uintptr_t iorb)
{
    mmcr8_set_bit(iorb + MEC_ESPI_VW_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

int mec_hal_espi_vw_is_ready(uintptr_t iorb)
{
    return mmcr8_test_bit(iorb + MEC_ESPI_VW_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

/* Compute Controller-to-Target VWire register index given VW GIRQ bit
 * position and VW GIRQ bank (0/1).
 */
void mec_hal_espi_vw_ct_from_girq_pos(uint8_t bank, uint8_t girq_pos,
                                      uint8_t *ctidx, uint8_t *ctsrc)
{
    uint32_t d, m;

    if (girq_pos > 31u) {
        return;
    }

    if (ctidx != NULL) {
        d = girq_pos / 4u;
        if (bank) {
            d += 7u;
        }
        *ctidx = (uint8_t)d;
    }

    if (ctsrc != NULL) {
        m = girq_pos % 4u;
        *ctsrc = (uint8_t)m;
    }
}

/* GIRQ VW bank = 0
 *  CT_VW groups 0 - 6 (28 VWires)
 * bank = 1
 *  CT_VW groups 7 - 10 (16 VWires)
 */
uint32_t mec_hal_espi_girq_pos_to_ct_vw(uint8_t bank, uint8_t girq_pos)
{
    uint32_t idx_src = 0;

    if (girq_pos > 31u) {
        return UINT16_MAX;
    }

    idx_src = girq_pos / 4u;
    if (bank != 0) {
        idx_src += 7u;
    }

    idx_src &= 0xffu;

    idx_src |= ((uint32_t)girq_pos % 4u) << 8;

    return idx_src;
}

/* Controller(Host) to Target(EC) VWires can be configured to generate
 * an interrupt to the EC when they change. The CT VWire register do
 * not contain any interrupt status only interrupt enable. GIRQ.SOURCE
 * is the VWire interrupt change status.
 *
 * MEC5 conntect the CT VWire groups to the ECIA as follows:
 * GIRQ24 bits [0 - 3] CTVW00 src[0 - 3]
 * GIRQ24 bits [4 - 7] CTVW01 src[0 - 3]
 * ...
 * GIRQ24 bits [24 - 27] CTVW06 src[0 - 3]
 * GIRQ25 bits [0 - 3] CTVW07 src[0 - 3]
 * GIRQ25 bits [4 - 7] CTVW08 src[0 - 3]
 * ...
 * GIRQ25 bits [12 - 15] CTVW10 src[0 - 3]
 *
 * NOTE: ECIA GIRQ24 and GIRQ25 are both aggregated only.
 * Hardware does not provide direct NVIC connection of all the
 * GIRQ.SOURCE bits.
 */

static ALWAYS_INLINE uint32_t ct_vw_bitpos(uint8_t ct_idx, uint8_t src_idx)
{
    if (ct_idx >= MEC_CTVW_IDX07) {
        ct_idx -= MEC_CTVW_IDX07;
    }

    return (((uint32_t)ct_idx * 4u) + src_idx);
}

static ALWAYS_INLINE uint32_t ct_vw_base_addr(uint8_t ct_idx)
{
    uint32_t base = MEC_ESPI_GIRQ_VWB0_BASE;

    if (ct_idx >= MEC_CTVW_IDX07) {
        base = MEC_ESPI_GIRQ_VWB1_BASE;
    }

    return base;
}

int mec_hal_espi_vw_ct_girq_ctrl(uint8_t ct_idx, uint8_t src_idx, uint8_t enable)
{
    uint32_t raddr = 0, bitpos = 0;

    if ((ct_idx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    bitpos = ct_vw_bitpos(ct_idx, src_idx);
    raddr = ct_vw_base_addr(ct_idx);

    if (enable != 0) {
        raddr += MEC_ESPI_GIRQ_ENSET_OFS;
    } else {
        raddr += MEC_ESPI_GIRQ_ENCLR_OFS;
    }

    mmcr32_wr(raddr, MEC_BIT(bitpos));

    return MEC_RET_OK;
}

void mec_hal_espi_vw_ct_girq_ctrl_all(uint8_t enable)
{
    uintptr_t raddr0 = MEC_ESPI_GIRQ_VWB0_BASE;
    uintptr_t raddr1 = MEC_ESPI_GIRQ_VWB1_BASE;

    if (enable != 0) {
        raddr0 += MEC_ESPI_GIRQ_ENSET_OFS;
        raddr1 += MEC_ESPI_GIRQ_ENSET_OFS;
    } else {
        raddr0 += MEC_ESPI_GIRQ_ENCLR_OFS;
        raddr1 += MEC_ESPI_GIRQ_ENCLR_OFS;
    }

    mmcr32_wr(UINT32_MAX, raddr0);
    mmcr32_wr(UINT32_MAX, raddr1);
}

int mec_hal_espi_vw_ct_girq_clr(uint8_t ct_idx, uint8_t src_idx)
{
    uint32_t raddr = MEC_ESPI_GIRQ_STS_OFS;
    uint32_t bitpos = 0;

    if ((ct_idx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    bitpos = ct_vw_bitpos(ct_idx, src_idx);
    raddr += ct_vw_base_addr(ct_idx);

    mmcr32_wr(MEC_BIT(bitpos), raddr);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_girq_clr_msk(uint8_t ct_idx, uint8_t clr_msk)
{
    uint32_t raddr = MEC_ESPI_GIRQ_STS_OFS;
    uint32_t bitpos = 0;

    if (ct_idx > MEC_CTVW_IDX10) {
        return MEC_RET_ERR_INVAL;
    }

    bitpos = ct_vw_bitpos(ct_idx, 0);
    raddr += ct_vw_base_addr(ct_idx);

    mmcr32_wr(((uint32_t)clr_msk << bitpos), raddr);

    return MEC_RET_OK;
}

void mec_hal_espi_vw_ct_girq_clr_all(void)
{
    mmcr32_wr(UINT32_MAX, MEC_ESPI_GIRQ_VWB0_BASE + MEC_ESPI_GIRQ_STS_OFS);
    mmcr32_wr(UINT32_MAX, MEC_ESPI_GIRQ_VWB1_BASE + MEC_ESPI_GIRQ_STS_OFS);
}

uint32_t mec_hal_espi_vw_ct_girq_sts(uint8_t ct_idx, uint8_t src_idx)
{
    uint32_t raddr = MEC_ESPI_GIRQ_STS_OFS;
    uint32_t bitpos = 0, status = 0;

    if ((ct_idx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return 0;
    }

    bitpos = ct_vw_bitpos(ct_idx, src_idx);
    raddr += ct_vw_base_addr(ct_idx);

    status = mmcr32_rd(raddr);
    return ((status >> bitpos) & 0x1u);
}

uint32_t mec_hal_espi_vw_ct_girq_bank_result(uint8_t bank)
{
    uint32_t raddr = MEC_ESPI_GIRQ_RESULT_OFS;

    if (bank == MEC_ESPI_CTVW_IRQ_BANK_0) {
        raddr += MEC_ESPI_GIRQ_VWB0_BASE;
    } else {
        raddr += MEC_ESPI_GIRQ_VWB1_BASE;
    }

    return mmcr32_rd(raddr);
}

void mec_hal_espi_vw_ct_girq_bank_clr(uint8_t bank, uint32_t clrmsk)
{
    uint32_t raddr = MEC_ESPI_GIRQ_STS_OFS;

    if (bank == MEC_ESPI_CTVW_IRQ_BANK_0) {
        raddr += MEC_ESPI_GIRQ_VWB0_BASE;
    } else {
        raddr += MEC_ESPI_GIRQ_VWB1_BASE;
    }

    mmcr32_wr(clrmsk, raddr);
}

uint32_t mec_hal_espi_vw_ct_girq_res(uint8_t ct_idx, uint8_t src_idx)
{
    uint32_t raddr = MEC_ESPI_GIRQ_RESULT_OFS;
    uint32_t bitpos = 0, result = 0;

    if ((ct_idx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return 0;
    }

    bitpos = ct_vw_bitpos(ct_idx, src_idx);
    raddr += ct_vw_base_addr(ct_idx);

    result = mmcr32_rd(raddr);

    return ((result >> bitpos) & 0x1u);
}

uint32_t mec_hal_espi_vw_ct_group_girq_sts(uint8_t ct_idx)
{
    uint32_t raddr = MEC_ESPI_GIRQ_STS_OFS;
    uint32_t bitpos = 0, status = 0;

    if (ct_idx > MEC_CTVW_IDX10) {
        return 0;
    }

    bitpos = ct_vw_bitpos(ct_idx, 0);
    raddr += ct_vw_base_addr(ct_idx);

    status = mmcr32_rd(raddr);

    return ((status >> bitpos) & 0xfu);
}

void mec_hal_espi_vw_ct_group_girq_sts_clr(uint8_t ct_idx)
{
    uintptr_t raddr = MEC_ESPI_GIRQ_STS_OFS;
    uint32_t bitpos = 0;

    if (ct_idx >MEC_CTVW_IDX10) {
        return;
    }

    bitpos = ct_vw_bitpos(ct_idx, 0);
    raddr += ct_vw_base_addr(ct_idx); 

    mmcr32_wr((0xfu << bitpos), raddr);
}

uint32_t mec_hal_espi_vw_ct_group_girq_res(uint8_t ct_idx)
{
    uintptr_t raddr = MEC_ESPI_GIRQ_RESULT_OFS;
    uint32_t bitpos = 0, result = 0;

    if (ct_idx > MEC_CTVW_IDX10) {
        return 0;
    }

    bitpos = ct_vw_bitpos(ct_idx, 0);
    raddr += ct_vw_base_addr(ct_idx); 

    result = mmcr32_rd(raddr);

    return ((result >> bitpos) & 0xfu);
}

int mec_hal_espi_vw_ct_group_girq_ctrl(uint8_t ct_idx, uint8_t src_msk, uint8_t enable)
{
    uint32_t raddr = 0;
    uint32_t bitpos = 0, regval = 0;

    if (ct_idx > MEC_CTVW_IDX10) {
        return MEC_RET_ERR_INVAL;
    }

    bitpos = ct_vw_bitpos(ct_idx, 0);
    raddr = ct_vw_base_addr(ct_idx); 

    if (enable != 0) {
        raddr += MEC_ESPI_GIRQ_ENSET_OFS;
    } else {
        raddr += MEC_ESPI_GIRQ_ENCLR_OFS;
    }

    regval = (uint32_t)(src_msk & 0xfu) << bitpos;

    mmcr32_wr(regval, raddr);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_irq_sel_set(uintptr_t vwrb, uint8_t vw_idx, uint8_t src_idx, uint8_t irq_sel)
{
    uint32_t reg_ofs = 0, rval = 0;
    uint32_t msk = 0, pos = 0;

    if ((vw_idx > MEC_ESPI_CTVW10_REG_IDX) || (src_idx > 3)
        || (irq_sel >= MEC_VW_CT_IXLAT_TBL_ENTRIES)) {
        return MEC_RET_ERR_INVAL;
    }

    /* Word 1 contains interrupt select fields for each VWire in the group */
    reg_ofs = MEC_ESPI_VW_HT_GRPW(vw_idx, 1);
    pos = MEC_ESPI_VW_H2T_W1_ISEL_POS(src_idx);
    msk = MEC_ESPI_VW_H2T_W1_ISEL_MSK(src_idx);
    rval = mmcr32_rd(vwrb + reg_ofs) & (uint32_t)~msk;
    rval |= (((uint32_t)irq_sel << pos) & msk); 
    mmcr32_wr(rval, vwrb + reg_ofs);

    return MEC_RET_OK;
}

/* Set all four VWires IRQ select fields in the CT VW group at vw_idx.
 * irq_sels b[7:0] = Source 0, ..., b[31:24] = Source 3 IRQ select.
 */
int mec_hal_espi_vw_ct_irq_sel_set_all(uintptr_t vwrb, uint8_t vw_idx, uint32_t irq_sels)
{
    uint32_t reg_ofs = 0, rval = 0, temp = 0;

    if (vw_idx > MEC_ESPI_CTVW10_REG_IDX) {
        return MEC_RET_ERR_INVAL;
    }

    /* Word 1 contains interrupt select fields for each VWire in the group */
    reg_ofs = MEC_ESPI_VW_HT_GRPW(vw_idx, 1);
    rval = mmcr32_rd(vwrb + reg_ofs) & 0xf0f0f0f0u;

    for (size_t n = 0; n < 4; n++) {
        temp = irq_sels & MEC_ESPI_VW_H2T_V1_ISEL_MSK0;

        if (temp >= MEC_VW_CT_IXLAT_TBL_ENTRIES) {
            return MEC_RET_ERR_INVAL;
        }

        temp = vw_ct_ien_xlat_tbl[temp];
        rval |= (temp << (n * 8u));
    }

    mmcr32_wr(rval, vwrb + reg_ofs);

    return MEC_RET_OK;
}

/* Configure MEC5 Controller-to-Target(EC) Virtual Wire group register.
 * All fields are set by RESET_SYS (chip reset) except the four VWire Source value
 * fields. VWire Source value fields are reset by the reset source field. If we
 * are requested to modify source values we must switch their reset source to
 * RESET_SYS, modify, then switch to requested reset source or restore the
 * original reset source.
 */
static void mec_hal_espi_vwg_ct_config(uintptr_t vwrb, uint8_t ctidx,
                                       struct mec_espi_vw_config *cfg, uint32_t flags)
{
    uint32_t r[3] = {0};
    uint32_t reg_ofs = 0, v = 0;
    unsigned int i = 0;

    if (flags == 0) { /* do not modify anything */
        return;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);

    r[0] = mmcr32_rd(vwrb + reg_ofs);
    r[1] = mmcr32_rd(vwrb + reg_ofs + 4u);
    r[2] = mmcr32_rd(vwrb + reg_ofs + 8u);

    if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_HI_POS)) != 0) {
        v = MEC_ESPI_VW_W0_HI_SET((uint32_t)cfg->host_idx);
        r[0] &= (uint32_t)~MEC_ESPI_VW_W0_HI_MSK;
        r[0] |= v;
    }

    if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_RST_SRC_POS)) != 0) {
        v = MEC_ESPI_VW_W0_RSRC_SET((uint32_t)cfg->reset_src);
        r[0] &= (uint32_t)~MEC_ESPI_VW_W0_RSRC_MSK;
        r[0] |= v;
    }

    for (i = 0; i < 4; i++) {
        if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_RST_VAL_POS + i)) != 0) {
            if ((cfg->reset_val_bm & MEC_BIT(i)) != 0) {
                r[0] |= MEC_BIT(i + MEC_ESPI_VW_W0_RSTATE_POS);
            } else {
                r[0] &= (uint32_t)~MEC_BIT(i + MEC_ESPI_VW_W0_RSTATE_POS);
            }

            if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_IRQ_POS + i)) != 0) {
                uint32_t msk = 0xfu << i;
                uint8_t j = cfg->src_irq_sel[i];

                if (j >= MEC_VW_CT_IXLAT_TBL_ENTRIES) {
                    j = MEC_ESPI_VW_CT_IEN_DIS;
                }

                r[1] = ((r[1] & ~msk) | ((uint32_t)vw_ct_ien_xlat_tbl[j] << (i * 8)));
            }

            if (cfg->src_val_bm & MEC_BIT(i)) {
                r[2] |= MEC_BIT(i * 8);
            } else {
                r[2] &= ~MEC_BIT(i * 8);
            }
        }
    }

    mmcr32_wr(r[2], vwrb + reg_ofs + 8u);
    mmcr32_wr(r[1], vwrb + reg_ofs + 4u);
    mmcr32_wr(r[0], vwrb + reg_ofs);
}

int mec_hal_espi_vw_ct_host_index_set(uintptr_t vwrb, uint8_t ctidx, uint8_t host_index)
{
    uint32_t reg_ofs = 0, v = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);
    v = mmcr32_rd(vwrb + reg_ofs) & (uint32_t)~MEC_ESPI_VW_W0_HI_MSK;
    v |= (host_index & MEC_ESPI_VW_W0_HI_MSK);
    mmcr32_wr(v, vwrb + reg_ofs);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_reset_source_get(uintptr_t vwrb, uint8_t ctidx, uint8_t *reset_source)
{
    uint32_t reg_ofs = 0, v = 0;

    if ((vwrb == 0) || (reset_source == NULL) || (ctidx > MEC_CTVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);
    v = mmcr32_rd(vwrb + reg_ofs);
    v = MEC_ESPI_VW_W0_RSRC_GET(v);
    *reset_source = (uint8_t)(v & 0xffu);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_reset_source_set(uintptr_t vwrb, uint8_t ctidx, uint8_t reset_source)
{
    uint32_t reg_ofs = 0, v = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);
    v = MEC_ESPI_VW_W0_RSRC_SET((uint32_t)reset_source);
    mmcr32_update_field(vwrb + reg_ofs, v, MEC_ESPI_VW_W0_RSRC_MSK);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_reset_state_set(uintptr_t vwrb, uint8_t ctidx, uint8_t src_idx,
                                       uint8_t reset_state)
{
    uint32_t reg_ofs = 0, v = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);
    v = mmcr32_rd(vwrb + reg_ofs);

    if (reset_state != 0) {
        v |= MEC_BIT(src_idx + MEC_ESPI_VW_W0_RSTATE_POS);
    } else {
        v &= (uint32_t)~MEC_BIT(src_idx + MEC_ESPI_VW_W0_RSTATE_POS);
    }

    mmcr32_wr(v, vwrb + reg_ofs);

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_irqsel_set(uintptr_t vwrb, uint8_t ctidx,
                                  uint8_t src_idx, uint8_t irq_sel)
{
    uint32_t reg_ofs = 0, v = 0, msk = 0, isel = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10) || (src_idx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 1u);
    isel = xlat_isel(irq_sel);
    msk = MEC_ESPI_VW_H2T_W1_ISEL_MSK((uint32_t)src_idx);
    v = MEC_ESPI_VW_H2T_W1_ISEL_SET((uint32_t)src_idx, isel);
    mmcr32_update_field(vwrb + reg_ofs, v, msk);

    return MEC_RET_OK;
}

/* Configure specified fields of a Controller-to-Target VWire in a group.
 * ctidx specifies the zero based index into the hardware array of 80-bit
 * C2T VWire registers. Each 80-bit register controls 4 VWires.
 * Properties affecting the whole group of 4 VWires:
 *      Host Index and Reset Source
 * Properities of the single VWire specified by src_idx:
 *      Reset State and IRQ Select
 * All of these fields are reset on POR or chip reset only.
 * NOTE: Reset States are loaded into SRC bits on the de-asserting
 * edge of Reset Source.
 */
static void mec_hal_espi_vw_ct_config(uintptr_t vwrb, uint8_t ctidx, uint8_t src_idx,
                                      uint8_t host_index, uint32_t config)
{
    uint32_t reg_ofs = 0, v = 0, temp = 0;

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 0);

    v = mmcr32_rd(vwrb + reg_ofs);

    v &= (uint32_t)~MEC_ESPI_VW_W0_HI_MSK;
    v |= MEC_ESPI_VW_W0_HI_SET((uint32_t)host_index);
    
    if ((config & MEC_BIT(MEC_ESPI_VW_CFG_RSTSRC_DO_POS)) != 0) {
        temp = (((uint32_t)config & MEC_ESPI_VW_CFG_RSTSRC_MSK) >> MEC_ESPI_VW_CFG_RSTSRC_POS);
        v &= (uint32_t)~MEC_ESPI_VW_W0_RSRC_MSK;
        v |= MEC_ESPI_VW_W0_RSRC_SET(temp);
    }
    if ((config & MEC_BIT(MEC_ESPI_VW_CFG_RSTVAL_DO_POS)) != 0) {
        temp = (((uint32_t)config & MEC_ESPI_VW_CFG_RSTVAL_MSK) >> MEC_ESPI_VW_CFG_RSTVAL_POS);
        v &= (uint32_t)~MEC_ESPI_VW_W0_RSTATE_MSK;
        v |= MEC_ESPI_VW_W0_RSTATE_SET(temp);
    }

    mmcr32_wr(v, vwrb + reg_ofs);

    if ((config & MEC_BIT(MEC_ESPI_VW_CFG_IRQSEL_DO_POS)) != 0) {
        v = ((config >> MEC_ESPI_VW_CFG_IRQSEL_POS) & MEC_ESPI_VW_CFG_IRQSEL_MSK0);
        v = xlat_isel(v);
        v = MEC_ESPI_VW_H2T_W1_ISEL_SET((uint32_t)src_idx, v);
        temp = MEC_ESPI_VW_H2T_W1_ISEL_MSK((uint32_t)src_idx);

        mmcr32_update_field(vwrb + reg_ofs + 4u, v, temp);
    }
}


int mec_hal_espi_vw_ct_wire_set(uintptr_t vwrb, uint8_t ctidx, uint8_t widx, uint8_t val)
{
    uint32_t reg_ofs = 0, bit_pos = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 2);
    bit_pos = MEC_ESPI_VW_STATE_POS(widx);

    if (val != 0) {
        mmcr32_set_bit(vwrb + reg_ofs, bit_pos);
    } else {
        mmcr32_clr_bit(vwrb + reg_ofs, bit_pos);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_vw_ct_wire_get(uintptr_t vwrb, uint8_t ctidx, uint8_t widx, uint8_t *val)
{
    uint32_t reg_ofs = 0, bit_pos = 0;

    if ((vwrb == 0) || (val == NULL) || (ctidx > MEC_CTVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 2);
    bit_pos = MEC_ESPI_VW_STATE_POS(widx);

    if (mmcr32_test_bit(vwrb + reg_ofs, bit_pos) != 0) {
        *val = 1u;
    } else {
        *val = 0u;
    }

    return MEC_RET_OK;
}

/* Set 4 VWires in group from b[3:0] of val if in mask */
int mec_hal_espi_vw_ct_group_set(uintptr_t vwrb, uint8_t ctidx, uint8_t val, uint8_t msk)
{
    uint32_t reg_ofs = 0, chgmsk = 0, v = 0;

    if ((vwrb == 0) || (ctidx > MEC_CTVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    if (msk == 0) {
        return MEC_RET_OK;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 2);

    for (unsigned int i = 0; i < 4u; i++) {
        if (msk & MEC_BIT(i)) {
            chgmsk |= MEC_BIT(i * 8);
            if (val & MEC_BIT(i)) {
                v |= MEC_BIT(i * 8);
            }
        }
    }

    mmcr32_update_field(vwrb + reg_ofs, v, chgmsk);

    return MEC_RET_OK;
}

static uint8_t vw_group_get(uint32_t vw_states)
{
    uint32_t temp = 0u;

    for (unsigned int i = 0; i < 4u; i++) {
        if (vw_states & MEC_BIT(i * 8u)) {
            temp |= MEC_BIT(i);
        }
    }

    return (uint8_t)(temp & 0xffu);
}

/* Copy 4 VWires in group to b[3:0] of byte pointed to by val */
int mec_hal_espi_vw_ct_group_get(uintptr_t vwrb, uint8_t ctidx, uint8_t *val)
{
    uint32_t reg_ofs = 0;

    if ((reg_ofs == 0) || (val == NULL) || (ctidx > MEC_CTVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_HT_GRPW(ctidx, 2);

    *val = vw_group_get(mmcr32_rd(vwrb + reg_ofs));

    return MEC_RET_OK;
}

/* Configure MEC5 Target(EC)-to-Controller(Host) Virtual Wire group register.
 * Similar to CT VWire group registers, the TC SRC bits are reset by the
 * reset source specified in the TC VWire reset source field. If the reset
 * source is active we will not be able to change the SRC bits. If SRC bits
 * are to be changed, temporarily change reset source to RESET_SYS and
 * restore/set to new reset source at the end of this routine.
 * NOTE: Target-to-Controller VW groups have no EC interrupt capability.
 * The struct mec_espi_vw_config src_irq_sec[] member is ignored.
 */
static void mec_hal_espi_vwg_tc_config(uintptr_t vwrb, uint8_t tcidx,
                                       struct mec_espi_vw_config *cfg, uint32_t flags)
{
    uint32_t reg_ofs = 0;
    uint32_t r[2];
    unsigned int i;

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);

    r[0] = mmcr32_rd(vwrb + reg_ofs);
    r[1] = mmcr32_rd(vwrb + reg_ofs + 4u);

    if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_HI_POS)) != 0) {
        r[0] &= (uint32_t)~MEC_ESPI_VW_W0_HI_MSK;
        r[0] |= MEC_ESPI_VW_W0_HI_SET((uint32_t)cfg->host_idx);
    }

    if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_RST_SRC_POS)) != 0) {
        r[0] &= (uint32_t)~MEC_ESPI_VW_W0_RSRC_MSK;
        r[0] |= MEC_ESPI_VW_W0_RSRC_SET((uint32_t)cfg->reset_src);
    }

    for (i = 0; i < 4; i++) {
        if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_RST_VAL_POS + i)) != 0) {
            if ((cfg->reset_val_bm & MEC_BIT(i)) != 0) {
                r[0] |= MEC_BIT(i + MEC_ESPI_VW_W0_RSTATE_POS);
            } else {
                r[0] &= (uint32_t)~MEC_BIT(i + MEC_ESPI_VW_W0_RSTATE_POS);
            }
        }

        if ((flags & MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_VAL_POS + i)) != 0) {
            if ((cfg->src_val_bm & MEC_BIT(i)) != 0) {
                r[1] |= MEC_BIT(i * 8);
            } else {
                r[1] &= (uint32_t)~MEC_BIT(i * 8);
            }
        }
    }

    mmcr32_wr(r[0], vwrb + reg_ofs);
    mmcr32_wr(r[1], vwrb + reg_ofs + 4u);
}

/* Configure specified fields of a Target-to-Controller VWire in a group.
 * tcidx specifies the zero based index into the hardware array of 64-bit
 * T2C VWire registers. Each 64-bit register controls 4 VWires.
 * Properties affecting the whole group of 4 VWires:
 *      Host Index and Reset Source
 * Properities of the single VWire specified by src_idx:
 *      Reset State
 * All of these fields are reset on POR or chip reset only.
 * NOTE: Reset States are loaded into SRC bits on the de-asserting
 * edge of Reset Source.
 * NOTE2: Target-to-Controller VWires do not generate interrupts to the EC.
 */
static void mec_hal_espi_vw_tc_config(uintptr_t vwrb, uint8_t tcidx, uint8_t src_idx,
                                      uint8_t host_index, uint32_t config)
{
    uint32_t reg_ofs = 0, rv = 0, cv = 0;

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);
    rv = mmcr32_rd(vwrb + reg_ofs);

    rv &= (uint32_t)~MEC_ESPI_VW_W0_HI_MSK;
    rv |= MEC_ESPI_VW_W0_HI_SET((uint32_t)host_index);

    if ((config & MEC_BIT(MEC_ESPI_VW_CFG_RSTSRC_DO_POS)) != 0) {
        cv = (((uint32_t)config & MEC_ESPI_VW_CFG_RSTSRC_MSK) >> MEC_ESPI_VW_CFG_RSTSRC_POS);
        rv &= (uint32_t)~MEC_ESPI_VW_W0_RSRC_MSK;
        rv |= MEC_ESPI_VW_W0_RSRC_SET(cv);
    }

    if ((config & MEC_BIT(MEC_ESPI_VW_CFG_RSTVAL_DO_POS)) != 0) {
        cv = (((uint32_t)config & MEC_ESPI_VW_CFG_RSTVAL_MSK) >> MEC_ESPI_VW_CFG_RSTVAL_POS);
        rv &= (uint32_t)~MEC_ESPI_VW_W0_RSTATE_MSK;
        rv |= (cv << (MEC_ESPI_VW_W0_RSTATE_POS + src_idx));
    }

    mmcr32_wr(rv, vwrb + reg_ofs);
}

int mec_hal_espi_vw_tc_wire_set(uintptr_t vwrb, uint8_t tcidx, uint8_t widx, uint8_t val,
                                uint32_t flags)
{
    uint32_t reg_ofs = 0, bitpos = 0;

    if ((vwrb == 0) || (tcidx > MEC_TCVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);
    bitpos = widx * 8u;

    if (val != 0) {
        mmcr32_set_bit(vwrb + reg_ofs + 4u, bitpos);
    } else {
        mmcr32_clr_bit(vwrb + reg_ofs + 4u, bitpos);
    }

    if ((flags & MEC_BIT(MEC_ESPI_VW_FLAG_WAIT_TC_TX_POS)) != 0) {
        while (mmcr32_rd(vwrb + reg_ofs) & (0xfu << MEC_ESPI_VW_T2H_W0_CHG0_POS)) {
                ;
        }
    }

    return MEC_RET_OK;
}

int mec_hal_espi_vw_tc_wire_set_cs(uintptr_t vwrb, uint8_t tcidx, uint8_t widx, uint8_t val,
                                   const struct mec_espi_vw_poll *vwp)
{
    uint32_t reg_ofs = 0, bitpos = 0, delay_loops = 0;

    if ((vwrb == 0) || (tcidx > MEC_TCVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);
    bitpos = widx * 8u;

    if (val != 0) {
        mmcr32_set_bit(vwrb + reg_ofs + 4u, bitpos);
    } else {
        mmcr32_clr_bit(vwrb + reg_ofs + 4u, bitpos);
    }

    if (vwp && vwp->delayfp) {
        delay_loops = vwp->nloops;
        while (mmcr32_rd(vwrb + reg_ofs) & (0xfu << MEC_ESPI_VW_T2H_W0_CHG0_POS)) {
            if (delay_loops == 0) {
                return MEC_RET_ERR_TIMEOUT;
            }

            vwp->delayfp(vwp->delay_param);
            delay_loops--;
        }
    }

    return MEC_RET_OK;
}

int mec_hal_espi_vw_tc_wire_get(uintptr_t vwrb, uint8_t tcidx, uint8_t widx, uint8_t *val)
{
    uint32_t reg_ofs = 0;

    if ((vwrb == 0) || (val == NULL) || (tcidx > MEC_TCVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 1);
    *val =(uint8_t)((mmcr32_rd(vwrb + reg_ofs) >> (widx * 8u)) & MEC_BIT(0));

    return MEC_RET_OK;
}

/* Obtains both the VWire state and its read-only change status.
 * Change status is 1 when FW has changed the VWire state. A change bit == 1 causes
 * the target eSPI to set the VWires Available status bit. If Alert mode is enabled
 * an eSPI in-band or pin alert is asserted to the eSPI Host.  The eSPI Host will
 * issue a GET_STATUS. The Host sees the VWires Available bit and issues a GET_VWIRES
 * command. When the eSPI Target responds to GET_VWIRES it clears all read-only
 * Target-to-Controller VWire change status bits.
 * val bit[0] = state of VWire
 * val bit[7] = Change bit. 1 = VWire state was changed by FW and the Host has not
 * read it. 0 = VWire has not changed or Host has read the current value.
 */
int mec_hal_espi_vw_tc_wire_cs_get(uintptr_t vwrb, uint8_t tcidx, uint8_t widx, uint8_t *val)
{
    uint32_t reg_ofs = 0, bitpos = 0, v = 0;
    uint8_t vw_sts = 0;

    if ((vwrb == 0) || (val == NULL) || (tcidx > MEC_TCVW_IDX10) || (widx > 3)) {
        return MEC_RET_ERR_INVAL;
    }

    bitpos = MEC_ESPI_VW_STATE_POS(widx);
    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);
    v = mmcr32_rd(vwrb + reg_ofs + 4u);
    vw_sts = (uint8_t)((v >> bitpos) & MEC_BIT(0));

    bitpos = MEC_ESPI_VW_T2H_W0_CHG_POS(widx);
    if (mmcr32_test_bit(vwrb + reg_ofs, bitpos) != 0) {
        vw_sts |= MEC_BIT(7);
    }
    
    *val = vw_sts;

    return MEC_RET_OK;
}

/* Set 4 VWires in group from b[3:0] of val if in mask */
int mec_hal_espi_vw_tc_group_set(uintptr_t vwrb, uint8_t tcidx, uint8_t val, uint8_t msk,
                                 uint32_t flags)
{
    uint32_t reg_ofs = 0, chgmsk = 0, rval = 0;

    if ((vwrb == 0) || (tcidx > MEC_TCVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    if (msk == 0) {
        return MEC_RET_OK;
    }

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 0);

    for (unsigned int i = 0; i < 4u; i++) {
        if (msk & MEC_BIT(i)) {
            chgmsk |= MEC_BIT(i * 8);
            if (val & MEC_BIT(i)) {
                rval |= MEC_BIT(i * 8);
            }
        }
    }

    mmcr32_update_field(vwrb + reg_ofs + 4u, rval, chgmsk);

    chgmsk = 0xfu << MEC_ESPI_VW_T2H_W0_CHG0_POS;
    if ((flags & MEC_BIT(MEC_ESPI_VW_FLAG_WAIT_TC_TX_POS)) != 0) {
        while ((mmcr32_rd(vwrb + reg_ofs) & chgmsk) != 0) {
                ;
        }
    }

    return MEC_RET_OK;
}

/* Copy 4 VWires in TC group to b[3:0] of byte pointed to by val */
int mec_hal_espi_vw_tc_group_get(uintptr_t vwrb, uint8_t tcidx, uint8_t *val)
{
    uint32_t reg_ofs = 0;

    if ((vwrb == 0) || (val == NULL) || (tcidx > MEC_TCVW_IDX10)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_VW_TH_GRPW(tcidx, 1);

    *val = vw_group_get(mmcr32_rd(vwrb + reg_ofs));

    return MEC_RET_OK;
}

int mec_hal_espi_vwg_config(uintptr_t vwrb, uint8_t vwidx, struct mec_espi_vw_config *cfg,
                            uint32_t flags)
{
    if ((vwrb == 0) || (cfg == NULL) || (vwidx >= MEC_ESPI_VW_MAX_REG_IDX)) {
        return MEC_RET_ERR_INVAL;
    }

    if (vwidx < MEC_ESPI_TCVW00_REG_IDX) {
        mec_hal_espi_vwg_ct_config(vwrb, vwidx, cfg, flags);
    } else {
        vwidx -= MEC_ESPI_TCVW00_REG_IDX;
        mec_hal_espi_vwg_tc_config(vwrb, vwidx, cfg, flags);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_vwire_config(uintptr_t vwrb, uint8_t vwidx, uint8_t src_idx,
                              uint8_t host_index, uint32_t config)
{
    if ((vwrb == 0) || (vwidx >= MEC_ESPI_VW_MAX_REG_IDX) || (src_idx > 3u)) {
        return MEC_RET_ERR_INVAL;
    }

    if (vwidx < MEC_ESPI_TCVW00_REG_IDX) {
        mec_hal_espi_vw_ct_config(vwrb, vwidx, src_idx, host_index, config);
    } else {
        vwidx -= MEC_ESPI_TCVW00_REG_IDX;
        mec_hal_espi_vw_tc_config(vwrb, vwidx, src_idx, host_index, config);
    }

    return MEC_RET_OK;
}

/* Get value of a VWire specified by MEC5 logical register index and source position in
 * the HW register.
 */
int mec_hal_espi_vw_get_src(uintptr_t vwrb, struct mec_espi_vw *vw,
                            uint32_t flags __attribute__((__unused__)))
{
    int ret = 0;
    uint8_t regidx = 0;

    if (vw == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    regidx = vw->vwidx;
    if (regidx < MEC_ESPI_TCVW00_REG_IDX) {
        ret = mec_hal_espi_vw_ct_wire_get(vwrb, regidx, vw->srcidx, &vw->val);
    } else {
        regidx -= MEC_ESPI_TCVW00_REG_IDX;
        ret = mec_hal_espi_vw_tc_wire_get(vwrb, regidx, vw->srcidx, &vw->val);
    }

    return ret;
}

/* Set value of VWire in a group
 * vwbase is base address of MEC5 VWire registers
 * vwidx is a zero based index of the MEC5 VWire register containing the VWire
 * src is the zero based VWire position in the group of 4. [0:3]
 * val is the new VWire value non-zero means 1 else 0.
 * flags: bit[0]=1 instructs this routine to wait forever for Target to
 * Controller (upstream) VWire value to be transmitted. NOTE: a packet is only
 * transmitted if the VWire value was changed.
 */
int mec_hal_espi_vw_set_src(uintptr_t vwrb, struct mec_espi_vw *vw, uint32_t flags)
{
    int ret = 0;
    uint8_t regidx = 0;

    if (vw == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    regidx = vw->vwidx;
    if (regidx < MEC_ESPI_TCVW00_REG_IDX) {
        ret = mec_hal_espi_vw_ct_wire_set(vwrb, regidx, vw->srcidx, vw->val);
    } else {
        regidx -= MEC_ESPI_TCVW00_REG_IDX;
        ret = mec_hal_espi_vw_tc_wire_set(vwrb, regidx, vw->srcidx, vw->val, flags);
    }

    return ret;
}

int mec_hal_espi_vw_set_src_cs(uintptr_t vwrb, struct mec_espi_vw *vw,
                               const struct mec_espi_vw_poll *vwp)
{
    int ret = 0;
    uint8_t regidx = 0;

    if (vw == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    regidx = vw->vwidx;
    if (regidx < MEC_ESPI_TCVW00_REG_IDX) {
        ret = mec_hal_espi_vw_ct_wire_set(vwrb, regidx, vw->srcidx, vw->val);
    } else {
        regidx -= MEC_ESPI_TCVW00_REG_IDX;
        ret = mec_hal_espi_vw_tc_wire_set_cs(vwrb, regidx, vw->srcidx, vw->val, vwp);
    }

    return ret;
}

/* Get VWire group source bits specified by struct mec_espi_vw.vwidx and
 * store in struct mec_espi_vw.val
 */
int mec_hal_espi_vw_get_src_group(uintptr_t vwrb, struct mec_espi_vw *vw,
                                  uint32_t flags __attribute__((__unused__)))
{
    int ret = 0;
    uint8_t regidx = 0;

    if (vw == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    regidx = vw->vwidx;
    if (regidx < MEC_ESPI_TCVW00_REG_IDX) {
        ret = mec_hal_espi_vw_tc_group_get(vwrb, regidx, &vw->val);
    } else {
        regidx -= MEC_ESPI_TCVW00_REG_IDX;
        ret = mec_hal_espi_vw_tc_group_get(vwrb, regidx, &vw->val);
    }

    return ret;
}

int mec_hal_espi_vw_set_src_group(uintptr_t vwrb, struct mec_espi_vw *vw, uint32_t flags)
{
    int ret = 0;
    uint8_t regidx = 0;

    if ((vw == NULL) || (vw->vwidx >= MEC_ESPI_VW_MAX_REG_IDX)) {
        return MEC_RET_ERR_INVAL;
    }

    if (vw->msk == 0) {
        return MEC_RET_OK;
    }

    regidx = vw->vwidx;
    if (regidx < MEC_ESPI_TCVW00_REG_IDX) {
        ret = mec_hal_espi_vw_ct_group_set(vwrb, regidx, vw->val, vw->msk);
    } else {
        regidx -= (uint8_t)MEC_ESPI_TCVW00_REG_IDX;
        ret = mec_hal_espi_vw_tc_group_set(vwrb, regidx, vw->val, vw->msk, flags);
    }

    return ret;
}

/* ---- API using eSPI Host Index and VWire src [0:3] for access ---- */
static int lookup_ct_vw_by_host_index(uintptr_t vwrb, uint8_t host_index)
{
    uint32_t reg_ofs = 0, v = 0;

    if (vwrb != 0) {
        for (uint32_t i = 0; i < MEC_CTVW_IDX10; i++) {
            reg_ofs = MEC_ESPI_VW_HT_GRPW(i, 0);
            v = mmcr32_rd(vwrb + reg_ofs);

            if ((uint8_t)(v & 0xffu) == host_index) {
                return i;
            }
        }
    }

    return -1;
}

static int lookup_tc_vw_by_host_index(uintptr_t vwrb, uint8_t host_index)
{
    uint32_t reg_ofs = 0, v = 0;

    if (vwrb != 0) {
        for (uint32_t i = 0; i < MEC_TCVW_IDX10; i++) {
            reg_ofs = MEC_ESPI_VW_TH_GRPW(i, 0);
            v = mmcr32_rd(vwrb + reg_ofs);
            
            if ((uint8_t)(v & 0xffu) == host_index) {
                return i;
            }
        }
    }

    return -1;
}

/* Read the state of the VWire given by its Host Index and source (bit)
 * position in the 4 wire group.
 */
int mec_hal_espi_vw_get(uintptr_t vwrb, uint8_t host_index, uint8_t src_id, uint8_t *val)
{
    int ret = MEC_RET_ERR_INVAL;
    int idx = lookup_ct_vw_by_host_index(vwrb, host_index);

    if (idx >= 0) {
        ret = mec_hal_espi_vw_ct_wire_get(vwrb, (uint8_t)idx & 0x7fu, src_id, val);
    }

    idx = lookup_tc_vw_by_host_index(vwrb, host_index);
    if (idx >= 0) {
        ret = mec_hal_espi_vw_tc_wire_get(vwrb, (uint8_t)idx & 0x7fu, src_id, val);
    }

    return ret;
}

int mec_hal_espi_vw_set(uintptr_t vwrb, uint8_t host_index, uint8_t src_id, uint8_t val,
                        uint32_t flags)
{
    int ret = MEC_RET_ERR_INVAL;
    int idx = lookup_ct_vw_by_host_index(vwrb, host_index);

    if (idx >= 0) {
        ret = mec_hal_espi_vw_ct_wire_set(vwrb, (uint8_t)idx & 0x7fu, src_id, val);
    }

    idx = lookup_tc_vw_by_host_index(vwrb, host_index);
    if (idx >= 0) {
        ret = mec_hal_espi_vw_tc_wire_set(vwrb, (uint8_t)idx & 0x7fu, src_id, val, flags);
    }

    return ret;
}

int mec_hal_espi_vw_set_cs(uintptr_t vwrb, uint8_t host_index, uint8_t src_id, uint8_t val,
                           const struct mec_espi_vw_poll *vwp)
{
    int ret = MEC_RET_ERR_INVAL;
    int idx = lookup_ct_vw_by_host_index(vwrb, host_index);

    if (idx >= 0) {
        ret = mec_hal_espi_vw_ct_wire_set(vwrb, (uint8_t)idx & 0x7fu, src_id, val);
    }

    idx = lookup_tc_vw_by_host_index(vwrb, host_index);
    if (idx >= 0) {
        ret = mec_hal_espi_vw_tc_wire_set_cs(vwrb, (uint8_t)idx & 0x7fu, src_id, val, vwp);
    }

    return ret;
}

/* VWire's are grouped into 4 VWires per host index. Read the states of the VWires
 * and pack them into bits[3:0] of the byte pointed to by groupval.
 */
int mec_hal_espi_vw_get_group(uintptr_t vwrb, uint8_t host_index, uint8_t *groupval)
{
    int ret = MEC_RET_ERR_INVAL;
    int idx = lookup_ct_vw_by_host_index(vwrb, host_index);

    if (idx >= 0) {
        ret = mec_hal_espi_vw_ct_group_get(vwrb, (uint8_t)idx & 0x7fu, groupval);
    }

    idx = lookup_tc_vw_by_host_index(vwrb, host_index);
    if (idx >= 0) {
        ret = mec_hal_espi_vw_tc_group_get(vwrb, (uint8_t)idx & 0x7fu, groupval);
    }

    return ret;
}

/* Each VWire group indentified by its Host Index implements 4 virtual wires.
 * Set the values of the VWires based upon the passed groupmsk. If groupmsk
 * b[3:0] is set then set that VWire's state to the corresponding bit in
 * groupval.
 */
int mec_hal_espi_vw_set_group(uintptr_t vwrb, uint8_t host_index, uint8_t groupval,
                              uint8_t groupmsk, uint32_t flags)
{
    int ret = MEC_RET_ERR_INVAL;
    int idx = lookup_ct_vw_by_host_index(vwrb, host_index);

    if (idx >= 0) {
        ret = mec_hal_espi_vw_ct_group_set(vwrb, (uint8_t)idx & 0x7fu, groupval, groupmsk);
    }

    idx = lookup_tc_vw_by_host_index(vwrb, host_index);
    if (idx >= 0) {
        ret = mec_hal_espi_vw_tc_group_set(vwrb, (uint8_t)idx & 0x7fu, groupval, groupmsk, flags);
    }

    return ret;
}

/* end mec_espi_vw.c */
