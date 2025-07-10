/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include <mec_espi_regs.h>
#include "mec_defs.h"
#include "mec_ecia_api.h"
#include "mec_espi_api.h"
#include "mec_retval.h"
#include "mec_mmcr.h"

/* ---- eSPI Flash Channel ----
 * eSPI target controller in the EC can access the flash device(s) attached
 * to the Host eSPI controller.
 */
#define MEC_ESPI_FC_ECIA_INFO MEC_ECIA_INFO(19, 6, 11, 109)

/* Flash erase request size must be non-zero.
 * Actual erase size is chosen by eSPI Host.
 * Specification recommands a value of 1.
 */
#define MEC_ESPI_FC_ERASE_SIZE 1u

/* Flash channel status errors */
#define MEC_ESPI_FC_ERR_ALL \
    (MEC_BIT(MEC_ESPI_IO_FCSTS_BAD_REQ_Pos) | MEC_BIT(MEC_ESPI_IO_FCSTS_START_OVRFL_Pos) \
     | MEC_BIT(MEC_ESPI_IO_FCSTS_FAIL_Pos) | MEC_BIT(MEC_ESPI_IO_FCSTS_DATA_INCOMPL_Pos) \
     | MEC_BIT(MEC_ESPI_IO_FCSTS_DATA_OVRUN_Pos) | MEC_BIT(MEC_ESPI_IO_FCSTS_ABORT_FW_Pos) \
     | MEC_BIT(MEC_ESPI_IO_FCSTS_EC_BUS_ERR_Pos) | MEC_BIT(MEC_ESPI_IO_FCSTS_DIS_BY_HOST_Pos))

/* ---- Public API ---- */

void mec_hal_espi_fc_girq_ctrl(uint8_t enable)
{
    mec_hal_girq_ctrl(MEC_ESPI_FC_ECIA_INFO, (int)enable);
}

void mec_hal_espi_fc_girq_status_clr(void)
{
    mec_hal_girq_clr_src(MEC_ESPI_FC_ECIA_INFO);
}

uint32_t mec_hal_espi_fc_girq_status(void)
{
    return mec_hal_girq_src(MEC_ESPI_FC_ECIA_INFO);
}

uint32_t mec_hal_espi_fc_girq_result(void)
{
    return mec_hal_girq_result(MEC_ESPI_FC_ECIA_INFO);
}

/* Flash channel HW bits for channel enable state and enable change match API definition:
 * bit[0] = enable state
 * bit[1] = enable state changed
 */
uint32_t mec_hal_espi_fc_en_status(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_FC_SR_OFS);

    return (v & (MEC_BIT(MEC_ESPI_FC_SR_CHEN_STATE_POS) | MEC_BIT(MEC_ESPI_FC_SR_CHEN_CHG_POS)));
}

void mec_hal_espi_fc_ready_set(uintptr_t iorb)
{
    mmcr8_set_bit(iorb + MEC_ESPI_FC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

int mec_hal_espi_fc_is_ready(uintptr_t iorb)
{
    return mmcr8_test_bit(iorb + MEC_ESPI_FC_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

int mec_hal_espi_fc_is_busy(uintptr_t iorb)
{
    return mmcr32_test_bit(iorb + MEC_ESPI_FC_CFG_OFS, MEC_ESPI_FC_CFG_BUSY_POS);
}

/* Start previoulsy configured Flash channel operation.
 * 1. Clear R/W1C status except for Flash channel enable change as this
 *    is asynchronous and must be handled separately. Preferably by
 *    by an ISR.
 * 2. Enable or disable FC transfer done interrupt per passed flag
 * 3. Start transaction.
 */
void mec_hal_espi_fc_op_start(uintptr_t iorb, uint32_t flags)
{
    uint32_t v = MEC_ESPI_FC_SR_ERR_ALL_MSK | MEC_BIT(MEC_ESPI_FC_SR_DONE_POS);

    mmcr32_wr(v, iorb + MEC_ESPI_FC_SR_OFS); /* clear done and error status */    

    if ((flags & MEC_BIT(MEC_ESPI_FC_XFR_FLAG_START_IEN_POS)) != 0) {
        mmcr32_set_bit(iorb + MEC_ESPI_FC_IER_OFS, MEC_ESPI_FC_IER_DONE_POS);
    } else {
        mmcr32_clr_bit(iorb + MEC_ESPI_FC_IER_OFS, MEC_ESPI_FC_IER_DONE_POS);
    }

    mmcr32_set_bit(iorb + MEC_ESPI_FC_CR_OFS, MEC_ESPI_FC_CR_START_POS);
}

void mec_hal_espi_fc_op_abort(uintptr_t iorb)
{
    mmcr32_set_bit(iorb + MEC_ESPI_FC_CR_OFS, MEC_ESPI_FC_CR_ABORT_POS);
}

void mec_hal_espi_fc_intr_ctrl(uintptr_t iorb, uint32_t msk, uint8_t en)
{
    uint32_t r = 0;

    if (iorb == 0) {
        return;
    }

    if (msk & MEC_BIT(MEC_ESPI_FC_INTR_DONE_POS)) {
        r |= MEC_BIT(MEC_ESPI_FC_IER_DONE_POS);
    }

    if (msk & MEC_BIT(MEC_ESPI_FC_INTR_CHEN_CHG_POS)) {
        r |= MEC_BIT(MEC_ESPI_FC_IER_CHG_EN_POS);
    }

    if (en != 0) {
        mmcr32_set_bits(iorb + MEC_ESPI_FC_IER_OFS, r);
    } else {
        mmcr32_clr_bits(iorb + MEC_ESPI_FC_IER_OFS, r);
    }
}

uint32_t mec_hal_espi_fc_status(uintptr_t iorb)
{
    if (iorb == 0) {
        return 0;
    }

    return mmcr32_rd(iorb + MEC_ESPI_FC_SR_OFS);
}

void mec_hal_espi_fc_status_clr(uintptr_t iorb, uint32_t msk)
{
    if (iorb == 0) {
        return;
    }

    mmcr32_wr(msk, iorb + MEC_ESPI_FC_SR_OFS);
    mec_hal_girq_clr_src(MEC_ESPI_FC_ECIA_INFO);
}

int mec_hal_espi_fc_is_error(uint32_t fc_status)
{
    if (fc_status & (MEC_ESPI_FC_SR_ERR_ALL_MSK)) {
        return 1;
    }

    return 0;
}

/* Return flash channel maximum read size selected by eSPI Host */
uint32_t mec_hal_espi_fc_max_read_req_sz(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_FC_CFG_OFS);
    uint32_t exp = MEC_ESPI_FC_CFG_MRDR_GET(v);
    
    if (exp == 0u) {
        return 0u;
    }

    exp += 5u; /* power of 2 */

    return MEC_BIT(exp);
}

/* Return flash channel maximum payload size selected by eSPI Host */
uint32_t mec_hal_espi_fc_max_pld_sz(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_FC_CFG_OFS);
    uint32_t exp = MEC_ESPI_FC_CFG_MPLD_GET(v);

    if ((exp == 0u) || (exp > 3u)) { /* reserved values */
        return 0u;
    }

    exp += 5u; /* power of 2 */

    return MEC_BIT(exp);
}

uint32_t mec_hal_espi_fc_max_taf_read_req_sz(uintptr_t iorb)
{
    uint32_t v = mmcr8_rd(iorb + MEC_ESPI_CAP_FC_OFS);
    uint32_t exp = MEC_ESPI_CAP_FC_TAF_MRRQ_GET(v);

    if (exp == 0) {
        return 0;
    }

    exp += 5u; /* power of 2 */
    
    return MEC_BIT(exp);
}

/* Return the two allowed erase block sizes in b[15:0] and b[31:16] in units
 * of KB. If only one erase size allowed both fields will be identical.
 * A return value of 0 indicates the flash channel has not been properly
 * configured during eSPI link negoitation.
 */
uint32_t mec_hal_espi_fc_get_erase_sz(uintptr_t iorb)
{
    uint32_t v = 0;
    uint16_t erb1 = 0, erb2 = 0;
    uint8_t ersz_encoding = 0;

    if (iorb == 0) {
        return 0;
    }

    v = mmcr32_rd(iorb + MEC_ESPI_FC_CFG_OFS);
    ersz_encoding = MEC_ESPI_FC_CFG_EBSZ_GET(v);

    switch (ersz_encoding) {
    case MEC_ESPI_FC_CFG_EBSZ_4K_VAL:
        erb1 = 4u;
        erb2 = erb1;
        break;
    case MEC_ESPI_FC_CFG_EBSZ_64K_VAL:
        erb1 = 64u;
        erb2 = erb1;
        break;
    case MEC_ESPI_FC_CFG_EBSZ_4K_64K_VAL:
        erb1 = 4u;
        erb2 = 64u;
        break;
    case MEC_ESPI_FC_CFG_EBSZ_128K_VAL:
        erb1 = 128u;
        erb2 = erb1;
        break;
    case MEC_ESPI_FC_CFG_EBSZ_256K_VAL:
        erb1 = 256u;
        erb2 = erb1;
        break;
    default:
        erb1 = 0;
        erb2 = 0;
        break;
    }

    return (uint32_t)erb1 + ((uint32_t)erb2 << 16);
}

int mec_hal_espi_fc_check_erase_sz(uintptr_t iorb, uint32_t ersz_bytes)
{
    uint32_t ersz = mec_hal_espi_fc_get_erase_sz(iorb);
    uint32_t er1 = (ersz & 0xffffu) * 1024u;
    uint32_t er2 = ((ersz >> 16) & 0xffffu) * 1024u;

    if ((ersz_bytes == er1) || (ersz_bytes == er2)) {
        return MEC_RET_OK;
    }

    return MEC_RET_ERR_DATA_LEN;
}

/* Start transmit of a flash request to the eSPI Host Controller.
 * Operations are: read, erase, or write. The caller can select
 * from two erase operations: Ask Host to erase smaller or larger
 * of two address ranges if the Host has a choice. This is based
 * upon flash devices connected to Host chipset and its policies.
 * The specs recommend using a value of 1 for erase size in the
 * request packet sent to the Host.
 * Read and write operations data lengths must be non-zero. The
 * FC hardware will break up requests into chunks of maximum payload
 * size. FC hardware will signal done or error when the last request
 * is done or there was an error on any packet in the transaction.
 */
int mec_hal_espi_fc_xfr_start(uintptr_t iorb, struct mec_espi_fc_xfr *pxfr, uint32_t flags)
{
    uint32_t xfr_len = 0, fc_cr = 0;

    if ((iorb == 0) || (pxfr == NULL) || (pxfr->byte_len == 0) || (pxfr->buf_addr == 0)
        || (pxfr->operation >= MEC_ESPI_FC_OP_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    if (!MEC_IS_PTR_ALIGNED32(pxfr->buf_addr)) {
        return MEC_RET_ERR_DATA_ALIGN;
    }

    if (mec_hal_espi_fc_is_ready(iorb)) {
        return MEC_RET_ERR_HW_NOT_INIT;
    }

    if (mec_hal_espi_fc_is_busy(iorb)) {
        return MEC_RET_ERR_BUSY;
    }

    switch (pxfr->operation) {
    case MEC_ESPI_FC_OP_ERASE_L:
        fc_cr = MEC_ESPI_FC_CR_FUNC_ERASE_LG;
        xfr_len = MEC_ESPI_FC_ERASE_SIZE;
        break;
    case MEC_ESPI_FC_OP_ERASE_S:
        fc_cr = MEC_ESPI_FC_CR_FUNC_ERASE_SM;
        xfr_len = MEC_ESPI_FC_ERASE_SIZE;
        break;
    case MEC_ESPI_FC_OP_WRITE:
        fc_cr = MEC_ESPI_FC_CR_FUNC_WRITE;
        xfr_len = pxfr->byte_len;
        break;
    default:
        fc_cr = MEC_ESPI_FC_CR_FUNC_READ;
        xfr_len = pxfr->byte_len;
        break;
    }

    mmcr32_clr_bit(iorb + MEC_ESPI_FC_IER_OFS, MEC_ESPI_FC_IER_DONE_POS);
    mmcr32_set_bits(iorb + MEC_ESPI_FC_SR_OFS, MEC_ESPI_FC_SR_ERR_ALL_MSK |
                    MEC_BIT(MEC_ESPI_FC_SR_DONE_POS));
    mmcr32_wr(pxfr->flash_addr, iorb + MEC_ESPI_FC_FA_OFS);
    mmcr32_wr(pxfr->buf_addr, iorb + MEC_ESPI_FC_BA_OFS);
    mmcr32_wr(xfr_len, iorb + MEC_ESPI_FC_LEN_OFS);

    if (flags & MEC_BIT(MEC_ESPI_FC_XFR_FLAG_START_IEN_POS)) {
        mmcr32_set_bit(iorb + MEC_ESPI_FC_IER_OFS, MEC_ESPI_FC_IER_DONE_POS);
    }

    fc_cr |= MEC_ESPI_FC_CR_TAG_SET((uint32_t)pxfr->tag);
    fc_cr |= MEC_BIT(MEC_ESPI_FC_CR_START_POS);

    mmcr32_wr(fc_cr, iorb + MEC_ESPI_FC_CR_OFS);

    return MEC_RET_OK;
}

/* end mec_espi_fc.c */
