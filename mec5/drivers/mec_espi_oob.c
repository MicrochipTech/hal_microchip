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
#include "mec_espi_oob.h"
#include "mec_pcr_api.h"
#include "mec_retval.h"
#include "mec_mmcr.h"

/* ---- eSPI Out-Of-Band (OOB) Channel ---- */
#define MEC5_ESPI_GIRQ               19
#define MEC5_ESPI_GIRQ_AGGR_NVIC     11
#define MEC5_ESPI_OOB_UP_GIRQ_POS    4
#define MEC5_ESPI_OOB_UP_NVIC_DIRECT 107
#define MEC5_ESPI_OOB_DN_GIRQ_POS    5
#define MEC5_ESPI_OOB_DN_NVIC_DIRECT 108

#define MEC_ESPI_OOB_UP_ECIA_INFO MEC_ECIA_INFO(MEC5_ESPI_GIRQ, MEC5_ESPI_OOB_UP_GIRQ_POS, \
                                                MEC5_ESPI_GIRQ_AGGR_NVIC, \
                                                MEC5_ESPI_OOB_UP_NVIC_DIRECT)
#define MEC_ESPI_OOB_DN_ECIA_INFO MEC_ECIA_INFO(MEC5_ESPI_GIRQ, MEC5_ESPI_OOB_DN_GIRQ_POS, \
                                                MEC5_ESPI_GIRQ_AGGR_NVIC, \
                                                MEC5_ESPI_OOB_DN_NVIC_DIRECT)

#define MEC_ESPI_OOB_TX_STS_RW1C \
    (MEC_ESPI_IO_OOBTXSTS_DONE_Msk | MEC_ESPI_IO_OOBTXSTS_CHEN_CHG_Msk \
     | MEC_ESPI_IO_OOBTXSTS_EC_BUS_ERR_Msk | MEC_ESPI_IO_OOBTXSTS_START_OVRUN_Msk \
     | MEC_ESPI_IO_OOBTXSTS_BAD_REQ_Msk)

#define MEC_ESPI_OOB_RX_STS_RW1C \
    (MEC_ESPI_IO_OOBRXSTS_DONE_Msk | MEC_ESPI_IO_OOBRXSTS_EC_BUS_ERR_Msk \
     | MEC_ESPI_IO_OOBRXSTS_DATA_OVRUN_Msk)


static uint32_t msk_to_girq_bitmap(uint8_t msk)
{
    uint32_t bitmap = 0;

    if (msk & MEC_ESPI_OOB_DIR_UP) {
        bitmap |= MEC_BIT(MEC5_ESPI_OOB_UP_GIRQ_POS);
    }
    if (msk & MEC_ESPI_OOB_DIR_DN) {
        bitmap |= MEC_BIT(MEC5_ESPI_OOB_DN_GIRQ_POS);
    }

    return bitmap;
}

static uint32_t bitmap_to_msk(uint32_t bitmap)
{
    uint32_t msk = 0;

    if (bitmap & MEC_BIT(MEC5_ESPI_OOB_UP_GIRQ_POS)) {
        msk |= MEC_ESPI_OOB_DIR_UP;
    }
    if (bitmap & MEC_BIT(MEC5_ESPI_OOB_DN_GIRQ_POS)) {
        msk |= MEC_ESPI_OOB_DIR_DN;
    }

    return msk;
}

/* ---- Public API ---- */

void mec_hal_espi_oob_girq_ctrl(uint8_t enable, uint8_t msk)
{
    uint32_t bitmap = msk_to_girq_bitmap(msk);

    mec_hal_girq_bm_en(MEC5_ESPI_GIRQ, bitmap, enable);
}

void mec_hal_espi_oob_girq_status_clr(uint8_t msk)
{
    uint32_t bitmap = msk_to_girq_bitmap(msk);

    mec_hal_girq_bm_clr_src(MEC5_ESPI_GIRQ, bitmap);
}

uint32_t mec_hal_espi_oob_girq_status(void)
{
    uint32_t src = mec_hal_girq_source_get(MEC5_ESPI_GIRQ);

    return bitmap_to_msk(src);
}

uint32_t mec_hal_espi_oob_girq_result(void)
{
    uint32_t result = mec_hal_girq_result_get(MEC5_ESPI_GIRQ);

    return bitmap_to_msk(result);
}

void mec_hal_espi_oob_ready_set(uintptr_t iorb)
{
    mmcr8_set_bit(iorb + MEC_ESPI_OOB_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

int mec_hal_espi_oob_is_ready(uintptr_t iorb)
{
    return mmcr8_test_bit(iorb + MEC_ESPI_OOB_RDY_OFS, MEC_ESPI_CHAN_RDY_POS);
}

/* OOB TX Status register channel enable change bit matches API defined
 * position (bit[1]). Current channel enable state is at bit[9] in the
 * register; move to bit[0].
 */
uint32_t mec_hal_espi_oob_en_status(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_OOB_TX_SR_OFS);
    uint32_t status = 0;

    if ((v & MEC_BIT(MEC_ESPI_OOB_TX_SR_CHEN_POS)) != 0) {
        status |= MEC_BIT(0);
    }

    if ((v & MEC_BIT(MEC_ESPI_OOB_TX_SR_CENC_POS)) != 0) {
        status |= MEC_BIT(1);
    }

    return status;
}

/* Return the maximum eSPI OOB packet size in bytes selected by the eSPI Host when
 * it configured the OOB channel.
 * OOB is different than the other channels. Size encoding are the
 * same but OOB adds 9 bytes to the standard sizes.
 */
uint32_t mec_hal_espi_oob_max_pkt_size(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_OOB_RX_CR_OFS);
    uint32_t sz = MEC_ESPI_OOB_MPLD_SZ_GET(v);

    if ((sz == 0) || (sz > 3u)) { /* reserved values */
        return 0;
    }

    sz += 5u;
    sz = MEC_BIT(sz) + MEC_ESPI_OOB_ADDED_SIZE;

    return sz;
}

/* Set OOB buffer address for upstream or downstream direction.
 * We set the buffer size to the passed value if it is <= OOB Configuration Max Packet Size.
 * Otherwise we set size to Max Packet Size.
 * For upstream, if the buffer size is 0 or > the Max Packet Size in the OOB Configuration
 * register then HW when started will generate a Bad Request error.
 * For downstream, if the specified buffer length >= OOB Config Max Packet Size the
 * HW limit check is disabled. This means no Data Overrun check will be made on
 * incoming data allowing potential buffer overrun corrupting memory.
 */
int mec_hal_espi_oob_buffer_set(uintptr_t iorb, uint8_t dir,
                                struct mec_espi_oob_buf * buf)
{
    uint32_t v = 0, lenb = 0, maxlen = 0;

    if ((iorb == 0) || (buf == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    if (!MEC_IS_PTR_ALIGNED32(buf->maddr)) {
        return MEC_RET_ERR_DATA_ALIGN;
    }

    if (buf->len == 0) {
        return MEC_RET_ERR_DATA_LEN;
    }

    maxlen = mec_hal_espi_oob_max_pkt_size(iorb);
    lenb = buf->len;
    if (lenb > maxlen) {
        lenb = maxlen;
    }

    if (dir == MEC_ESPI_OOB_DIR_DN) {
        mmcr32_wr(buf->maddr, iorb + MEC_ESPI_OOB_RX_BA_OFS);
        v = MEC_ESPI_OOB_RXL_BLEN_SET(lenb);
        mmcr32_update_field(iorb + MEC_ESPI_OOB_RXL_OFS, v, MEC_ESPI_OOB_RXL_BLEN_MSK);
    } else {
        mmcr32_wr(buf->maddr, iorb + MEC_ESPI_OOB_TX_BA_OFS);
        v = MEC_ESPI_OOB_TXL_MLEN_SET(lenb);
        mmcr32_update_field(iorb + MEC_ESPI_OOB_TXL_OFS, v, MEC_ESPI_OOB_TXL_MLEN_MSK);
    }

    return MEC_RET_OK;
}

/* Inform the hardware we are ready to receive OOB packets from the upstream eSPI Host.
 * The RX buffer must be set before calling this routine.
 * NOTE: RX available bit is write-only. Once set it causes the OOB receive enable to be
 * set in the OOB RX Status register.
 */
void mec_hal_espi_oob_rx_buffer_avail(uintptr_t iorb)
{
    mmcr32_set_bit(iorb + MEC_ESPI_OOB_RX_CR_OFS, MEC_ESPI_OOB_RX_CR_SRA_POS);
}

/* Enable OOB channel interrupts to the EC
 * NOTE: All of the OOB errors cause DONE status to be set.
 */
void mec_hal_espi_oob_intr_ctrl(uintptr_t iorb, uint32_t msk, uint8_t en)
{
    uint32_t tx_ien_msk = 0;

    if (msk & MEC_BIT(MEC_ESPI_OOB_UP_INTR_DONE_POS)) {
        tx_ien_msk |= MEC_BIT(MEC_ESPI_OOB_TX_IER_DONE_POS);
    }

    if (msk & MEC_BIT(MEC_ESPI_OOB_UP_INTR_CHEN_CHG_POS)) {
        tx_ien_msk |= MEC_BIT(MEC_ESPI_OOB_TX_IER_CENC_POS);
    }

    if (tx_ien_msk != 0) {
        if (en != 0) {
            mmcr32_set_bits(iorb + MEC_ESPI_OOB_TX_IER_OFS, tx_ien_msk);
        } else {
            mmcr32_clr_bits(iorb + MEC_ESPI_OOB_TX_IER_OFS, tx_ien_msk);
        }
    }

    if (msk & MEC_BIT(MEC_ESPI_OOB_DN_INTR_DONE_POS)) {
        if (en) {
            mmcr32_set_bit(iorb + MEC_ESPI_OOB_RX_IER_OFS, MEC_ESPI_OOB_RX_IER_DONE_POS);
        } else {
            mmcr32_clr_bit(iorb + MEC_ESPI_OOB_RX_IER_OFS, MEC_ESPI_OOB_RX_IER_DONE_POS);
        }
    }
}

void mec_hal_espi_oob_tx_start(uintptr_t iorb, uint8_t tag, uint8_t start)
{
    uint32_t v = MEC_ESPI_OOB_TX_TAG_SET(tag);

    mmcr32_update_field(iorb + MEC_ESPI_OOB_TX_CR_OFS, v, MEC_ESPI_OOB_TX_TAG_MSK);

    if (start != 0) {
        mmcr32_set_bit(iorb + MEC_ESPI_OOB_TX_CR_OFS, MEC_ESPI_OOB_TX_CR_START_POS);
    }
}

int mec_hal_espi_oob_tx_is_busy(uintptr_t iorb)
{
    return mmcr32_test_bit(iorb + MEC_ESPI_OOB_TX_SR_OFS, MEC_ESPI_OOB_TX_SR_DONE_POS);
}

/* Get TAG value we received in last OOB RX message */
uint8_t mec_hal_espi_oob_rx_tag(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_OOB_RX_SR_OFS);
    uint32_t tag = MEC_ESPI_OOB_TX_TAG_GET(v);

    return (uint8_t)tag;
}

/* Get length of last received OOB RX message */
uint32_t mec_hal_espi_oob_received_len(uintptr_t iorb)
{
    uint32_t v = mmcr32_rd(iorb + MEC_ESPI_OOB_RXL_OFS);

    return (uint32_t)MEC_ESPI_OOB_RXL_MLEN_GET(v);
}

/* Get raw register status of specified OOB direction */
uint32_t mec_hal_espi_oob_status(uintptr_t iorb, uint8_t dir)
{
    if (dir == MEC_ESPI_OOB_DIR_DN) {
        return mmcr32_rd(iorb + MEC_ESPI_OOB_RX_SR_OFS);
    } else {
        return mmcr32_rd(iorb + MEC_ESPI_OOB_TX_SR_OFS);
    }
}

int mec_hal_espi_oob_is_done(uint32_t status, uint8_t dir)
{
    uint8_t done_pos = MEC_ESPI_OOB_TX_SR_DONE_POS;

    if (dir == MEC_ESPI_OOB_DIR_DN) {
        done_pos = MEC_ESPI_OOB_RX_SR_DONE_POS;
    }

    if ((status & MEC_BIT(done_pos)) != 0) {
        return 1;
    }

    return 0;
}

int mec_hal_espi_oob_is_error(uint32_t status, uint8_t dir)
{
    uint32_t msk = MEC_ESPI_OOB_TX_SR_ALL_ERR_MSK;

    if (dir == MEC_ESPI_OOB_DIR_DN) {
        msk = MEC_ESPI_OOB_RX_SR_ALL_ERR_MSK;
    }

    if (status & msk) {
        return 1;
    }

    return 0;
}

/* Check if OOB status has channel enable change event
 * param status = OOB TX status register value
 * return 0: no channel enable change
 * return 1: disable to enable change
 * return -1: enable to disable change
 */
int mec_hal_espi_oob_up_is_chan_event(uint32_t status)
{
    int ev = 0; /* no event */

    if (status & MEC_BIT(MEC_ESPI_OOB_TX_SR_CENC_POS)) {
        if (status & MEC_BIT(MEC_ESPI_OOB_TX_SR_CHEN_POS)) {
            ev = 1; /* 0 -> 1 is enable */
        } else {
            ev = -1; /* 1 -> 0 disable */
        }
    }

    return ev;
}

void mec_hal_espi_oob_status_clr_done(uintptr_t iorb, uint8_t dir)
{
   if (dir == MEC_ESPI_OOB_DIR_UP) {
       mmcr32_set_bit(iorb + MEC_ESPI_OOB_TX_SR_OFS, MEC_ESPI_OOB_TX_SR_DONE_POS);
   } else {
       mmcr32_set_bit(iorb + MEC_ESPI_OOB_RX_SR_OFS, MEC_ESPI_OOB_RX_SR_DONE_POS);
   }
}

void mec_hal_espi_oob_status_clr_err(uintptr_t iorb, uint8_t dir)
{
    if (dir == MEC_ESPI_OOB_DIR_UP) {
        mmcr32_set_bits(iorb + MEC_ESPI_OOB_TX_SR_OFS, MEC_ESPI_OOB_TX_SR_ALL_ERR_MSK);
    } else {
        mmcr32_set_bits(iorb + MEC_ESPI_OOB_RX_SR_OFS, MEC_ESPI_OOB_RX_SR_ALL_ERR_MSK);
    }
}

void mec_hal_espi_oob_status_clr_chen_change(uintptr_t iorb)
{
    mmcr32_set_bit(iorb + MEC_ESPI_OOB_TX_SR_OFS, MEC_ESPI_OOB_TX_SR_CENC_POS);
}

void mec_hal_espi_oob_status_clr_all(uintptr_t iorb, uint8_t dir)
{
    uint32_t msk = MEC_BIT(MEC_ESPI_OOB_RX_SR_DONE_POS) | MEC_ESPI_OOB_RX_SR_ALL_ERR_MSK;

    if (dir == MEC_ESPI_OOB_DIR_DN) {
        mmcr32_set_bits(iorb + MEC_ESPI_OOB_RX_SR_OFS, msk);
    } else {
        msk = MEC_BIT(MEC_ESPI_OOB_TX_SR_DONE_POS) | MEC_ESPI_OOB_TX_SR_ALL_ERR_MSK;
        mmcr32_set_bits(iorb + MEC_ESPI_OOB_TX_SR_OFS, msk);
    }
}

/* end mec_espi_oob.c */
