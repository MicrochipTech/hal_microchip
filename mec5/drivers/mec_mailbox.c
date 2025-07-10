/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include <mec_mailbox_regs.h>
#include "mec_defs.h"
#include "mec_ecia_api.h"
#include "mec_pcr_api.h"
#include "mec_mailbox_api.h"
#include "mec_retval.h"
#include "mec_mmcr.h"

struct mec_mbox_info {
    uint32_t base_addr;
    uint32_t devi;
    uint16_t pcr_id;
};

static const struct mec_mbox_info mbox_instances[MEC5_MAILBOX_INSTANCES] = {
    { MEC_MBOX0_BASE, MEC_MBOX0_ECIA_INFO, MEC_PCR_MBOX0 },
};

static struct mec_mbox_info const *find_mbox_info(uint32_t base_addr)
{
    for (size_t i = 0; i < MEC5_MAILBOX_INSTANCES; i++) {
        if (base_addr == mbox_instances[i].base_addr) {
            return &mbox_instances[i];
        }
    }

    return NULL;
}


/* ---- Public API ---- */

int mec_hal_mbox_init(uintptr_t regbase, uint32_t swi_ien_msk, uint32_t flags)
{
    const struct mec_mbox_info *mbi = find_mbox_info((uint32_t)regbase);

    if (mbi == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_pcr_clr_blk_slp_en(mbi->pcr_id);
    mec_hal_girq_ctrl(mbi->devi, 0);

    if (flags & MEC_MBOX_FLAG_RESET) {
        mec_hal_pcr_blk_reset(mbi->pcr_id);
    } else {
        mmcr8_wr(0xffu, regbase + MBOX_H2E_OFS); /* clear EC MBOX interrupt status */
    }

    mec_hal_girq_clr_src(mbi->devi);

    mmcr8_wr((uint8_t)(swi_ien_msk & 0xffu), regbase + MBOX_SMI_IM_OFS);

    if (flags & MEC_MBOX_FLAG_INTR_EN) {
        mec_hal_girq_ctrl(mbi->devi, 1);
    }

    return MEC_RET_OK;
}

int mec_hal_mbox_girq_ctrl(uintptr_t regbase, uint8_t enable)
{
    const struct mec_mbox_info *mbi = find_mbox_info((uint32_t)regbase);

    if (mbi == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_girq_ctrl(mbi->devi, enable);

    return MEC_RET_OK;
}

int mec_hal_mbox_girq_clr(uintptr_t regbase)
{
    const struct mec_mbox_info *mbi = find_mbox_info((uint32_t)regbase);

    if (mbi == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_girq_clr_src(mbi->devi);

    return MEC_RET_OK;
}

uint32_t mec_hal_mbox_girq_result(uintptr_t regbase)
{
    const struct mec_mbox_info *mbi = find_mbox_info((uint32_t)regbase);

    if (mbi == NULL) {
        return 0;
    }

    return mec_hal_girq_result(mbi->devi);
}

int mec_hal_mbox_sirq_set(uintptr_t regbase, uint8_t bitmap)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    if (bitmap != 0) {
        mmcr8_wr(bitmap, regbase + MBOX_SMI_IS_OFS);
    }

    return MEC_RET_OK;
}

int mec_hal_mbox_sirq_en_mask(uintptr_t regbase, uint8_t val, uint8_t mask)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    if (mask != 0) {
        mmcr8_update_field(regbase + MBOX_SMI_IM_OFS, val, mask);
    }

    return MEC_RET_OK;
}

int mec_hal_mbox_get_host_to_ec(uintptr_t regbase, uint8_t *data)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    uint8_t host_to_ec = mmcr8_rd(regbase + MBOX_H2E_OFS);

    if (data != NULL) {
        *data = host_to_ec;
    }

    return MEC_RET_OK;
}

int mec_hal_mbox_set_host_to_ec(uintptr_t regbase, uint8_t data)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    mmcr8_wr(data, regbase + MBOX_H2E_OFS);

    return MEC_RET_OK;
}

int mec_hal_mbox_get_ec_to_host(uintptr_t regbase, uint8_t *data)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    uint8_t ec_to_host = mmcr8_rd(regbase + MBOX_E2H_OFS);

    if (data != NULL) {
        *data = ec_to_host;
    }

    return MEC_RET_OK;
}

int mec_hal_mbox_set_ec_to_host(uintptr_t regbase, uint8_t data)
{
    if (regbase != MEC_MBOX0_BASE) {
        return MEC_RET_ERR_INVAL;
    }

    mmcr8_wr(data, regbase + MBOX_E2H_OFS);

    return MEC_RET_OK;
}

int mec_hal_mbox_get(uintptr_t regbase, uint8_t mbox, uint8_t *data)
{
    if ((regbase != MEC_MBOX0_BASE) || (data == NULL) || (mbox > MEC_MBOX_MAX_INDEX)) {
        return MEC_RET_ERR_INVAL;
    }

    *data = mmcr8_rd(regbase + MBOX_MB_OFS(mbox));

    return MEC_RET_OK;
}

int mec_hal_mbox_put(uintptr_t regbase, uint8_t mbox, uint8_t data)
{
    if ((regbase != MEC_MBOX0_BASE) || (mbox > MEC_MBOX_MAX_INDEX)) {
        return MEC_RET_ERR_INVAL;
    }

    mmcr8_wr(data, regbase + MBOX_MB_OFS(mbox));

    return MEC_RET_OK;
}

/* 32 8-bit mailboxes are grouped as 8 32-bit registers */
int mec_hal_mbox32_get(uintptr_t regbase, uint8_t mbox, uint32_t *data)
{
    uint32_t mbox_ofs = MBOX_MB_OFS(0);

    if ((regbase != MEC_MBOX0_BASE) || (data == NULL) || (mbox > (MEC_MBOX_MAX_INDEX / 4))) {
        return MEC_RET_ERR_INVAL;
    }

    mbox_ofs += (uint32_t)mbox * 4u;
    *data = mmcr32_rd(regbase + mbox_ofs);

    return MEC_RET_OK;
}

int mec_hal_mbox32_put(uintptr_t regbase, uint8_t mbox, uint32_t data)
{
    uint32_t mbox_ofs = MBOX_MB_OFS(0);

    if ((regbase != MEC_MBOX0_BASE) || (mbox > (MEC_MBOX_MAX_INDEX / 4))) {
        return MEC_RET_ERR_INVAL;
    }

    mbox_ofs += (uint32_t)mbox * 4u;
    mmcr32_wr(data, regbase + mbox_ofs);

    return MEC_RET_OK;
}

/* end mec_mailbox.c */
