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
#include "mec_tach_api.h"
#include "mec_retval.h"

#if MEC5_TACH_INSTANCES > 0

#define MEC_TACH0_ECIA_INFO MEC5_ECIA_INFO(17, 1, 9, 71)
#define MEC_TACH1_ECIA_INFO MEC5_ECIA_INFO(17, 2, 9, 72)
#define MEC_TACH2_ECIA_INFO MEC5_ECIA_INFO(17, 3, 9, 73)
#define MEC_TACH3_ECIA_INFO MEC5_ECIA_INFO(17, 4, 9, 159)

struct mec_tach_info {
    uintptr_t base_addr;
    uint32_t devi;
    uint16_t pcr_id;
};

static const struct mec_tach_info tach_instances[MEC5_TACH_INSTANCES] = {
    {TACH0_BASE, MEC_TACH0_ECIA_INFO, MEC_PCR_TACH0 },
    {TACH1_BASE, MEC_TACH1_ECIA_INFO, MEC_PCR_TACH1 },
    {TACH2_BASE, MEC_TACH2_ECIA_INFO, MEC_PCR_TACH2 },
    {TACH3_BASE, MEC_TACH3_ECIA_INFO, MEC_PCR_TACH3 },
};

static struct mec_tach_info const *tach_get_info(struct tach_regs *regs)
{
    for (int i = 0; i < MEC5_TACH_INSTANCES; i++) {
        const struct mec_tach_info *p = &tach_instances[i];

        if (tach_instances[i].base_addr == (uintptr_t)regs) {
            return p;
        }
    }

    return NULL;
}

#if 0
#define MEC5_TACH_CFG_RESET 0x01u
#define MEC5_TACH_CFG_ENABLE 0x02u
#define MEC5_TACH_CFG_FILTER_EN 0x04u
#define MEC5_TACH_CFG_CNT_INCR_CLK 0x08u
#define MEC5_TACH_CFG_OOL_INTR_EN 0x10u
#define MEC5_TACH_CFG_CNT_RDY_INTR_EN 0x20u
#define MEC5_TACH_CFG_INPUT_CHG_INTR_EN 0x40u
#define MEC5_TACH_CFG_INTERVAL_EDGES_POS 8
#define MEC5_TACH_CFG_INTERVAL_EDGES_MSK 0x300u
#define MEC5_TACH_CFG_INTERVAL_EDGES_2 0
#define MEC5_TACH_CFG_INTERVAL_EDGES_3 0x100u
#define MEC5_TACH_CFG_INTERVAL_EDGES_5 0x200u
#define MEC5_TACH_CFG_INTERVAL_EDGES_9 0x300u
#endif

int mec_tach_init(struct tach_regs *regs, uint32_t limits, uint32_t flags)
{
    const struct mec_tach_info *info = tach_get_info(regs);
    uint32_t ctrl = 0, temp = 0;

    if (!info) {
        return MEC_RET_ERR_INVAL;
    }

    mec_pcr_clr_blk_slp_en(info->pcr_id);
    if (flags & MEC5_TACH_CFG_RESET) {
        mec_pcr_blk_reset(info->pcr_id);
    } else {
        regs->CTRL = 0u;
        regs->STATUS = UINT32_MAX;
    }

    mec_girq_ctrl(info->devi, 0);
    mec_girq_clr_src(info->devi);

    /* program high and low 16-bit counter limits */
    regs->LIMIT_LO = (uint16_t)(limits & 0xffffu);
    regs->LIMIT_HI = (uint16_t)(limits >> 16);

    /* program number of tach edges for the count interval */
    temp = (flags & MEC5_TACH_CFG_INTERVAL_EDGES_MSK) >> MEC5_TACH_CFG_INTERVAL_EDGES_POS;
    ctrl |= ((temp << TACH_CTRL_EDGES_Pos) & TACH_CTRL_EDGES_Msk);

    if (flags & MEC5_TACH_CFG_FILTER_EN) {
        ctrl |= BIT(TACH_CTRL_FILT_IN_Pos);
    }

    /* counter is incremented on rising edge of tach input or rising edge
     * of Tach input clock. Input clock in the PCR slow clock.
     */
    if (flags & MEC5_TACH_CFG_CNT_INCR_CLK) {
        ctrl |= BIT(TACH_CTRL_RDMODE_Pos);
    }

    if (flags & MEC5_TACH_CFG_OOL_INTR_EN) { /* out of limit interrupt? */
        ctrl |= BIT(TACH_CTRL_ENOOL_Pos);
    }

    if (flags & MEC5_TACH_CFG_CNT_RDY_INTR_EN) {
        ctrl |= BIT(TACH_CTRL_CNTRDY_IEN_Pos);
    }

    if (flags & MEC5_TACH_CFG_INPUT_CHG_INTR_EN) {
        ctrl |= BIT(TACH_CTRL_INTOG_IEN_Pos);
    }

    if (flags & MEC5_TACH_CFG_ENABLE) {
        ctrl |= BIT(TACH_CTRL_ENABLE_Pos);
    }

    regs->CTRL = ctrl;
    mec_girq_ctrl(info->devi, 1);

    return MEC_RET_OK;
}

void tach_enable(struct tach_regs *regs, uint8_t enable)
{
    if (enable) {
        regs->CTRL |= BIT(TACH_CTRL_ENABLE_Pos);
    } else {
        regs->CTRL &= (uint32_t)~BIT(TACH_CTRL_ENABLE_Pos);
    }
}

uint32_t mec_tach_clock_freq(void)
{
    return mec_pcr_slow_clock_freq_get();
}

uint32_t mec_tach_counter(struct tach_regs *regs)
{
    const struct mec_tach_info *info = tach_get_info(regs);

    if (!info) {
        return 0;
    }

    return (regs->CTRL * TACH_CTRL_COUNT_Msk) >> TACH_CTRL_COUNT_Pos;
}

uint32_t mec_tach_status(struct tach_regs *regs)
{
    return regs->STATUS;
}

void mec_tach_status_clr(struct tach_regs *regs, uint32_t status)
{
    regs->STATUS = status;
}

#endif /* MEC5_TACH_INSTANCES */
/* end mec_tach.c */
