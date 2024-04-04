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
#include "mec_btimer_api.h"
#include "mec_ecia_api.h"
#include "mec_pcr_api.h"
#include "mec_retval.h"

#define MEC_BTIMER_GIRQ 23

#define MEC_BTIMER0_GIRQ_POS 0
#define MEC_BTIMER1_GIRQ_POS 1
#define MEC_BTIMER2_GIRQ_POS 2
#define MEC_BTIMER3_GIRQ_POS 3
#define MEC_BTIMER4_GIRQ_POS 4
#define MEC_BTIMER5_GIRQ_POS 5

#define MEC_BTIMER0_ECIA_INFO MEC5_ECIA_INFO(23, 0, 14, 136)
#define MEC_BTIMER1_ECIA_INFO MEC5_ECIA_INFO(23, 1, 14, 137)
#define MEC_BTIMER2_ECIA_INFO MEC5_ECIA_INFO(23, 2, 14, 138)
#define MEC_BTIMER3_ECIA_INFO MEC5_ECIA_INFO(23, 3, 14, 139)
#define MEC_BTIMER4_ECIA_INFO MEC5_ECIA_INFO(23, 4, 14, 140)
#define MEC_BTIMER5_ECIA_INFO MEC5_ECIA_INFO(23, 5, 14, 141)

#define MEC5_BTMR_PRESCALE_MSK0 (BTMR_CTRL_PRESCALE_Msk >> BTMR_CTRL_PRESCALE_Pos)

struct mec_btimer_info {
    uintptr_t base_addr;
    uint16_t pcr_id;
    uint32_t devi;
};

static const struct mec_btimer_info btimer_instances[MEC5_BASIC_TIMER_INSTANCES] = {
    { BTMR0_BASE, MEC_PCR_BTMR0, MEC_BTIMER0_ECIA_INFO },
    { BTMR1_BASE, MEC_PCR_BTMR1, MEC_BTIMER1_ECIA_INFO },
    { BTMR2_BASE, MEC_PCR_BTMR2, MEC_BTIMER2_ECIA_INFO },
    { BTMR3_BASE, MEC_PCR_BTMR3, MEC_BTIMER3_ECIA_INFO },
    { BTMR4_BASE, MEC_PCR_BTMR4, MEC_BTIMER4_ECIA_INFO },
    { BTMR5_BASE, MEC_PCR_BTMR5, MEC_BTIMER5_ECIA_INFO },
};

static struct mec_btimer_info const *find_btimer_info(uintptr_t base_addr)
{
    for (size_t i = 0; i < MEC5_BASIC_TIMER_INSTANCES; i++) {
        if (base_addr == btimer_instances[i].base_addr) {
            return &btimer_instances[i];
        }
    }

    return NULL;
}

static int find_btimer_index(uintptr_t base_addr)
{
    for (int i = 0; i < (int)MEC5_BASIC_TIMER_INSTANCES; i++) {
        if (base_addr == btimer_instances[i].base_addr) {
            return i;
        }
    }

    return -1;
}

/* ---- Public API ---- */

/* Initialize a basic timer instance.
 * TODO: timer frequency = max_freq / (pre_scale + 1)
 * pre_scale = b[31:16] of Control register.
 */
int mec_btimer_init(struct btmr_regs *regs, uint32_t freq_div,
                    uint32_t count, uint32_t flags)
{
    const struct mec_btimer_info *info = find_btimer_info((uintptr_t)regs);

    if (!info || !freq_div) {
        return MEC_RET_ERR_INVAL;
    }

    mec_girq_ctrl(info->devi, 0);
    mec_pcr_clr_blk_slp_en(info->pcr_id);

    regs->CTRL = BIT(BTMR_CTRL_RESET_Pos);
    regs->CTRL = ((freq_div - 1u) & MEC5_BTMR_PRESCALE_MSK0) << BTMR_CTRL_PRESCALE_Pos;
    regs->PRELOAD = count;
    regs->COUNT = count;

    if (flags & BIT(MEC5_BTIMER_CFG_FLAG_AUTO_RELOAD_POS)) {
        regs->CTRL |= BIT(BTMR_CTRL_RELOAD_Pos);
    }

    if (flags & BIT(MEC5_BTIMER_CFG_FLAG_COUNT_UP_POS)) {
        regs->CTRL |= BIT(BTMR_CTRL_CNT_DIR_Pos);
    }

    if (flags & BIT(MEC5_BTIMER_CFG_FLAG_INTR_EN_POS)) {
        regs->IEN |= BIT(BTMR_IEN_EVENT_Pos);
        mec_girq_ctrl(info->devi, 1);
    }

    if (flags & BIT(MEC5_BTIMER_CFG_FLAG_START_POS)) {
        regs->CTRL |= BIT(BTMR_CTRL_START_Pos);
    }

    return MEC_RET_OK;
}

int mec_btimer_has_counter32(struct btmr_regs *regs)
{
    int idx = find_btimer_index((uint32_t)regs);

    if ((idx >= 0) && (idx < 32)) {
        if (BIT(idx) & MEC5_BASIC_TIMER_32_MSK) {
            return 1;
        }
    }

    return 0;
}

uint32_t mec_btimer_freq(struct btmr_regs *regs)
{
    uint32_t freqhz = 0u;

    if (regs) {
        freqhz = (regs->CTRL & BTMR_CTRL_PRESCALE_Msk) >> BTMR_CTRL_PRESCALE_Pos;
        freqhz = MEC5_BTIMER_MAX_FREQ_HZ / (freqhz + 1u); /* truncates */
    }

    return freqhz;
}

int mec_btimer_girq_ctrl(struct btmr_regs *regs, uint8_t enable)
{
    const struct mec_btimer_info *info = find_btimer_info((uintptr_t)regs);

    if (!info) {
        return MEC_RET_ERR_INVAL;
    }

    mec_girq_ctrl(info->devi, enable);

    return MEC_RET_OK;
}

int mec_btimer_girq_status_clr(struct btmr_regs *regs)
{
    const struct mec_btimer_info *info = find_btimer_info((uintptr_t)regs);

    if (!regs) {
        return MEC_RET_ERR_INVAL;
    }

    mec_girq_clr_src(info->devi);

    return MEC_RET_OK;
}

uint32_t mec_btimer_count(struct btmr_regs *regs)
{
    return regs->COUNT;
}

uint32_t mec_btimer_preload(struct btmr_regs *regs)
{
    return regs->PRELOAD;
}

void mec_btimer_start(struct btmr_regs *regs)
{
    regs->CTRL |= BIT(BTMR_CTRL_START_Pos);
}

void mec_btimer_stop(struct btmr_regs *regs)
{
    regs->CTRL &= (uint32_t)~BIT(BTMR_CTRL_START_Pos);
}

void mec_btimer_halt(struct btmr_regs *regs)
{
    regs->CTRL |= BIT(BTMR_CTRL_HALT_Pos);
}

void mec_btimer_unhalt(struct btmr_regs *regs)
{
    regs->CTRL &= (uint32_t)~BIT(BTMR_CTRL_HALT_Pos);
}

void mec_btimer_reload(struct btmr_regs *regs)
{
    regs->CTRL |= BIT(BTMR_CTRL_RELOAD_Pos);
}

void mec_btimer_intr_clr(struct btmr_regs *regs)
{
    regs->STATUS = BIT(BTMR_STATUS_EVENT_Pos);
}

void mec_btimer_intr_en(struct btmr_regs *regs, uint8_t enable)
{
    if (enable) {
        regs->IEN |= BIT(BTMR_IEN_EVENT_Pos);
    } else {
        regs->IEN &= (uint32_t)~BIT(BTMR_IEN_EVENT_Pos);
    }
}

/* end mec_btimer.c */
