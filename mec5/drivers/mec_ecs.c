/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include <mec_ecs_regs.h>

#include "mec_defs.h"
#include "mec_ecs_api.h"
#include "mec_mmcr.h"
#include "mec_retval.h"

void mec_hal_ecs_ictrl(uint8_t direct_en)
{
    uintptr_t raddr = MEC_ECS_BASE + MEC_ECS_ICR_OFS;

    if (direct_en) {
        mmcr32_set_bit(raddr, MEC_ECS_ICR_DIRECT_EN_POS);
    } else {
        mmcr32_clr_bit(raddr, MEC_ECS_ICR_DIRECT_EN_POS);
    }
}

int mec_hal_ecs_is_idirect(void)
{
    return mmcr32_test_bit(MEC_ECS_BASE + MEC_ECS_ICR_OFS, MEC_ECS_ICR_DIRECT_EN_POS);
}

void mec_hal_ecs_ahb_error_ctrl(uint8_t ahb_err_enable)
{
    uintptr_t raddr = MEC_ECS_BASE + MEC_ECS_AERC_OFS;

    if (ahb_err_enable) { /* clear AHB error capture disable bit */
        mmcr32_clr_bit(raddr, MEC_ECS_AERC_DIS_POS);
    } else {
        mmcr32_set_bit(raddr, MEC_ECS_AERC_DIS_POS);
    }
}

uint32_t mec_hal_ecs_ahb_error_val(uint8_t clr_after_read)
{
    uintptr_t raddr = MEC_ECS_BASE + MEC_ECS_AERC_OFS;
    uint32_t ahb_error_val = mmcr32_rd(raddr);

    if (clr_after_read != 0) {
        mmcr32_wr(0, raddr);
    }

    return ahb_error_val;
}

int mec_hal_ecs_is_feature_disabled(uint8_t feature)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    if (feature < 32) {
        return mmcr32_test_bit(ecs_base + MEC_ECS_FLOCK_OFS, feature);
    } else if (feature < 64) {
        feature -= 32;
        return mmcr32_test_bit(ecs_base + MEC_ECS_MLOCK_OFS, feature);
    }

    return 0;
}

void mec_hal_ecs_etm_pins(uint8_t enable)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    if (enable) {
        mmcr32_set_bit(ecs_base + MEC_ECS_DTR_OFS, MEC_ECS_DTR_ETM_EN_POS);
    } else {
        mmcr32_clr_bit(ecs_base + MEC_ECS_DTR_OFS, MEC_ECS_DTR_ETM_EN_POS);
    }
}

int mec_hal_ecs_debug_port(enum mec_debug_mode mode)
{
    uintptr_t ecs_base = MEC_ECS_BASE;
    uint32_t val = 0;

    switch (mode) {
    case MEC_DEBUG_MODE_DISABLE:
        mmcr32_clr_bit(ecs_base + MEC_ECS_DCR_OFS, MEC_ECS_DCR_EN_POS);
        return MEC_RET_OK;
    case MEC_DEBUG_MODE_JTAG:
        val = MEC_ECS_DCR_CFG_SET(MEC_ECS_DCR_CFG_JTAG);
        break;
    case MEC_DEBUG_MODE_SWD:
        val = MEC_ECS_DCR_CFG_SET(MEC_ECS_DCR_CFG_SWD);
        break;
    case MEC_DEBUG_MODE_SWD_SWV:
        val = MEC_ECS_DCR_CFG_SET(MEC_ECS_DCR_CFG_SWD_SWV);
        break;
    default:
        return MEC_RET_ERR_INVAL;
    }

    mmcr32_update_field(ecs_base + MEC_ECS_DCR_OFS, val, MEC_ECS_DCR_CFG_MSK);

    return MEC_RET_OK;
}

/* -------- Analog Comparator -------- */

/* Configure Analog comparator - enables, deep sleep enables, and comparator 0 config lock.
 * NOTE 1: Once comparator 0 is locked its configuration bits cannot be changed until RESET_SYS.
 * NOTE 2: Caller responsible for configuring comparator pins.
 */
void mec_hal_ecs_analog_comparator_config(uint32_t config)
{
    uintptr_t ecs_base = MEC_ECS_BASE;
    uint32_t msk = MEC_ECS_ACMP_CR_MSK;
    uint32_t val = 0;

    /* enables and lock */
    if ((config & MEC_ACMP_CFG_EN0) != 0) {
        val |= MEC_BIT(MEC_ECS_ACMP_CR_EN0_POS);
    }

    if ((config & MEC_ACMP_CFG_LOCK0) != 0) {
        val |= MEC_BIT(MEC_ECS_ACMP_CR_LOCK0_POS);
    }

    if ((config & MEC_ACMP_CFG_EN1) != 0) {
        val |= MEC_BIT(MEC_ECS_ACMP_CR_EN1_POS);
    }

    mmcr32_update_field(ecs_base + MEC_ECS_ACMP_CR_OFS, val, msk);

    /* deep sleep bits. program after enables */
    msk = MEC_BIT(MEC_ECS_ACMP_SLP_EN0_POS) | MEC_BIT(MEC_ECS_ACMP_SLP_EN1_POS);

    if ((config & MEC_ACMP_CFG_DS0) != 0) {
        val |= MEC_BIT(MEC_ECS_ACMP_SLP_EN0_POS);
    }

    if ((config & MEC_ACMP_CFG_DS1) != 0) {
        val |= MEC_BIT(MEC_ECS_ACMP_SLP_EN1_POS);
    }

    mmcr32_update_field(ecs_base + MEC_ECS_ACMP_SLP_CR_OFS, val, msk);
}

/* -------- Embedded Reset -------- */

bool mec_hal_ecs_emb_reset_is_enabled(void)
{
    if (mmcr32_test_bit(MEC_ECS_BASE + MEC_ECS_ERST_CR_OFS, MEC_ECS_ERST_CR_EN_POS) != 0) {
        return true;
    }

    return false;
}

void mec_hal_ecs_emb_reset_enable(uint8_t enable)
{
    if (enable) {
        mmcr32_set_bit(MEC_ECS_BASE + MEC_ECS_ERST_CR_OFS, MEC_ECS_ERST_CR_EN_POS);
    } else {
        mmcr32_clr_bit(MEC_ECS_BASE + MEC_ECS_ERST_CR_OFS, MEC_ECS_ERST_CR_EN_POS);
    }
}

uint8_t mec_hal_ecs_emb_reset_timeout_get(void)
{
    uint32_t val = mmcr32_rd(MEC_ECS_BASE + MEC_ECS_ERST_TMCR_OFS);

    return (uint8_t)MEC_ECS_ERST_TMV_GET(val);
}

void mec_hal_ecs_emb_reset_timeout(uint8_t timeout)
{
    uint32_t v = MEC_ECS_ERST_TMV_SET(timeout);

    mmcr32_update_field(MEC_ECS_BASE + MEC_ECS_ERST_TMCR_OFS, v, MEC_ECS_ERST_TMV_MSK);
}

uint32_t mec_hal_ecs_emb_reset_status(void)
{
    return mmcr32_rd(MEC_ECS_BASE + MEC_ECS_ERST_SR_OFS);
}

void mec_hal_ecs_emb_reset_status_clear(void)
{
    mmcr32_wr(MEC_BIT(MEC_ECS_ERST_SR_ACTV_POS), MEC_ECS_BASE + MEC_ECS_ERST_SR_OFS);
}

uint32_t mec_hal_ecs_emb_reset_count(void)
{
    uint32_t v = mmcr32_rd(MEC_ECS_BASE + MEC_ECS_ERST_CNTR_OFS);

    return MEC_ECS_ERST_CNT_GET(v);
}

/* ---- PECI VTT Vref pin control ---- */
void mec_hal_ecs_peci_vtt_ref_pin_ctrl(uint8_t enable)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    if (enable) {
        mmcr32_set_bit(ecs_base + MEC_ECS_PECI_DIS_OFS, MEC_ECS_PECI_VREF_OFF_POS);
    } else {
        mmcr32_clr_bit(ecs_base + MEC_ECS_PECI_DIS_OFS, MEC_ECS_PECI_VREF_OFF_POS);
    }
}

uint8_t mec_hal_ecs_peci_vtt_ref_pin_is_enabled(void)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    if (mmcr32_test_bit(ecs_base + MEC_ECS_PECI_DIS_OFS, MEC_ECS_PECI_VREF_OFF_POS) == 0) {
        return 1u;
    }

    return 0;
}

/* ---- Power management ----
 * Debug interface and ETM control registers.
 */

#define MEC_ECS_PM_SAVE_ITEMS_CNT 4
static uint8_t ecs_pm_save_buf[MEC_ECS_PM_SAVE_ITEMS_CNT];

void mec_hal_ecs_debug_ifc_save_disable(void)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    ecs_pm_save_buf[0] = (uint8_t)(mmcr32_rd(ecs_base + MEC_ECS_DTR_OFS) & 0xffu);
    ecs_pm_save_buf[1] = (uint8_t)(mmcr32_rd(ecs_base + MEC_ECS_DCR_OFS) & 0xffu);

    mmcr32_wr(0, ecs_base + MEC_ECS_DTR_OFS);
    mmcr32_wr(0, ecs_base + MEC_ECS_DCR_OFS);
}

void mec_hal_ecs_debug_ifc_restore(void)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    mmcr32_wr(ecs_pm_save_buf[0], ecs_base + MEC_ECS_DTR_OFS);
    mmcr32_wr(ecs_pm_save_buf[1], ecs_base + MEC_ECS_DCR_OFS);
}


void mec_hal_ecs_pm_save_disable(void)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    mec_hal_ecs_debug_ifc_save_disable();

    ecs_pm_save_buf[2] = (uint8_t)(mmcr32_rd(ecs_base + MEC_ECS_PECI_DIS_OFS) & 0xffu);

    mec_hal_ecs_peci_vtt_ref_pin_ctrl(0);

    mmcr32_set_bits(ecs_base + MEC_ECS_ACMP_SLP_CR_OFS,
                    MEC_BIT(MEC_ECS_ACMP_SLP_EN0_POS) | MEC_BIT(MEC_ECS_ACMP_SLP_EN1_POS));
}

void mec_hal_ecs_pm_restore(void)
{
    uintptr_t ecs_base = MEC_ECS_BASE;

    mec_hal_ecs_debug_ifc_restore();

    mmcr32_update_field(ecs_base + MEC_ECS_PECI_DIS_OFS, ecs_pm_save_buf[2], 0xffu);

    mmcr32_clr_bits(ecs_base + MEC_ECS_ACMP_SLP_CR_OFS,
                    MEC_BIT(MEC_ECS_ACMP_SLP_EN0_POS) | MEC_BIT(MEC_ECS_ACMP_SLP_EN1_POS));
}

/* end mec_ecs.c */
