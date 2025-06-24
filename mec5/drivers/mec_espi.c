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
#include "mec_espi_api.h"
#include "mec_pcr_api.h"
#include "mec_retval.h"
#include "mec_mmcr.h"
#include "mec_espi_regs.h"

/* ---- eSPI Reset interrupt ---- */
#define MEC_ESPI_RESET_ECIA_INFO MEC_ECIA_INFO(19, 7, 11, 110)

static void set_supported_channels(uintptr_t ioreg_base, uint32_t capabilities)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP0_OFS;
    uint8_t mask = MEC_ESPI_CAP0_SUPP_MSK;
    uint8_t v = 0;

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS)) {
        v |= MEC_BIT(MEC_ESPI_CAP0_PC_SUPP_POS);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_VW_CHAN_SUP_POS)) {
        v |= MEC_BIT(MEC_ESPI_CAP0_VW_SUPP_POS);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_OOB_CHAN_SUP_POS)) {
        v |= MEC_BIT(MEC_ESPI_CAP0_OOB_SUPP_POS);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_FLASH_CHAN_SUP_POS)) {
        v |= MEC_BIT(MEC_ESPI_CAP0_PC_SUPP_POS);
    }

    mmcr8_update_field(rb, v, mask);
}

static void set_supported_max_freq(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP1_OFS;
    uint8_t mask = MEC_ESPI_CAP1_MAX_FREQ_MSK;
    uint8_t v = 0;

    v = (uint8_t)((caps >> MEC_ESPI_CFG_MAX_SUPP_FREQ_POS) & MEC_ESPI_CFG_MAX_SUPP_FREQ_MSK0);

    mmcr8_update_field(rb, v, mask);
}

/* eSPI capabilties 1 register has maximum supported frequency field.
 * The field values match the API enum mec_espi_max_freq values.
 */
static uint32_t get_max_freq(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP1_OFS;
    uint32_t encoded_freq = mmcr8_rd(rb);

    encoded_freq = MEC_ESPI_CAP1_MAX_FREQ_GET(encoded_freq);

    return encoded_freq;
}

static void set_supported_io_modes(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP1_OFS;
    uint32_t v = ((caps & MEC_ESPI_CFG_IO_MODE_SUPP_MSK) >> MEC_ESPI_CFG_IO_MODE_SUPP_POS);

    v |= MEC_ESPI_CAP1_IOM_SET(v);

    mmcr8_update_field(rb, v, MEC_ESPI_CAP1_IOM_MSK);
}

/* eSPI capabilities 1 register contains a bitfield for supported I/O modes.
 * Our API enum mec_espi_io_mode values are indentical hardware bitfield values.
 */
static uint32_t get_supported_io_modes(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP1_OFS;
    uint32_t iom = mmcr8_rd(rb);

    iom = MEC_ESPI_CAP1_IOM_GET(iom);

    return (iom << MEC_ESPI_CFG_IO_MODE_SUPP_POS) & MEC_ESPI_CFG_IO_MODE_SUPP_MSK;
}

/* CAP1 has 3 bits for nALERT
 * b[6] = RW ALERT pin drive type: 0=push-pull, 1=open-drain.
 *        Firmware uses this bit to advertise nALERT pin drive type is push-pull only(0) or
 *        can do both push-pull and open-drain (1).
 * b[7] = RO from HW Config 08h b[23]: Host selected 0=push-pull, 1=open-drain
 * b[3] = RO rom HW Config 08h b[28]: Host selected 0=nALERT muxed on IO[1], 1=nALERT on pin
 */
static void set_supported_alert_io_pin_mode(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP1_OFS;

    if (caps & MEC_BIT(MEC_ESPI_CFG_ALERT_OD_SUPP_POS)) { /* we support push-pull & open-drain */
        mmcr8_set_bit(rb, MEC_ESPI_CAP1_ALERT_OD_CAP_POS);
    } else { /* inform Host we only support push-pull nALERT pin */
        mmcr8_clr_bit(rb, MEC_ESPI_CAP1_ALERT_OD_CAP_POS);
    }
}

static void set_pc_capabilities(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_PC_OFS;
    uint32_t v = (caps & MEC_ESPI_CFG_PC_MAX_PLD_SZ_MSK) >> MEC_ESPI_CFG_PC_MAX_PLD_SZ_POS; 

    v = MEC_ESPI_CAP_PC_MPLS_SET(v);

    mmcr8_update_field(rb, v, MEC_ESPI_CAP_PC_MPLS_MSK);
}

static uint32_t get_pc_max_pld_size(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_PC_OFS;
    uint32_t v = mmcr8_rd(rb);
 
    return ((v << MEC_ESPI_CFG_PC_MAX_PLD_SZ_POS) & MEC_ESPI_CFG_PC_MAX_PLD_SZ_MSK);
}

static void set_vw_capabilities(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_VW_OFS;
    uint32_t v = (caps & MEC_ESPI_CFG_VW_CNT_MAX_MSK) >> MEC_ESPI_CFG_VW_CNT_MAX_POS;

    v = MEC_ESPI_CAP_VW_MAX_GRPS_SET(v);
    
    mmcr8_update_field(rb, v, MEC_ESPI_CAP_VW_MAX_GRPS_MSK);
}

static uint32_t get_vw_groups_max_cnt(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_VW_OFS;
    uint32_t v = mmcr8_rd(rb);

    v = MEC_ESPI_CAP_VW_MAX_GRPS_GET(v);

    return ((v << MEC_ESPI_CFG_VW_CNT_MAX_POS) & MEC_ESPI_CFG_VW_CNT_MAX_MSK);
}

static void set_oob_capabilities(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_OOB_OFS;
    uint32_t v = (caps & MEC_ESPI_CFG_OOB_MAX_PLD_SZ_MSK) >> MEC_ESPI_CFG_OOB_MAX_PLD_SZ_POS;

    v = MEC_ESPI_CAP_OOB_MPLD_SET(v);

    mmcr8_update_field(rb, v, MEC_ESPI_CAP_OOB_MPLD_MSK);
}

static uint32_t get_oob_pld_size(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_OOB_OFS;
    uint32_t v = mmcr8_rd(rb);

    v = MEC_ESPI_CAP_OOB_MPLD_GET(v);
    
    return ((v << MEC_ESPI_CFG_OOB_MAX_PLD_SZ_POS) & MEC_ESPI_CFG_OOB_MAX_PLD_SZ_MSK);
}

static void set_fc_max_pld(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_FC_OFS;
    uint32_t v = (caps & MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK) >> MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_POS;

    v = MEC_ESPI_CAP_FC_MPLD_SET(v);

    mmcr8_update_field(rb, v, MEC_ESPI_CAP_FC_MPLD_MSK);
}

static uint32_t get_fc_pld_size(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_FC_OFS;
    uint32_t v = mmcr8_rd(rb);

    v = MEC_ESPI_CAP_FC_MPLD_GET(v);

    return ((v << MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_POS) & MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK);
}

static uint8_t fc_sharing_hw(uint32_t cfg)
{
    uint8_t cap = 0;

    if (cfg & MEC_BIT(MEC_ESPI_CFG_FLASH_SHARED_TAF_POS)) {
        if (cfg & MEC_BIT(MEC_ESPI_CFG_FLASH_SHARED_CAF_POS)) {
            cap |= MEC_ESPI_CAP_FC_SM_CAF_TAF;
        } else { /* TAF only */
            cap |= MEC_ESPI_CAP_FC_SM_TAF;
        }
    } else { /* CAF only */
        cap |= MEC_ESPI_CAP_FC_SM_CAF;
    }

    return cap;
}

static uint32_t fc_sharing_get(uint32_t fc_cap)
{
    uint32_t cfg = 0;

    fc_cap = (fc_cap & MEC_ESPI_CAP_FC_SM_MSK) >> MEC_ESPI_CAP_FC_SM_POS;
    if (fc_cap == MEC_ESPI_IO_CAPFC_SHARING_SUPP_TAF) {
        cfg |= MEC_BIT(MEC_ESPI_CFG_FLASH_SHARED_TAF_POS);
    } else {
        cfg |= MEC_BIT(MEC_ESPI_CFG_FLASH_SHARED_CAF_POS);
        if (fc_cap == MEC_ESPI_IO_CAPFC_SHARING_SUPP_CAF_TAF) {
            cfg |= MEC_BIT(MEC_ESPI_CFG_FLASH_SHARED_TAF_POS);
        }
    }

    return cfg;
}

static void set_fc_shared_mode(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_FC_OFS;
    uint32_t v = fc_sharing_hw(caps);

    v = MEC_ESPI_CAP_FC_SM_SET(v);

    mmcr8_update_field(rb, v, MEC_ESPI_CAP_FC_SM_MSK);
}

static uint32_t get_fc_shared_mode(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_FC_OFS;
    uint32_t v = mmcr8_rd(rb);
    uint32_t fcsh = fc_sharing_get(v);

    return fcsh;
}

static void set_fc_capabilities(uintptr_t ioreg_base, uint32_t caps)
{
    set_fc_max_pld(ioreg_base, caps);
    set_fc_shared_mode(ioreg_base, caps);
}

/* Flash channel capabilities register TAF max payload size is fixed at 64 bytes per
 * Intel TAF specification.
 */
static uint32_t get_fc_taf_max_rdsz(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_CAP_FC_OFS;
    uint32_t v = mmcr8_rd(rb);
    uint32_t rdsz = 0;

    rdsz = MEC_ESPI_CAP_FC_TAF_MRRQ_GET(v);
    rdsz &= MEC_ESPI_CAP_FLASH_SHARED_MAX_RD_REQ_SZ_MSK0;
    rdsz <<= MEC_ESPI_CAP_FLASH_SHARED_MAX_RD_REQ_SZ_POS;

    return rdsz;
}

/* If Platform Reset is peformed is different way than eSPI PLTRST# virtual wire
 * we set a bit so our eSPI controller will ignore PLTRST# VWire.
 */
static void set_pltrst_source(uintptr_t ioreg_base, uint32_t caps)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_PLTRST_SRC_OFS;
    uint8_t host_reset_sel = 0;

    if (caps & MEC_BIT(MEC_ESPI_CFG_PLTRST_EXT_POS)) {
        mmcr8_set_bit(rb, MEC_ESPI_PLTRST_SRC_POS);
    } else { /* use PLTRST# virtual wire */
        mmcr8_clr_bit(rb, MEC_ESPI_PLTRST_SRC_POS);
        host_reset_sel = MEC_PCR_PLATFORM_RST_IS_ESPI_PLTRST;
    }

    mec_hal_pcr_host_reset_select(host_reset_sel);
}

/* ---- Public API ---- */
void mec_hal_espi_reset_change_clr(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_RESET_SR_OFS;

    mmcr8_set_bit(rb, MEC_ESPI_RESET_SR_CHG_POS);
    mec_hal_girq_clr_src(MEC_ESPI_RESET_ECIA_INFO);
}

/* Return bits indicating ESPI_RESET# has changed and its current state */
uint32_t mec_hal_espi_reset_state(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_RESET_SR_OFS;
    uint32_t v = mmcr8_rd(rb);

    return v & (MEC_BIT(MEC_ESPI_RESET_SR_CHG_POS) | MEC_BIT(MEC_ESPI_RESET_SR_STATE_POS));
}

void mec_hal_espi_reset_change_intr_en(uintptr_t ioreg_base, uint8_t enable)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_RESET_IER_OFS;

    if (enable != 0) {
        mmcr8_set_bit(rb, MEC_ESPI_RESET_IER_EN_POS);
    } else {
        mmcr8_clr_bit(rb, MEC_ESPI_RESET_IER_EN_POS);
    }
}

/* ESPI_RESET edge interrupt ECIA control */
void mec_hal_espi_reset_girq_ctrl(uint8_t enable)
{
    mec_hal_girq_ctrl(MEC_ESPI_RESET_ECIA_INFO, (int)enable);
}

void mec_hal_espi_reset_girq_status_clr(void)
{
    mec_hal_girq_clr_src(MEC_ESPI_RESET_ECIA_INFO);
}

uint32_t mec_hal_espi_reset_girq_status(void)
{
    return mec_hal_girq_src(MEC_ESPI_RESET_ECIA_INFO);
}

uint32_t mec_hal_espi_reset_girq_result(void)
{
    return mec_hal_girq_result(MEC_ESPI_RESET_ECIA_INFO);
}

/* NOTE eSPI is only fully reset by a full chip reset or power cycle.
 * The external ESPI_RESET# signal when asserted does hold portions of the logic
 * in reset state. Please refer to the Microchip eSPI block document.
 */
int mec_hal_espi_init(struct mec_espi_config *cfg)
{
    if ((cfg == 0) || (cfg->iobase == 0)) {
        return MEC_RET_ERR_INVAL;
    }

    uintptr_t ioreg_base = cfg->iobase;
    uint32_t girq_en = 0u;

    set_supported_channels(ioreg_base, cfg->capabilities);
    set_supported_max_freq(ioreg_base, cfg->capabilities);
    set_supported_io_modes(ioreg_base, cfg->capabilities);
    set_supported_alert_io_pin_mode(ioreg_base, cfg->capabilities);
    set_pc_capabilities(ioreg_base, cfg->capabilities);
    set_vw_capabilities(ioreg_base, cfg->capabilities);
    set_oob_capabilities(ioreg_base, cfg->capabilities);
    set_fc_capabilities(ioreg_base, cfg->capabilities);
    set_pltrst_source(ioreg_base, cfg->capabilities);

    mec_hal_espi_reset_change_intr_en(ioreg_base, 0);
    mec_hal_espi_reset_change_clr(ioreg_base);

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_VW_CT_GIRQ_EN_POS)) {
        MEC_ECIA0->GIRQ[MEC_GIRQ_IDX_GIRQ24].EN_SET = 0x0fffffffu;
        MEC_ECIA0->GIRQ[MEC_GIRQ_IDX_GIRQ25].EN_SET = 0x0000ffffu;
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_PC_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_PC_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_BM1_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_PC_BM1_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_BM2_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_PC_BM2_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_LTR_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_PC_LTR_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_OOB_UP_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_OOB_UP_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_OOB_DN_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_OOB_DN_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_FC_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_FC_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_VW_CHEN_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_VW_CHEN_GIRQ_POS);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_ERST_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(MEC_ESPI_RESET_GIRQ_POS);
    }

    MEC_ECIA0->GIRQ[MEC_GIRQ_IDX_GIRQ19].EN_SET = girq_en;

    return 0;
}

/* Enable eSPI controller after all static configuration has been performed.
 * MEC eSPI activate must be set before the Host de-asserts ESPI_RESET#.
 */
void mec_hal_espi_activate(uintptr_t ioreg_base, uint8_t enable)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_ACTV_OFS;

    if (enable != 0) {
        mmcr8_set_bit(rb, MEC_ESPI_ACTV_EN_POS);
    } else {
        mmcr8_clr_bit(rb, MEC_ESPI_ACTV_EN_POS);
    }
}

int mec_hal_espi_is_activated(uintptr_t ioreg_base)
{
    uintptr_t rb = ioreg_base + MEC_ESPI_ACTV_OFS;

    return mmcr8_test_bit(rb, MEC_ESPI_ACTV_EN_POS);
}

int mec_hal_espi_capability_set(uintptr_t ioreg_base, enum mec_espi_global_cap cap, uint32_t cfg)
{
    if (ioreg_base == 0) {
        return MEC_RET_ERR_INVAL;
    }

    switch (cap) {
    case MEC_ESPI_CAP_MAX_FREQ:
        set_supported_max_freq(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_IO_MODE:
        set_supported_io_modes(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_ALERT_OD:
        set_supported_alert_io_pin_mode(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_PERIPH_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS)) {
            mmcr8_set_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_PC_SUPP_POS);
        } else {
            mmcr8_clr_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_PC_SUPP_POS);
        }
        break;
    case MEC_ESPI_CAP_PC_MAX_PLD_SIZE:
        set_pc_capabilities(ioreg_base, cfg);
        break;
    case  MEC_ESPI_CAP_VWIRE_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_VW_CHAN_SUP_POS)) {
            mmcr8_set_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_VW_SUPP_POS);
        } else {
            mmcr8_clr_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_VW_SUPP_POS);
        }
        break;
    case MEC_ESPI_CAP_MAX_VW_COUNT:
        set_vw_capabilities(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_OOB_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_OOB_CHAN_SUP_POS)) {
            mmcr8_set_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_OOB_SUPP_POS);
        } else {
            mmcr8_clr_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_OOB_SUPP_POS);
        }
        break;
    case MEC_ESPI_CAP_OOB_MAX_PLD_SIZE:
        set_oob_capabilities(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_FLASH_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_FLASH_CHAN_SUP_POS)) {
            mmcr8_set_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_FC_SUPP_POS);
        } else {
            mmcr8_set_bit(ioreg_base + MEC_ESPI_CAP0_OFS, MEC_ESPI_CAP0_FC_SUPP_POS);
        }
        break;
    case MEC_ESPI_CAP_FC_MAX_PLD_SIZE:
        set_fc_max_pld(ioreg_base, cfg);
        break;
    case MEC_ESPI_CAP_FC_SHARING:
        set_fc_shared_mode(ioreg_base, cfg);
        break;
    default:
        return MEC_RET_ERR_INVAL;
    }

    return MEC_RET_OK;
}

int mec_hal_espi_capabilities_get(uintptr_t iobase, uint32_t *cfg)
{
    uint32_t cv = 0;
    uint8_t cap0 = 0, cap1 = 0;

    if ((iobase == 0) || (cfg == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    cap0 = mmcr8_rd(iobase + MEC_ESPI_CAP0_OFS);
    cap1 = mmcr8_rd(iobase + MEC_ESPI_CAP1_OFS);

    /* Max frequency */
    cv |= ((get_max_freq(iobase) << MEC_ESPI_CFG_MAX_SUPP_FREQ_POS)
           & MEC_ESPI_CFG_MAX_SUPP_FREQ_MSK);

    /* IO Mode */
    cv |= get_supported_io_modes(iobase);

    /* Supports open-drain Alert pin */
    if ((cap1 & MEC_BIT(MEC_ESPI_CAP1_ALERT_OD_CAP_POS)) != 0) {
        cv |= MEC_BIT(MEC_ESPI_CFG_ALERT_OD_SUPP_POS);
    }

    /* Peripheral channel */
    if ((cap0 & MEC_BIT(MEC_ESPI_CAP0_PC_SUPP_POS)) != 0) {
        cv |= MEC_BIT(MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS);
    }

    cv |= get_pc_max_pld_size(iobase);

    /* VW channel */
    if ((cap0 & MEC_BIT(MEC_ESPI_CAP0_VW_SUPP_POS)) != 0) {
        cv |= MEC_BIT(MEC_ESPI_CFG_VW_CHAN_SUP_POS);
    }

    cv |= get_vw_groups_max_cnt(iobase);

    /* OOB channel */
    if ((cap0 & MEC_BIT(MEC_ESPI_CAP0_OOB_SUPP_POS)) != 0) {
        cv |= MEC_BIT(MEC_ESPI_CFG_OOB_CHAN_SUP_POS);
    }

    cv |= get_oob_pld_size(iobase);

    /* Flash channel */
    if ((cap0 & MEC_BIT(MEC_ESPI_CAP0_FC_SUPP_POS)) != 0) {
        cv |= MEC_BIT(MEC_ESPI_CFG_FLASH_CHAN_SUP_POS);
    }

    cv |= get_fc_pld_size(iobase);
    cv |= get_fc_shared_mode(iobase);
    cv |= get_fc_taf_max_rdsz(iobase);

    *cfg = cv;

    return MEC_RET_OK;
}

static void set_espi_global_cap(uintptr_t iobase, uint32_t cfg)
{
    uint32_t v = 0;
    uint8_t host_reset_sel = 0;
    uint8_t cap = 0;
    uint8_t msk = MEC_ESPI_CAP0_SUPP_MSK;

    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_SUPP_PC_POS)) {
        cap |= MEC_BIT(MEC_ESPI_CAP0_PC_SUPP_POS);
    }
    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_SUPP_VW_POS)) {
        cap |= MEC_BIT(MEC_ESPI_CAP0_VW_SUPP_POS);
    }
    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_SUPP_OOB_POS)) {
        cap |= MEC_BIT(MEC_ESPI_CAP0_OOB_SUPP_POS);
    }
    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_SUPP_FLASH_POS)) {
        cap |= MEC_BIT(MEC_ESPI_CAP0_FC_SUPP_POS);
    }

    mmcr8_update_field(iobase + MEC_ESPI_CAP0_OFS, cap, msk);

    v = (cfg & MEC_ESPI_CAP_GL_MAX_FREQ_MSK) >> MEC_ESPI_CAP_GL_MAX_FREQ_POS;
    cap = (uint8_t)MEC_ESPI_CAP1_MAX_FREQ_SET(v);

    v = ((cfg & MEC_ESPI_CAP_GL_IOM_MSK) >> MEC_ESPI_CAP_GL_IOM_POS);
    cap |= (uint8_t)MEC_ESPI_CAP1_IOM_SET(v);
    
    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_SUPP_ALERT_OD_POS)) {
        cap |= MEC_BIT(MEC_ESPI_CAP1_ALERT_OD_CAP_POS);
    }

    msk = (MEC_ESPI_CAP1_MAX_FREQ_MSK | MEC_ESPI_CAP1_IOM_MSK |
           MEC_BIT(MEC_ESPI_CAP1_ALERT_OD_CAP_POS));

    mmcr8_update_field(iobase + MEC_ESPI_CAP1_OFS, cap, msk);

    if (cfg & MEC_BIT(MEC_ESPI_CAP_GL_PLTRST_EXT_POS)) {
        mmcr8_set_bit(iobase + MEC_ESPI_PLTRST_SRC_OFS, MEC_ESPI_PLTRST_SRC_POS);
    } else {
        mmcr8_clr_bit(iobase + MEC_ESPI_PLTRST_SRC_OFS, MEC_ESPI_PLTRST_SRC_POS);
        host_reset_sel = MEC_PCR_PLATFORM_RST_IS_ESPI_PLTRST;
    }

    mec_hal_pcr_host_reset_select(host_reset_sel);
}

static uint32_t get_espi_global_cap(uintptr_t iobase)
{
    uint32_t hwval = mmcr8_rd(iobase + MEC_ESPI_CAP0_OFS);
    uint32_t cfg = 0;
    uint32_t v = 0;

    if ((hwval & MEC_BIT(MEC_ESPI_CAP0_PC_SUPP_POS)) != 0) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_SUPP_PC_POS);
    }
    if ((hwval & MEC_BIT(MEC_ESPI_CAP0_VW_SUPP_POS)) != 0) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_SUPP_VW_POS);
    }
    if ((hwval & MEC_BIT(MEC_ESPI_CAP0_OOB_SUPP_POS)) != 0) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_SUPP_OOB_POS);
    }
    if ((hwval & MEC_BIT(MEC_ESPI_CAP0_FC_SUPP_POS)) != 0) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_SUPP_FLASH_POS);
    }

    hwval = mmcr8_rd(iobase + MEC_ESPI_CAP1_OFS);
    v = MEC_ESPI_CAP1_MAX_FREQ_GET(hwval);
    cfg |= ((v << MEC_ESPI_CAP_GL_MAX_FREQ_POS) & MEC_ESPI_CAP_GL_MAX_FREQ_MSK);
    v = MEC_ESPI_CAP1_IOM_GET(hwval);
    cfg |= ((v << MEC_ESPI_CAP_GL_IOM_POS) & MEC_ESPI_CAP_GL_IOM_MSK);

    if (hwval & MEC_BIT(MEC_ESPI_CAP1_ALERT_OD_CAP_POS)) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_SUPP_ALERT_OD_POS);
    }

    if (mmcr8_test_bit(iobase + MEC_ESPI_PLTRST_SRC_OFS, MEC_ESPI_PLTRST_SRC_POS) != 0) {
        cfg |= MEC_BIT(MEC_ESPI_CAP_GL_PLTRST_EXT_POS);
    }

    return cfg;
}

static void set_espi_pc_cap(uintptr_t iobase, uint32_t cfg)
{
    uint32_t v = ((cfg & MEC_ESPI_CAP_PC_MAX_PLD_SIZE_MSK) >> MEC_ESPI_CAP_PC_MAX_PLD_SIZE_POS);

    v = MEC_ESPI_CAP_PC_MPLS_SET(v);
    mmcr8_update_field(iobase + MEC_ESPI_CAP_PC_OFS, v, MEC_ESPI_CAP_PC_MPLS_MSK);
}

static uint32_t get_espi_pc_cap(uintptr_t iobase)
{
    uint32_t v = mmcr8_rd(iobase + MEC_ESPI_CAP_PC_OFS);
    
    v = MEC_ESPI_CAP_PC_MPLS_GET(v);

    return ((v << MEC_ESPI_CAP_PC_MAX_PLD_SIZE_POS) & MEC_ESPI_CAP_PC_MAX_PLD_SIZE_MSK);
}

static void set_espi_vw_cap(uintptr_t iobase, uint32_t cfg)
{
    uint32_t v = (cfg & MEC_ESPI_CAP_VW_MAX_VW_GRP_CNT_MSK) >> MEC_ESPI_CAP_VW_MAX_VW_GRP_CNT_POS;

    v = MEC_ESPI_CAP_VW_MAX_GRPS_SET(v);
    mmcr8_update_field(iobase + MEC_ESPI_CAP_VW_OFS, v, MEC_ESPI_CAP_VW_MAX_GRPS_MSK);
}

static uint32_t get_espi_vw_cap(uintptr_t iobase)
{
    uint32_t v = mmcr8_rd(iobase + MEC_ESPI_CAP_VW_OFS);

    v = MEC_ESPI_CAP_VW_MAX_GRPS_GET(v);
    
    return ((v << MEC_ESPI_CAP_VW_MAX_VW_GRP_CNT_POS) & MEC_ESPI_CAP_VW_MAX_VW_GRP_CNT_MSK);
}

static void set_espi_oob_cap(uintptr_t iobase, uint32_t cfg)
{
    uint32_t v = (cfg & MEC_ESPI_CAP_OOB_MAX_PLD_SIZE_MSK) >> MEC_ESPI_CAP_OOB_MAX_PLD_SIZE_POS;

    v = MEC_ESPI_CAP_OOB_MPLD_SET(v);

    mmcr8_update_field(iobase + MEC_ESPI_CAP_OOB_OFS, v, MEC_ESPI_CAP_OOB_MPLD_MSK);
}

static uint32_t get_espi_oob_cap(uintptr_t iobase)
{
    uint32_t v = mmcr8_rd(iobase + MEC_ESPI_CAP_OOB_OFS);

    v = MEC_ESPI_CAP_OOB_MPLD_GET(v);

    return ((v << MEC_ESPI_CAP_OOB_MAX_PLD_SIZE_POS) & MEC_ESPI_CAP_OOB_MAX_PLD_SIZE_MSK);
}

static void set_espi_fc_cap(uintptr_t iobase, uint32_t cfg)
{
    uint32_t capfc = 0;
    uint32_t v = (cfg & MEC_ESPI_CAP_FC_TAF_ERBSZ_MSK) >> MEC_ESPI_CAP_FC_TAF_ERBSZ_POS;

    mmcr8_wr((uint8_t)(v & 0xffu), iobase + MEC_ESPI_TAF_EBSZ_OFS);

    v = (cfg & MEC_ESPI_CAP_FC_MAX_PLD_SIZE_MSK) >> MEC_ESPI_CAP_FC_MAX_PLD_SIZE_POS;
    capfc = MEC_ESPI_CAP_FC_MPLD_SET(v);
    capfc |= fc_sharing_hw(cfg);

    mmcr8_wr(capfc, iobase + MEC_ESPI_CAP_FC_OFS);
}

static uint32_t get_espi_fc_cap(uintptr_t iobase)
{
    uint32_t capfc = mmcr8_rd(iobase + MEC_ESPI_CAP_FC_OFS);
    uint32_t tafersz = mmcr8_rd(iobase + MEC_ESPI_TAF_EBSZ_OFS);
    uint32_t cfg = fc_sharing_get(capfc);
    uint32_t v = MEC_ESPI_CAP_FC_MPLD_GET(capfc);
    
    cfg |= ((v << MEC_ESPI_CAP_FC_MAX_PLD_SIZE_POS) & MEC_ESPI_CAP_FC_MAX_PLD_SIZE_MSK);
    v = MEC_ESPI_CAP_FC_TAF_MRRQ_GET(capfc);
    cfg |= ((v << MEC_ESPI_CAP_FC_TAF_MAX_RDREQ_SIZE_POS) & MEC_ESPI_CAP_FC_TAF_MAX_RDREQ_SIZE_MSK);
    cfg |= (tafersz << MEC_ESPI_CAP_FC_TAF_ERBSZ_POS);

    return cfg;
}

int mec_hal_espi_cap_set(uintptr_t iobase, enum mec_espi_cap_id id, uint32_t cfg)
{
    if (iobase == 0) {
        return MEC_RET_ERR_INVAL;
    }

    switch (id) {
    case MEC_ESPI_CAP_ID_GLOBAL:
        set_espi_global_cap(iobase, cfg);
        break;
    case MEC_ESPI_CAP_ID_PC:
        set_espi_pc_cap(iobase, cfg);
        break;
    case MEC_ESPI_CAP_ID_VW:
        set_espi_vw_cap(iobase, cfg);
        break;
    case MEC_ESPI_CAP_ID_OOB:
        set_espi_oob_cap(iobase, cfg);
        break;
    case MEC_ESPI_CAP_ID_FC:
        set_espi_fc_cap(iobase, cfg);
        break;
    default:
        return MEC_RET_ERR_INVAL;
    }

    return MEC_RET_OK;
}

uint32_t mec_hal_espi_cap_get(uintptr_t iobase, enum mec_espi_cap_id id)
{
    uint32_t cap = 0u;

    switch (id) {
    case MEC_ESPI_CAP_ID_GLOBAL:
        cap = get_espi_global_cap(iobase);
        break;
    case MEC_ESPI_CAP_ID_PC:
        cap = get_espi_pc_cap(iobase);
        break;
    case MEC_ESPI_CAP_ID_VW:
        cap = get_espi_vw_cap(iobase);
        break;
    case MEC_ESPI_CAP_ID_OOB:
        cap = get_espi_oob_cap(iobase);
        break;
    case MEC_ESPI_CAP_ID_FC:
        cap = get_espi_fc_cap(iobase);
        break;
    default:
        break;
    }

    return cap;
}

/* end mec_espi.c */
