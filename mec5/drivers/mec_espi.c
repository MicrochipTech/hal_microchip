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

/* ---- eSPI Reset interrupt ---- */
#define MEC_ESPI_RESET_ECIA_INFO MEC5_ECIA_INFO(19, 7, 11, 110)

static void set_supported_channels(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t mask = (ESPI_IO_CAP0_PC_SUPP_Msk | ESPI_IO_CAP0_VW_SUPP_Msk
                     | ESPI_IO_CAP0_OOB_SUPP_Msk | ESPI_IO_CAP0_FC_SUPP_Msk);
    uint32_t temp = 0;

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS)) {
        temp |= MEC_BIT(ESPI_IO_CAP0_PC_SUPP_Pos);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_VW_CHAN_SUP_POS)) {
        temp |= MEC_BIT(ESPI_IO_CAP0_VW_SUPP_Pos);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_OOB_CHAN_SUP_POS)) {
        temp |= MEC_BIT(ESPI_IO_CAP0_OOB_SUPP_Pos);
    }

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_FLASH_CHAN_SUP_POS)) {
        temp |= MEC_BIT(ESPI_IO_CAP0_FC_SUPP_Pos);
    }

    iobase->CAP0 = (iobase->CAP0 & ~mask) | temp;
}

static void set_supported_max_freq(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t mask = ESPI_IO_CAP1_MAX_FREQ_SUPP_Msk;
    uint32_t temp = capabilities & MEC_ESPI_CFG_MAX_SUPP_FREQ_MSK;

    temp >>= MEC_ESPI_CFG_MAX_SUPP_FREQ_POS;
    temp = (temp << ESPI_IO_CAP1_MAX_FREQ_SUPP_Pos) & ESPI_IO_CAP1_MAX_FREQ_SUPP_Msk;

    iobase->CAP1 = (iobase->CAP1 & ~mask) | temp;
}

static void set_supported_io_modes(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = iobase->CAP1 & ~(ESPI_IO_CAP1_IO_MODE_SUPP_Msk);

    temp |= ((((capabilities >> MEC_ESPI_CFG_IO_MODE_SUPP_POS)
             & MEC_ESPI_CFG_IO_MODE_SUPP_MSK0) << ESPI_IO_CAP1_IO_MODE_SUPP_Pos)
             & ESPI_IO_CAP1_IO_MODE_SUPP_Msk);

    iobase->CAP1 = (uint8_t)(temp & 0xffu);
}

static void set_supported_alert_io_pin_mode(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = iobase->CAP1 & ~(ESPI_IO_CAP1_ALERT_OD_SUPP_Msk);

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_ALERT_OD_SUPP_POS)) {
        temp |= MEC_BIT(ESPI_IO_CAP1_ALERT_OD_SUPP_Pos);
    }

    iobase->CAP1 = (uint8_t)(temp & 0xffu);
}

static void set_pc_capabilities(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = iobase->CAPPC & ~(ESPI_IO_CAPPC_PC_MAX_PLD_Msk);

    temp |= ((((capabilities >> MEC_ESPI_CFG_PC_MAX_PLD_SZ_POS)
             & MEC_ESPI_CFG_PC_MAX_PLD_SZ_MSK0) << ESPI_IO_CAPPC_PC_MAX_PLD_Pos)
             & ESPI_IO_CAPPC_PC_MAX_PLD_Msk);

    iobase->CAPPC = (uint8_t)(temp & 0xffu);
}

static void set_vw_capabilities(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = iobase->CAPVW & ~(ESPI_IO_CAPVW_MAX_VW_CNT_Msk);

    temp |= ((((capabilities >> MEC_ESPI_CFG_VW_CNT_MAX_POS)
             & MEC_ESPI_CFG_VW_CNT_MAX_MSK0) << ESPI_IO_CAPVW_MAX_VW_CNT_Pos)
             & ESPI_IO_CAPVW_MAX_VW_CNT_Msk);

    iobase->CAPVW = (uint8_t)(temp & 0xffu);
}

static void set_oob_capabilities(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = iobase->CAPOOB & ~(ESPI_IO_CAPOOB_MAX_PLD_SIZE_Msk);

    temp |= ((((capabilities >> MEC_ESPI_CFG_OOB_MAX_PLD_SZ_POS)
             & MEC_ESPI_CFG_OOB_MAX_PLD_SZ_MSK0) << ESPI_IO_CAPOOB_MAX_PLD_SIZE_Pos)
             & ESPI_IO_CAPOOB_MAX_PLD_SIZE_Msk);

    iobase->CAPOOB = (uint8_t)(temp & 0xffu);
}

static void set_fc_max_pld(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = ((capabilities >> MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_POS)
                     & MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK0);
    uint8_t msk = ESPI_IO_CAPFC_MAX_PLD_SIZE_Msk;
    uint8_t regval = (uint8_t)((temp << ESPI_IO_CAPFC_MAX_PLD_SIZE_Pos) & ESPI_IO_CAPFC_MAX_PLD_SIZE_Msk);

    iobase->CAPFC = (iobase->CAPFC & ~msk) | regval;
}

static void set_fc_shared_mode(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint8_t msk = ESPI_IO_CAPFC_SHARING_SUPP_Msk;
    uint8_t temp = (capabilities >> MEC_ESPI_CFG_FLASH_SHARED_MODE_POS) & MEC_ESPI_CFG_FLASH_SHARED_MODE_MSK0;
    uint8_t regval = (uint8_t)((temp << ESPI_IO_CAPFC_SHARING_SUPP_Pos) & ESPI_IO_CAPFC_SHARING_SUPP_Msk);

    iobase->CAPFC = (iobase->CAPFC & ~msk) | regval;
}

static void set_fc_capabilities(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint32_t temp = ((capabilities >> MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_POS)
                     & MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK0);
    uint8_t msk = (ESPI_IO_CAPFC_MAX_PLD_SIZE_Msk | ESPI_IO_CAPFC_SHARING_SUPP_Msk
                   | ESPI_IO_CAPFC_TAF_MAX_READ_SIZE_Msk);
    uint8_t regval = (uint8_t)((temp << ESPI_IO_CAPFC_MAX_PLD_SIZE_Pos) & ESPI_IO_CAPFC_MAX_PLD_SIZE_Msk);

    temp = (capabilities >> MEC_ESPI_CFG_FLASH_SHARED_MODE_POS) & MEC_ESPI_CFG_FLASH_SHARED_MODE_MSK0;
    regval |= (uint8_t)((temp << ESPI_IO_CAPFC_SHARING_SUPP_Pos) & ESPI_IO_CAPFC_SHARING_SUPP_Msk);

    iobase->CAPFC = (iobase->CAPFC & ~msk) | regval;
}

/* If Platform Reset is peformed is different way than eSPI PLTRST# virtual wire
 * we set a bit so our eSPI controller will ignore PLTRST# VWire.
 */
static void set_pltrst_source(struct espi_io_regs *iobase, uint32_t capabilities)
{
    uint8_t host_reset_sel = 0;

    if (capabilities & MEC_BIT(MEC_ESPI_CFG_PLTRST_EXT_POS)) {
        iobase->PLTRST_SRC |= MEC_BIT(ESPI_IO_PLTRST_SRC_SEL_Pos);
    } else { /* use PLTRST# virtual wire */
        iobase->PLTRST_SRC &= (uint8_t)~MEC_BIT(ESPI_IO_PLTRST_SRC_SEL_Pos);
        host_reset_sel = MEC_PCR_PLATFORM_RST_IS_ESPI_PLTRST;
    }

    mec_pcr_host_reset_select(host_reset_sel);
}

/* ---- Public API ---- */
void mec_espi_reset_change_clr(struct espi_io_regs *iobase)
{
    iobase->ERIS = MEC_BIT(ESPI_IO_ERIS_CHG_Pos);
    mec_girq_clr_src(MEC_ESPI_RESET_ECIA_INFO);
}

/* Return bits indicating ESPI_RESET# has changed and its current state */
uint32_t mec_espi_reset_state(struct espi_io_regs *iobase)
{
    return iobase->ERIS & (ESPI_IO_ERIS_CHG_Msk | ESPI_IO_ERIS_STATE_Msk);
}

void mec_espi_reset_change_intr_en(struct espi_io_regs *iobase, uint8_t enable)
{
    if (enable) {
        iobase->ERIE |= MEC_BIT(ESPI_IO_ERIE_CHG_INTR_Pos);
    } else {
        iobase->ERIE &= (uint8_t)~MEC_BIT(ESPI_IO_ERIE_CHG_INTR_Pos);
    }
}

/* ESPI_RESET edge interrupt ECIA control */
void mec_espi_reset_girq_ctrl(uint8_t enable)
{
    mec_girq_ctrl(MEC_ESPI_RESET_ECIA_INFO, (int)enable);
}

void mec_espi_reset_girq_status_clr(void)
{
    mec_girq_clr_src(MEC_ESPI_RESET_ECIA_INFO);
}

uint32_t mec_espi_reset_girq_status(void)
{
    return mec_girq_src(MEC_ESPI_RESET_ECIA_INFO);
}

uint32_t mec_espi_reset_girq_result(void)
{
    return mec_girq_result(MEC_ESPI_RESET_ECIA_INFO);
}

/* NOTE eSPI is only fully reset by a full chip reset or power cycle.
 * The external ESPI_RESET# signal when asserted does hold portions of the logic
 * in reset state. Please refer to the Microchip eSPI block document.
 */
int mec_espi_init(struct espi_config *cfg)
{
    if (!cfg) {
        return MEC_RET_ERR_INVAL;
    }

    struct espi_io_regs *iobase = cfg->iobase;
    uint32_t girq_en = 0u;

    set_supported_channels(iobase, cfg->capabilities);
    set_supported_max_freq(iobase, cfg->capabilities);
    set_supported_io_modes(iobase, cfg->capabilities);
    set_supported_alert_io_pin_mode(iobase, cfg->capabilities);
    set_pc_capabilities(iobase, cfg->capabilities);
    set_vw_capabilities(iobase, cfg->capabilities);
    set_oob_capabilities(iobase, cfg->capabilities);
    set_fc_capabilities(iobase, cfg->capabilities);
    set_pltrst_source(iobase, cfg->capabilities);

    mec_espi_reset_change_intr_en(iobase, 0);
    mec_espi_reset_change_clr(iobase);

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_VW_CT_GIRQ_EN_POS)) {
        ECIA0->GIRQ[GIRQS_IDX_GIRQ24].EN_SET = 0x0fffffffu;
        ECIA0->GIRQ[GIRQS_IDX_GIRQ25].EN_SET = 0x0000ffffu;
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_PC_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(0);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_BM1_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(1);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_BM2_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(2);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_LTR_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(3);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_OOB_UP_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(4);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_OOB_DN_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(5);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_FC_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(6);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_VW_CHEN_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(8);
    }

    if (cfg->cfg_flags & MEC_BIT(MEC_ESPI_CFG_FLAG_ERST_GIRQ_EN_POS)) {
        girq_en |= MEC_BIT(7);
    }

    ECIA0->GIRQ[GIRQS_IDX_GIRQ19].EN_SET = girq_en;

    return 0;
}

/* Enable eSPI controller after all static configuration has been performed.
 * MEC eSPI activate must be set before the Host de-asserts ESPI_RESET#.
 */
void mec_espi_activate(struct espi_io_regs *iobase, uint8_t enable)
{
    if (enable) {
        iobase->ACTV |= MEC_BIT(ESPI_IO_ACTV_EN_Pos);
    } else {
        iobase->ACTV &= ~MEC_BIT(ESPI_IO_ACTV_EN_Pos);
    }
}

int mec_espi_is_activated(struct espi_io_regs *iobase)
{
    if (iobase) {
        if (iobase->ACTV & MEC_BIT(ESPI_IO_ACTV_EN_Pos)) {
            return 1;
        }
    }

    return 0;
}

int mec_espi_capability_set(struct espi_io_regs *iobase,
                            enum mec_espi_global_cap cap, uint32_t cfg)
{
    if (!iobase) {
        return MEC_RET_ERR_INVAL;
    }

    switch (cap) {
    case MEC_ESPI_CAP_MAX_FREQ:
        set_supported_max_freq(iobase, cfg);
        break;
    case MEC_ESPI_CAP_IO_MODE:
        set_supported_io_modes(iobase, cfg);
        break;
    case MEC_ESPI_CAP_ALERT_OD:
        set_supported_alert_io_pin_mode(iobase, cfg);
        break;
    case MEC_ESPI_CAP_PERIPH_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS)) {
            iobase->CAP0 |= MEC_BIT(ESPI_IO_CAP0_PC_SUPP_Pos);
        } else {
            iobase->CAP0 &= (uint8_t)~MEC_BIT(ESPI_IO_CAP0_PC_SUPP_Pos);
        }
        break;
    case MEC_ESPI_CAP_PC_MAX_PLD_SIZE:
        set_pc_capabilities(iobase, cfg);
        break;
    case  MEC_ESPI_CAP_VWIRE_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_VW_CHAN_SUP_POS)) {
            iobase->CAP0 |= MEC_BIT(ESPI_IO_CAP0_VW_SUPP_Pos);
        } else {
            iobase->CAP0 &= (uint8_t)~MEC_BIT(ESPI_IO_CAP0_VW_SUPP_Pos);
        }
        break;
    case MEC_ESPI_CAP_MAX_VW_COUNT:
        set_vw_capabilities(iobase, cfg);
        break;
    case MEC_ESPI_CAP_OOB_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_OOB_CHAN_SUP_POS)) {
            iobase->CAP0 |= MEC_BIT(ESPI_IO_CAP0_OOB_SUPP_Pos);
        } else {
            iobase->CAP0 &= (uint8_t)~MEC_BIT(ESPI_IO_CAP0_OOB_SUPP_Pos);
        }
        break;
    case MEC_ESPI_CAP_OOB_MAX_PLD_SIZE:
        set_oob_capabilities(iobase, cfg);
        break;
    case MEC_ESPI_CAP_FLASH_CHAN:
        if (cfg & MEC_BIT(MEC_ESPI_CFG_FLASH_CHAN_SUP_POS)) {
            iobase->CAP0 |= MEC_BIT(ESPI_IO_CAP0_FC_SUPP_Pos);
        } else {
            iobase->CAP0 &= (uint8_t)~MEC_BIT(ESPI_IO_CAP0_FC_SUPP_Pos);
        }
        break;
    case MEC_ESPI_CAP_FC_MAX_PLD_SIZE:
        set_fc_max_pld(iobase, cfg);
        break;
    case MEC_ESPI_CAP_FC_SHARING:
        set_fc_shared_mode(iobase, cfg);
        break;
    default:
        return MEC_RET_ERR_INVAL;
    }

    return MEC_RET_OK;
}

/* end mec_espi.c */
