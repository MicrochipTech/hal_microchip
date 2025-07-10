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
#include "mec_i2c_api.h"
#include "mec_pcr_api.h"
#include "mec_retval.h"

/* MEC5 I2C controller notes:
 * The I2C byte mode Control(WO) and Status(RO) register are both located at offset 0.
 * I2C.STATUS register is an 8-bit read-only status register:
 * bit[0] = Not Bus Busy(NBB) 0 indicates the bus is owned by this controller.
 * bit[1] = Lost Arbitration(LAB) 1 indicates controller lost arbitration to another
 *          controller on the bus. LAB detection is performed during data transfer,
 *          ackowledgement, START, and Repeated-START phases of the transfer. LAB
 *          is not checked during a STOP phase. If LAB occurs this controller will tri-state
 *          its pins and continue to clock in the address/data from the external controller.
 *          On the 9th clock NIPEND and LAB will both assert.
 *          I2C-NL CM FSM will transition to IDLE state after 9th clock (n)ACK bit and
 *          clear the PIN bit to 1(de-asserted).
 * bit[2] = AAS = 1 indicates an external Controller issued (RPT)-START + target address
 *          the target address matches one of the two target addresses in this controller's
 *          own address register or the I2C generate call address(0x00). NOTE: general
 *          call address detction can be enabled/disabled in the Configuration register.
 * bit[3] = LRB_AD0 value interpretation depends upon the AAS status bit.
 *          AAS==0: LRB is the last received bit on the bus. Usually this is the value
 *          of SDA on the 9th I2C clock (0=ACK, 1=NACK).
 *          AAS==1: AD0=1 if received target address was the I2C GC(0x00) address else
 *          0 if the target address matched one of the controller's own addresses.
 * bit[4] = BER 1 = Controller detected a bus error. Assertion of BER cause the controller
 *          to de-assert NBB and NPIPEND: NBB=1 and NIPEND=1.
 * bit[5] = STS 1 = externally generated STOP was detected. BROKEN in MEC520x. A HW fix
 *          implemented in MEC540x.
 * bit[6] = SAD 1 = SMBus address decode asserted if enabled in the Configuration register.
 * bit[7] = NIPEND 1 = No Interrupt Pending(de-asserted). 1 = Interrupt is Pending(asserted).
 *          NIPEND is asserted(0) after the 9th I2C clock or on detection of a bus error.
 *          TX direction: NIPEND de-asserted when I2C.Data register is written.
 *          RX direction: NIPEND de-asserted when I2C.Data is read.
 *          NOTE: In Controller Mode reading I2C.Data when direction is RX and I2C.CTRL.STO=0
 *          returns current data byte and generates clocks for the next byte (read-ahead).
 *          HW determines direction based on the W/nR bit which is bit[0] of the I2C target
 *          address.
 *
 * I2C.CTRL register is an 8-bit write-only control register:
 * bit[0] = ACK = 1 HW will generate an ACK on the 9th after receiving a
 * data byte from an external Controller.
 * bit[1] = STO = 1 generate a STOP
 * bit[2] = STA = 1 generate a START and transmit the address in the I2C.DATA register.
 * bit[3] = ENI = 1 Assert the I2C controller's interrupt active signal to the corresponding
 *          GIRQ Source bit. Interrupt signal asserted when I2C.STATUS.NIPEND -> 0.
 * bits[5:4] = reserved
 * bit[6] = ESO = 1 Enable SDA outout. 0 = SDA disabled.
 * bit[7] = NIPEND_RST = Not Pending Interrupt. A software reset de-asserting all status
 *          except the Not Bus Busy status bit. WARNING: Behaviour of this bit
 *          depends upon the I2C FSM state.
 *
 * Controller clears I2C.CTRL.STA write-only bit after 7th clock of I2C address.
 * Controller clears I2C.CTRL.STO write-only bit when it drives SDA low at
 * the beginning of generating the STOP sequence on the bus. This occurs
 * a minimum 1/2 I2C clock period after the 9th clock pulse of the previous
 * data on the bus.
 */
#define MEC_I2C_SMB_COMPL_STS_RW1C_MSK    0xe1397f00u
#define MEC_I2C_SMB_COMPL_STS_RO_MSK      0x02020040u
#define MEC_I2C_SMB_COMPL_EN_RW_MSK       0x3cu
#define MEC_I2C_SMB_COMPL_TM_STS_RW1C_MSK 0x80390000u
#define MEC_I2C_SMB_COMPL_TM_STS_ALL_MSK  0x803b0000u

#define MEC_I2C_SMB0_ECIA_INFO MEC_ECIA_INFO(13, 0, 5, 20)
#define MEC_I2C_SMB1_ECIA_INFO MEC_ECIA_INFO(13, 1, 5, 21)
#define MEC_I2C_SMB2_ECIA_INFO MEC_ECIA_INFO(13, 2, 5, 22)
#define MEC_I2C_SMB3_ECIA_INFO MEC_ECIA_INFO(13, 3, 5, 23)
#define MEC_I2C_SMB4_ECIA_INFO MEC_ECIA_INFO(13, 4, 5, 158)

/* #define MEC_I2C_NL_DEBUG_SAVE_CM_CMD */

struct mec_i2c_info {
    uintptr_t base_addr;
    uint32_t devi;
    uint16_t pcr_id;
};

static const struct mec_i2c_freq_cfg freq_cfg_dflt[MEC_I2C_STD_FREQ_MAX] = {
    [MEC_I2C_STD_FREQ_100K] = {
        .freqhz = 100000u,
        .idle_scaling = 0x01fc01edu,
        .timeout_scaling = 0x4b9cc2c7u,
        .data_timing = 0x0c4d5006u,
        .bus_clk = 0x4f4fu,
        .rpt_start_hold_time = 0x4du,
    },
    [MEC_I2C_STD_FREQ_400K] = {
        .freqhz = 400000u,
        .idle_scaling = 0x01000050u,
        .timeout_scaling = 0x159cc2c7u,
        .data_timing = 0x040a0a06u,
        .bus_clk = 0xf17u,
        .rpt_start_hold_time = 0x0au,
    },
    [MEC_I2C_STD_FREQ_1M] = {
        .freqhz = 1000000u,
        .idle_scaling = 0x10000050u,
        .timeout_scaling = 0x089cc2c7u,
        .data_timing = 0x04060601u,
        .bus_clk = 0x509u,
        .rpt_start_hold_time = 0x06u,
    },
};

static const struct mec_i2c_info i2c_instances[MEC5_I2C_SMB_INSTANCES] = {
    {MEC_I2C_SMB0_BASE, MEC_I2C_SMB0_ECIA_INFO, MEC_PCR_I2C_SMB0 },
    {MEC_I2C_SMB1_BASE, MEC_I2C_SMB1_ECIA_INFO, MEC_PCR_I2C_SMB1 },
    {MEC_I2C_SMB2_BASE, MEC_I2C_SMB2_ECIA_INFO, MEC_PCR_I2C_SMB2 },
    {MEC_I2C_SMB3_BASE, MEC_I2C_SMB3_ECIA_INFO, MEC_PCR_I2C_SMB3 },
    {MEC_I2C_SMB4_BASE, MEC_I2C_SMB4_ECIA_INFO, MEC_PCR_I2C_SMB4 },
};

static struct mec_i2c_info const *get_i2c_smb_info(uintptr_t regbase)
{
    for (int i = 0; i < MEC5_I2C_SMB_INSTANCES; i++) {
        const struct mec_i2c_info *p = &i2c_instances[i];

        if (p->base_addr == regbase) {
            return p;
        }
    }

    return NULL;
}

int mec_hal_i2c_smb_reset(struct mec_i2c_smb_ctx *ctx)
{
    if (!ctx) {
        return MEC_RET_ERR_INVAL;
    }

    const struct mec_i2c_info *info = get_i2c_smb_info(ctx->regbase);

    if (!info) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_pcr_blk_reset(info->pcr_id);

    return MEC_RET_OK;
}

static void i2c_timing(uintptr_t regbase, const struct mec_i2c_freq_cfg *freq_cfg)
{
    mmcr32_wr(freq_cfg->bus_clk, regbase + MEC_I2C_BCLK_OFS);
    mmcr32_wr(freq_cfg->rpt_start_hold_time, regbase + MEC_I2C_RSHT_OFS);
    mmcr32_wr(freq_cfg->data_timing, regbase + MEC_I2C_DT_OFS);
    mmcr32_wr(freq_cfg->idle_scaling, regbase + MEC_I2C_ISC_OFS);
    mmcr32_wr(freq_cfg->timeout_scaling, regbase + MEC_I2C_TOSC_OFS);
}

static void i2c_config(struct mec_i2c_smb_ctx *ctx, struct mec_i2c_smb_cfg *config,
                       struct mec_i2c_freq_cfg *custom_freq_cfg)
{
    uintptr_t rb = ctx->regbase;
    uint32_t v = 0;
    uint8_t control = 0;

    /* disable, set port MUX, and enable digitial filter */
    v = MEC_I2C_CFG_PORT_SET(config->port) | MEC_BIT(MEC_I2C_CFG_FEN_POS);
    mmcr32_wr(v, rb + MEC_I2C_CFG_OFS);

    v = MEC_I2C_OA_1_SET(config->target_addr1) | MEC_I2C_OA_2_SET(config->target_addr2);
    mmcr32_wr(v, rb + MEC_I2C_OA_OFS);

    if (config->cfg_flags & MEC_I2C_SMB_CFG_CUST_FREQ) {
        i2c_timing(rb, custom_freq_cfg);
    } else {
        i2c_timing(rb, &freq_cfg_dflt[config->std_freq]);
    }

    /* clear latched status in completion register */
    mmcr32_wr(MEC_I2C_COMP_RW1C_MSK, rb + MEC_I2C_COMP_OFS);

    /* Clear I2C PIN status, enable output drive and HW ACK generation */
    control = (MEC_BIT(MEC_I2C_CR_PIN_POS) | MEC_BIT(MEC_I2C_SMB_CTRL_PIN_Pos) |
               MEC_BIT(MEC_I2C_SMB_CTRL_ESO_Pos) | MEC_BIT(MEC_I2C_SMB_CTRL_ACK_Pos));
    ctx->i2c_ctrl_cached = control;
    mmcr8_wr(control, rb + MEC_I2C_CR_OFS);

    /* enable I2C controller */
    mmcr32_set_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_ENAB_POS);
    for (int i = 0; i < 8; i++) { /* delay */
        mmcr32_wr(0, rb + MEC_I2C_ELEN_OFS);
    }

    /* clear latched status in completion register */
    mmcr32_wr(MEC_I2C_COMP_RW1C_MSK, rb + MEC_I2C_COMP_OFS);
}

uint8_t mec_hal_i2c_smb_port_get(uintptr_t regbase)
{
    uint32_t v = 0;

    if (regbase == 0) {
        return MEC_I2C_PORT_MAX;
    }

    v = mmcr32_rd(regbase + MEC_I2C_CFG_OFS);
    return (uint8_t)MEC_I2C_CFG_PORT_GET(v);
}

uint8_t mec_hal_i2c_smb_port_set(uintptr_t regbase, uint8_t port)
{
    uint32_t v = 0;

    if ((regbase == 0) || (port >= MEC_I2C_PORT_MAX)) {
        return MEC_I2C_PORT_MAX;
    }

    v = MEC_I2C_CFG_PORT_SET(port);

    mmcr32_update_field(regbase + MEC_I2C_CFG_OFS, v, MEC_I2C_CFG_PORT_MSK);

    return port;
}

int mec_hal_i2c_smb_get_id(uintptr_t regbase, uint8_t *ctrl_id)
{
    if (ctrl_id == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    for (uint8_t i = 0; i < MEC5_I2C_SMB_INSTANCES; i++) {
        if (i2c_instances[i].base_addr == regbase) {
            *ctrl_id = i;
            return MEC_RET_OK;
        }
    }

    return MEC_RET_ERR_INVAL;
}

int mec_hal_i2c_smb_bus_freq_get(uintptr_t regbase, uint32_t *bus_freq_hz)
{
    uint32_t v = 0, lcnt = 0, hcnt = 0;

    if ((regbase == 0) || (bus_freq_hz == 0)) {
        return MEC_RET_ERR_INVAL;
    }

    v = mmcr32_rd(regbase + MEC_I2C_BCLK_OFS);
    lcnt = MEC_I2C_BCLK_LOP_GET(v);
    hcnt = MEC_I2C_BCLK_HIP_GET(v);

    *bus_freq_hz = MEC_I2C_SMB_BAUD_CLK_FREQ_HZ / ((lcnt + 1u) + (hcnt + 1u));

    return MEC_RET_OK;
}

/* I2C-SMB bus clock frequency =
 * Fixed HW BAUD clock freq / ((low_period+1) + (high_period+1))
 */
int mec_hal_i2c_smb_bus_freq_get_by_ctx(struct mec_i2c_smb_ctx *ctx, uint32_t *bus_freq_hz)
{
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }

    return mec_hal_i2c_smb_bus_freq_get(ctx->regbase, bus_freq_hz);
}

int mec_hal_i2c_smb_bus_freq_get_by_id(uint8_t i2c_id, uint32_t *bus_freq_hz)
{
    uintptr_t regbase = 0;

    if (i2c_id >= MEC5_I2C_SMB_INSTANCES) {
        return MEC_RET_ERR_INVAL;
    }

    regbase = MEC_I2C_SMB_BASE(i2c_id);

    return mec_hal_i2c_smb_bus_freq_get(regbase, bus_freq_hz);
}

int mec_hal_i2c_smb_init(struct mec_i2c_smb_ctx *ctx, struct mec_i2c_smb_cfg *config,
                         struct mec_i2c_freq_cfg *custom_freq_cfg)
{
    uint32_t v = 0;

    if (ctx == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    const struct mec_i2c_info *info = get_i2c_smb_info(ctx->regbase);

    if ((info == NULL) || (config == NULL)) {
        return MEC_RET_ERR_INVAL;
    }

    ctx->devi = info->devi;

    if (config->cfg_flags & MEC_I2C_SMB_CFG_PRESERVE_TARGET_ADDRS) {
        v = mmcr32_rd(ctx->regbase + MEC_I2C_OA_OFS);
        config->target_addr1 = (uint8_t)MEC_I2C_OA_1_GET(v);
        config->target_addr2 = (uint8_t)MEC_I2C_OA_2_GET(v);
    }

    mec_hal_pcr_clr_blk_slp_en(info->pcr_id);
    mec_hal_pcr_blk_reset(info->pcr_id);

    if ((MEC_BIT(config->port) & MEC5_I2C_SMB_PORT_MAP) == 0) {
        return MEC_RET_ERR_INVAL;
    }

    if (((config->cfg_flags & MEC_I2C_SMB_CFG_CUST_FREQ) != 0) && (custom_freq_cfg == 0)) {
        return MEC_RET_ERR_INVAL;
    } else if (config->std_freq >= MEC_I2C_STD_FREQ_MAX) {
        return MEC_RET_ERR_INVAL;
    }

    /* configure controller and cache last value written to write-only control register */
    i2c_config(ctx, config, custom_freq_cfg);

    /* clear GIRQ latched status */
    mec_hal_girq_clr_src(ctx->devi);

    return MEC_RET_OK;
}

/* Helper to set/get specified own (target mode) I2C 7-bit address.
 * If curr_addr pointer is non-NULL then we get the current value else
 * set own address to new_addr.
 */
static int mec_hal_i2c_smb_own_addr(struct mec_i2c_smb_ctx *ctx, uint8_t tid,
                                    uint16_t new_addr, uint16_t *curr_addr)
{
    uint32_t v = 0;

    if (ctx == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    const struct mec_i2c_info *info = get_i2c_smb_info(ctx->regbase);

    if (info == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    v = mmcr32_rd(ctx->regbase + MEC_I2C_OA_OFS);

    switch (tid) {
    case MEC_I2C_TARGET_ADDR_0:
        if (curr_addr == NULL) { /* set */
            v &= (uint32_t)~MEC_I2C_OA_1_MSK;
            v |= MEC_I2C_OA_1_SET((uint32_t)new_addr);
            mmcr32_wr(v, ctx->regbase + MEC_I2C_OA_OFS);
        } else { /* get */
            *curr_addr = (uint16_t)MEC_I2C_OA_1_GET(v);
        }
        break;
    case MEC_I2C_TARGET_ADDR_1:
        if (curr_addr == NULL) { /* set */
            v &= (uint32_t)~MEC_I2C_OA_2_MSK;
            v |= MEC_I2C_OA_2_SET((uint32_t)new_addr);
            mmcr32_wr(v, ctx->regbase + MEC_I2C_OA_OFS);
        } else { /* get */
            *curr_addr = (uint16_t)MEC_I2C_OA_2_GET(v);
        }
        break;
    default:
        return MEC_RET_ERR_INVAL;
    }

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_get_target_addr(struct mec_i2c_smb_ctx *ctx, uint8_t target_id,
                                    uint16_t *target_addr)
{
    if (target_addr == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    return mec_hal_i2c_smb_own_addr(ctx, target_id, 0u, target_addr);
}

int mec_hal_i2c_smb_set_target_addr(struct mec_i2c_smb_ctx *ctx, uint8_t target_id,
                                    uint16_t target_addr)
{
    return mec_hal_i2c_smb_own_addr(ctx, target_id, target_addr, NULL);
}

int mec_hal_i2c_smb_clr_target_addr(struct mec_i2c_smb_ctx *ctx, uint16_t target_addr)
{
    if (ctx == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    uintptr_t rb = ctx->regbase + MEC_I2C_OA_OFS;
    uint32_t oaddr = mmcr32_rd(rb);
    uint32_t taddr = MEC_I2C_OA_1_GET(oaddr);

    if (taddr == (uint32_t)target_addr) {
        mmcr32_clr_bits(rb, MEC_I2C_OA_1_MSK);
    } else {
        taddr = MEC_I2C_OA_2_GET(oaddr);
        if (taddr == (uint32_t)target_addr) {
            mmcr32_clr_bits(rb, MEC_I2C_OA_2_MSK);
        }
    }

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_girq_status_clr(struct mec_i2c_smb_ctx *ctx)
{
    if (ctx == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    mec_hal_girq_clr_src(ctx->devi);

    return MEC_RET_OK;
}

/* Enable/disable I2C controller interrupt signal from propagating to NVIC */
int mec_hal_i2c_smb_girq_ctrl(struct mec_i2c_smb_ctx *ctx, int flags)
{
    if (ctx == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    if (flags & MEC_I2C_SMB_GIRQ_DIS) {
        mec_hal_girq_ctrl(ctx->devi, 0);
    }

    if (flags & MEC_I2C_SMB_GIRQ_CLR_STS) {
        mec_hal_girq_clr_src(ctx->devi);
    }

    if (flags & MEC_I2C_SMB_GIRQ_EN) {
        mec_hal_girq_ctrl(ctx->devi, 1);
    }

    return MEC_RET_OK;
}

uint32_t mec_hal_i2c_smb_get_nvic_id(uint8_t i2c_ctrl_id)
{
    if (i2c_ctrl_id >= MEC5_I2C_SMB_INSTANCES) {
        return UINT32_MAX;
    }

    return MEC_ECIA_INFO_NVIC_DIRECT(i2c_instances[i2c_ctrl_id].devi);
}

int mec_hal_i2c_smb_girq_status(struct mec_i2c_smb_ctx *ctx)
{
    if (ctx == NULL) {
        return 0;
    }

    return (int)mec_hal_girq_src(ctx->devi);
}

int mec_hal_i2c_smb_girq_result(struct mec_i2c_smb_ctx *ctx)
{
    if (ctx == NULL) {
        return 0;
    }

    return (int)mec_hal_girq_result(ctx->devi);
}

/* check I2C.Status Not Busy bit. If set the bus is NOT owned by this controller.
 * Returns 0 if not owned or the parameter check fails.
 *         1 if bus is owned by this controller.
 */
int mec_hal_i2c_smb_is_bus_owned(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif
    uintptr_t rb = ctx->regbase + MEC_I2C_SR_OFS;

    if (mmcr8_test_bit(rb, MEC_I2C_SR_NBB_POS) != 0) {
        return 0;
    }

    return 1;
}

int mec_hal_i2c_smb_ctrl_set(struct mec_i2c_smb_ctx *ctx, uint8_t ctrl)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif
    uintptr_t rb = ctx->regbase + MEC_I2C_CR_OFS;

    ctx->i2c_ctrl_cached = ctrl;
    mmcr8_wr(ctrl, rb);

    return MEC_RET_OK;
}

uint8_t mec_hal_i2c_smb_ctrl_get(struct mec_i2c_smb_ctx *ctx)
{
    if (ctx == NULL) {
        return 0u;
    }

    return ctx->i2c_ctrl_cached;
}

int mec_hal_i2c_cmd_ack_ctrl(struct mec_i2c_smb_ctx *ctx, uint8_t ack_en)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif
    uintptr_t rb = ctx->regbase + MEC_I2C_CR_OFS;
    uint8_t ctrl = ctx->i2c_ctrl_cached;

    if (ack_en != 0) {
        ctrl |= MEC_BIT(MEC_I2C_CR_ACK_POS);
    } else {
        ctrl &= (uint8_t)~MEC_BIT(MEC_I2C_CR_ACK_POS);
    }

    ctx->i2c_ctrl_cached = ctrl;
    mmcr8_wr(ctrl, rb);

    return MEC_RET_OK;
}

/* Re-arm Target mode receive after an external STOP.  */
int mec_hal_i2c_smb_rearm_target_rx(struct mec_i2c_smb_ctx *ctx)
{
    uint8_t ctrl = (MEC_BIT(MEC_I2C_CR_PIN_POS) | MEC_BIT(MEC_I2C_CR_ESO_POS) |
                    MEC_BIT(MEC_I2C_CR_ACK_POS));

    return mec_hal_i2c_smb_ctrl_set(ctx, ctrl);
}

int mec_hal_i2c_smb_auto_ack_enable(struct mec_i2c_smb_ctx *ctx, uint8_t ien)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif
    uintptr_t rb = ctx->regbase + MEC_I2C_CR_OFS;
    uint8_t ctr = MEC_BIT(MEC_I2C_CR_ESO_POS) | MEC_BIT(MEC_I2C_CR_ACK_POS);

    if (ien != 0) {
        ctr |= MEC_BIT(MEC_I2C_CR_ENI_POS);
    }

    ctx->i2c_ctrl_cached = ctr;
    mmcr8_wr(ctr, rb);

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_auto_ack_disable(struct mec_i2c_smb_ctx *ctx, uint8_t ien)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif
    uintptr_t rb = ctx->regbase + MEC_I2C_CR_OFS;
    uint8_t ctr = MEC_BIT(MEC_I2C_CR_ESO_POS);

    if (ien) {
        ctr |= MEC_BIT(MEC_I2C_CR_ENI_POS);
    }

    ctx->i2c_ctrl_cached = ctr;
    mmcr8_wr(ctr, rb);

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_idle_intr_enable(struct mec_i2c_smb_ctx *ctx, uint8_t enable)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase + MEC_I2C_CFG_OFS;

    if (enable != 0) {
        mmcr32_set_bit(rb, MEC_I2C_CFG_IDLE_IEN_POS);
    } else {
        mmcr32_clr_bit(rb, MEC_I2C_CFG_IDLE_IEN_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_intr_ctrl(struct mec_i2c_smb_ctx *ctx, uint32_t mask, uint8_t enable)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint32_t cfg = 0;

    if ((mask & MEC_BIT(MEC_I2C_IEN_BYTE_MODE_POS)) != 0) {
        if (enable) {
            ctx->i2c_ctrl_cached |= MEC_BIT(MEC_I2C_CR_ENI_POS);
        } else {
            ctx->i2c_ctrl_cached &= (uint8_t)~MEC_BIT(MEC_I2C_CR_ENI_POS);
        }
        mmcr8_wr(ctx->i2c_ctrl_cached, rb + MEC_I2C_CR_OFS);
    }

    if (mask & MEC_BIT(MEC_I2C_IEN_IDLE_POS)) {
        cfg |= MEC_BIT(MEC_I2C_CFG_IDLE_IEN_POS);
    }
    if (mask & MEC_BIT(MEC_I2C_NL_IEN_CM_DONE_POS)) {
        cfg |= MEC_BIT(MEC_I2C_CFG_HD_IEN_POS);
    }
    if (mask & MEC_BIT(MEC_I2C_NL_IEN_TM_DONE_POS)) {
        cfg |= MEC_BIT(MEC_I2C_CFG_TD_IEN_POS);
    }
    if (mask & MEC_BIT(MEC_I2C_NL_IEN_AAT_POS)) {
        cfg |= MEC_BIT(MEC_I2C_CFG_AAT_IEN_POS);
    }

    if (enable != 0) {
        mmcr32_set_bits(rb + MEC_I2C_CFG_OFS, cfg);
    } else {
        mmcr32_clr_bits(rb + MEC_I2C_CFG_OFS, cfg);
    }

    return MEC_RET_OK;
}

/* Returns 32-bit unsigned containing combined I2C status
 * b[7:0] = I2C.Status read-only register (byte status)
 * b[31:8] = I2C.Completion status bits.
 * We lose I2C.Completion bit[6] = TIMERR but this bit is
 * a logical OR of bits[12:8].
 * If parameter clear != 0 this routine will clear the R/W1C
 * bits in I2C.Completion after reading them.
 */
uint32_t mec_hal_i2c_smb_status(struct mec_i2c_smb_ctx *ctx, uint8_t clear)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint32_t compl = 0, status = 0;

    /* Read completion first. Not sure if low-level read-only I2C.STATUS has read side-effects. */
    compl = mmcr32_rd(rb + MEC_I2C_COMP_OFS);
    status = mmcr8_rd(rb + MEC_I2C_SR_OFS);

    if (clear != 0) {
        mmcr32_wr(compl, rb + MEC_I2C_COMP_OFS);
    }

    /* completion b[7:0] are control not status bits. Store 8-bit I2C status there */
    compl &= 0xffffff00u;
    compl |= (status & 0xffu);

    return compl;
}

uint32_t mec_hal_i2c_smb_wake_status(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    return mmcr32_rd(ctx->regbase + MEC_I2C_WKSR_OFS);
}

void mec_hal_i2c_smb_wake_status_clr(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return;
    }
#endif

    mmcr32_wr(MEC_BIT(MEC_I2C_WKSR_SB_POS), ctx->regbase + MEC_I2C_WKSR_OFS);
}

int mec_hal_i2c_smb_is_idle_ien(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    uintptr_t rb = ctx->regbase + MEC_I2C_CFG_OFS;

    if (mmcr32_test_bit(rb, MEC_I2C_CFG_IDLE_IEN_POS) != 0) {
        return 1;
    }

    return 0;
}

int mec_hal_i2c_smb_is_idle_intr(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    uintptr_t rb = ctx->regbase;

    if ((mmcr32_test_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_IDLE_IEN_POS) != 0) &&
        (mmcr32_test_bit(rb + MEC_I2C_COMP_OFS, MEC_I2C_COMP_IDLE_POS) != 0)) {
        return 1;
    }

    return 0;
}

/* Return 1 if AAT interrupt is enabled else 0.
 * NOTE we can't condition this with AAT read-only status from the
 * I2C.Status register because that value is dynamic, it can change
 * due to Network layer causing the status to be cleared.
 */
bool mec_hal_i2c_smb_is_aat_ien(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return false;
    }
#endif

    uintptr_t rb = ctx->regbase + MEC_I2C_CFG_OFS;

    if (mmcr32_test_bit(rb, MEC_I2C_CFG_AAT_IEN_POS) != 0) {
        return true;
    }

    return false;
}

int mec_hal_i2c_smb_idle_status_clr(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    mmcr32_set_bit(ctx->regbase + MEC_I2C_COMP_OFS, MEC_I2C_COMP_IDLE_POS);

    return MEC_RET_OK;
}

/* Controller specification sequence:
 * START + addr. Bus is idle (I2C.STATUS.NBB == 1)
 *    write addr to I2C.Data
 *    write 0xC5 to I2C.Control or 0xCb if interrupts are enabled
 * RPT-START + addr. Bus must we owned by this controller (I2C.STATUS.NBB == 0)
 *    write 0x45 or 0x4b to I2C.Control
 *    write addr I2C.Data
 */
int mec_hal_i2c_smb_start_gen(struct mec_i2c_smb_ctx *ctx, uint8_t target_addr, int flags)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint8_t ctr = (MEC_BIT(MEC_I2C_CR_ESO_POS) | MEC_BIT(MEC_I2C_CR_STA_POS)
                   | MEC_BIT(MEC_I2C_CR_ACK_POS));

    if (flags & MEC_I2C_SMB_BYTE_ENI) {
        ctr |= MEC_BIT(MEC_I2C_CR_ENI_POS);
    }

    if (mmcr8_test_bit(rb + MEC_I2C_SR_OFS, MEC_I2C_SR_NBB_POS) != 0) {
        ctr |= MEC_BIT(MEC_I2C_CR_PIN_POS);
        ctx->i2c_ctrl_cached = ctr;
        mmcr8_wr(target_addr, rb + MEC_I2C_DATA_OFS);
        mmcr8_wr(ctr, rb + MEC_I2C_CR_OFS);
    } else {
        ctx->i2c_ctrl_cached = ctr;
        mmcr8_wr(ctr, rb + MEC_I2C_CR_OFS);
        mmcr8_wr(target_addr, rb + MEC_I2C_DATA_OFS);
    }

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_stop_gen(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint8_t ctrl = (MEC_BIT(MEC_I2C_CR_PIN_POS) | MEC_BIT(MEC_I2C_CR_ESO_POS)
                       | MEC_BIT(MEC_I2C_CR_ACK_POS) | MEC_BIT(MEC_I2C_CR_STO_POS));

    /* Nothing to do. Controller does not own the bus at this time */
    if (mmcr8_test_bit(rb + MEC_I2C_SR_OFS, MEC_I2C_SR_NBB_POS) != 0) {
        return MEC_RET_OK;
    }

    ctx->i2c_ctrl_cached = ctrl;
    mmcr8_wr(ctrl, rb + MEC_I2C_CR_OFS);

    return MEC_RET_OK;
}

/* Write byte to I2C.DATA for transmit
 * Prerequisites: Bus is owned by this controller and a START or Rpt-START plus
 * target write address has been sent by the controller.
 * If byte mode interrupts are required they should be enabled in the (Rpt)START
 * generation API.
 */
int mec_hal_i2c_smb_xmit_byte(struct mec_i2c_smb_ctx *ctx, uint8_t msg_byte)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    /* TODO should we check for NBB==1 and return error?
     * Only for data bytes. In the case of START we write target
     * address to I2C.DATA before writing I2C.Control.
     * If this function is only used for data then we can add the
     * check of NBB.
     */
    mmcr8_wr(msg_byte, ctx->regbase + MEC_I2C_DATA_OFS);

    return MEC_RET_OK;
}

/* Read byte currently in receive buffer and generate clocks for next
 * byte if CTRL.STO == 0.
 */
int mec_hal_i2c_smb_read_byte(struct mec_i2c_smb_ctx *ctx, uint8_t *msg_byte)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint8_t b = mmcr8_rd(rb + MEC_I2C_DATA_OFS);

    if (msg_byte != NULL) {
        *msg_byte = b;
    }

    return MEC_RET_OK;
}

int mec_hal_i2c_smb_bbctrl(struct mec_i2c_smb_ctx *ctx, uint8_t enable, uint8_t pin_drive)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint8_t bbctr = MEC_BIT(MEC_I2C_BBCR_CM_POS);

    if (enable != 0) {
        bbctr |= MEC_BIT(MEC_I2C_BBCR_EN_POS);
        if ((pin_drive & MEC_BIT(MEC_I2C_BB_SCL_POS)) != 0) { /* drive low? */
            bbctr |= MEC_BIT(MEC_I2C_BBCR_SCL_POS);
        }
        if ((pin_drive & MEC_BIT(MEC_I2C_BB_SDA_POS)) != 0) { /* drive low? */
            bbctr |= MEC_BIT(MEC_I2C_BBCR_SDA_POS);
        }
    }

    mmcr8_wr(bbctr, rb + MEC_I2C_BBCR_OFS);

    return MEC_RET_OK;
}

/* Read SCL and SDA pin states using bit-bang control register.
 * NOTE 1: Bit-bang mode must be enabled otherwise HW will return
 * both pin states as high.
 * NOTE 2: Enabling bit-bang switches the SCL and SDA lines away from
 * I2C logic to BB logic. When bit-bang is is disabled one must allow
 * time for I2C logic to resync to pins.
 */
uint8_t mec_hal_i2c_smb_bbctrl_pin_states(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0x3u;
    }
#endif

    uintptr_t rb = ctx->regbase;

    return ((mmcr8_rd(rb + MEC_I2C_BBCR_OFS) >> MEC_I2C_BBCR_SCL_IN_POS) & 0x3u);
}

/* -------- I2C-NL -------- */

#ifdef MEC_I2C_NL_DEBUG_SAVE_CM_CMD
static volatile uint32_t mec_i2c_nl_dbg_save[4];
#endif

int mec_hal_i2c_nl_cm_cfg_start(struct mec_i2c_smb_ctx *ctx, uint16_t ntx, uint16_t nrx,
                                uint32_t flags)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint32_t v = 0, cmd = 0;

    if (ntx == 0) { /* Any I2C transaction requires transmit of target address! */
        return MEC_RET_ERR_INVAL;
    }

    ctx->wrcnt = ntx;
    ctx->rdcnt = nrx;

    mmcr32_clr_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_HD_IEN_POS);
    
    if ((flags & MEC_I2C_NL_FLAG_FLUSH_BUF) != 0) {
        v = MEC_BIT(MEC_I2C_CFG_FHTX_POS) | MEC_BIT(MEC_I2C_CFG_FHRX_POS);
        mmcr32_set_bits(rb + MEC_I2C_CFG_OFS, v);
    }

    mmcr32_set_bit(rb + MEC_I2C_COMP_OFS, MEC_I2C_COMP_HDONE_POS);

    /* program bits[15:8] of TX and RX byte counts */
    v = ((ntx >> 8) & 0xffu) | (nrx & 0xff00u);
    mmcr32_wr(v, rb + MEC_I2C_ELEN_OFS);

    /* build host command register value */
    v = MEC_I2C_HC_WCL_SET((uint32_t)(ntx & 0xffu));
    v |= MEC_I2C_HC_RCL_SET((uint32_t)(nrx & 0xffu));
    v |= MEC_BIT(MEC_I2C_HC_RUN_POS) | MEC_BIT(MEC_I2C_HC_PROC_POS);
    
    if ((flags & MEC_I2C_NL_FLAG_START) != 0) {
        cmd |= MEC_BIT(MEC_I2C_HC_START0_POS);
    }

    if ((flags & MEC_I2C_NL_FLAG_RPT_START) != 0) {
        cmd |= MEC_BIT(MEC_I2C_HC_STARTN_POS);
    }

    if ((flags & MEC_I2C_NL_FLAG_STOP) != 0) {
        cmd |= MEC_BIT(MEC_I2C_HC_STOP_POS);
    }

    if ((flags & MEC_I2C_NL_FLAG_CM_DONE_IEN) != 0) {
        mmcr32_set_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_HD_IEN_POS);
    }

#ifdef MEC_I2C_NL_DEBUG_SAVE_CM_CMD
    mec_i2c_nl_dbg_save[0] = cmd;
    mec_i2c_nl_dbg_save[1] = mmcr32_rd(rb + MEC_I2C_CFG_OFS);
    mec_i2c_nl_dbg_save[2] = mmcr32_rd(rb + MEC_I2C_COMP_OFS);
    mec_i2c_nl_dbg_save[3] = mmcr32_rd(rb + MEC_I2C_ELEN_OFS);
#endif

    /* save CM_CMD b[15:0] in context */
    ctx->cmdctrl = (uint16_t)(cmd & 0xffffu);

    /* clear IDLE status just before starting HW
     * Only enable IDLE interrupt if I2C NBB bit is 0.
     */
    mmcr32_set_bit(rb + MEC_I2C_COMP_OFS, MEC_I2C_COMP_IDLE_POS);
    if (((flags & MEC_I2C_NL_FLAG_IDLE_IEN) != 0) &&
        (mmcr8_test_bit(rb + MEC_I2C_SR_OFS, MEC_I2C_SR_NBB_POS) == 0)) {
        mmcr32_set_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_IDLE_IEN_POS);
    }

    /* host or target command register must be written once.
     * I2C-NL will being processing the operation.
     * NOTE: due to I2C FSM checking state of SCL and SDA in time window
     * the FSM will not generate the I2C START immediately.
     */
    mmcr32_wr(cmd, rb + MEC_I2C_HC_OFS);

    return MEC_RET_OK;
}

int mec_hal_i2c_nl_cm_start(uintptr_t regbase, uint16_t ntx, uint16_t nrx, uint32_t flags,
                            uint32_t *cm_cmd_val)
{
    uint32_t cmd = 0, v = 0;

    if (regbase == 0) {
        return MEC_RET_ERR_INVAL;
    }

    /* disable Host done interrupt */
    mmcr32_clr_bit(regbase + MEC_I2C_CFG_OFS, MEC_I2C_CFG_HD_IEN_POS);

    if ((flags & MEC_I2C_NL_FLAG_FLUSH_BUF) != 0) {
        v = MEC_BIT(MEC_I2C_CFG_FHTX_POS) | MEC_BIT(MEC_I2C_CFG_FHRX_POS);
        mmcr32_set_bits(regbase + MEC_I2C_CFG_OFS, v);
    }

    /* clear CM R/W1C status bits */
    mmcr32_set_bits(regbase + MEC_I2C_COMP_OFS, MEC_I2C_COMP_RW1C_MSK);
 
    /* upper 8-bits of write and read counts */
    v = ((ntx >> 8) & 0xffu) | (nrx & 0xff00u);
    mmcr32_wr(v, regbase + MEC_I2C_ELEN_OFS);

    /* build host command register value */
    v = MEC_I2C_HC_WCL_SET((uint32_t)(ntx & 0xffu));
    v |= MEC_I2C_HC_RCL_SET((uint32_t)(nrx & 0xffu));
    v |= MEC_BIT(MEC_I2C_HC_RUN_POS) | MEC_BIT(MEC_I2C_HC_PROC_POS);
    
    if ((flags & MEC_I2C_NL_FLAG_START) != 0) {
        cmd |= MEC_BIT(MEC_I2C_HC_START0_POS);
    }

    if ((flags & MEC_I2C_NL_FLAG_RPT_START) != 0) {
        cmd |= MEC_BIT(MEC_I2C_HC_STARTN_POS);
    }

    if (flags & MEC_I2C_NL_FLAG_STOP) {
        cmd |= MEC_BIT(MEC_I2C_HC_STOP_POS);
    }

    if (flags & MEC_I2C_NL_FLAG_CM_DONE_IEN) {
        mmcr32_set_bit(regbase + MEC_I2C_CFG_OFS, MEC_I2C_CFG_HD_IEN_POS);
    }

    if (cm_cmd_val != NULL) {
        *cm_cmd_val = cmd;
    }

    /* write I2C CM_CMD triggering HW FSM to begin processing */
    mmcr32_wr(cmd, regbase + MEC_I2C_HC_OFS);

    return MEC_RET_OK;
}

int mec_hal_i2c_nl_cm_start_by_id(uint8_t i2c_ctrl_id, uint16_t ntx, uint16_t nrx, uint32_t flags,
                                  uint32_t *cm_cmd_val)
{
    uintptr_t regbase = 0;

    if (i2c_ctrl_id >= MEC5_I2C_SMB_INSTANCES) {
        return MEC_RET_ERR_INVAL;
    }

    regbase = MEC_I2C_SMB_BASE(i2c_ctrl_id);

    return mec_hal_i2c_nl_cm_start(regbase, ntx, nrx, flags, cm_cmd_val);
}

/* I2C-NL FSM clears MRUN and MPROCEED when both wrCnt and rdCnt transition to
 * 0. MRUN==1 and MPROCEED is cleared to 0 when FSM requires software to
 * reconfigure DMA for the direction change from write to read. After the
 * Rpt-Start and rdAddr are transmitted and (n)ACK'd the FSM clears MPROCEED
 * only. NOTE: any error should clear MRUN and MPROCEED.
 */
uint32_t mec_hal_i2c_nl_get_events(struct mec_i2c_smb_ctx *ctx, uint8_t is_tm)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint32_t cfg = 0, cmd = 0, sts = 0, events = 0, extlen = 0;
    uint32_t rdcnt = 0, wrcnt = 0;

    cmd = mmcr32_rd(rb + MEC_I2C_HC_OFS);
    if (is_tm != 0) {
        cmd = mmcr32_rd(rb + MEC_I2C_TC_OFS);
    }

    extlen = mmcr32_rd(rb + MEC_I2C_ELEN_OFS);
    wrcnt = ((extlen & 0xffu) << 8) | ((cmd >> 16) & 0xffu);
    rdcnt = (extlen & 0xff00u) | ((cmd >> 24) & 0xffu);

    cfg = mmcr32_rd(rb + MEC_I2C_CFG_OFS);
    sts = (mmcr32_rd(rb + MEC_I2C_COMP_OFS) & 0xffffff00u) |
        (mmcr8_rd(rb + MEC_I2C_SR_OFS) & 0xffu);

    if (((cfg & MEC_BIT(MEC_I2C_CFG_IDLE_IEN_POS)) != 0) &&
        ((sts & MEC_BIT(MEC_I2C_COMP_IDLE_POS)) != 0)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_IDLE_POS);
    }

    if (sts & MEC_BIT(MEC_I2C_SR_BER_POS)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_BERR_POS);
    }

    if (sts & MEC_BIT(MEC_I2C_SR_LAB_POS)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_LAB_POS);
    }

    if (sts & MEC_BIT(MEC_I2C_COMP_HNAKX_POS)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_NACK_POS);
    }

    /* Write-to-Read turn around is wrcnt==0, rdcnd!=0, and cmd[1:0]==01b */
    cmd &= (MEC_BIT(MEC_I2C_HC_RUN_POS) | MEC_BIT(MEC_I2C_HC_PROC_POS));
    if (cmd == MEC_BIT(MEC_I2C_HC_RUN_POS)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_PAUSE_POS);
    }

    if ((rdcnt == 0) && (wrcnt == 0) && (cmd == 0)) {
        events |= MEC_BIT(MEC_I2C_NL_EVENT_DONE_POS);
    }

    return events;
}

int mec_hal_i2c_nl_cmd_clear(struct mec_i2c_smb_ctx *ctx, uint8_t is_tm)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;
    uint32_t msk = 0;

    if (is_tm) {
        mmcr32_wr(0, rb + MEC_I2C_TC_OFS);
        msk = MEC_I2C_ELEN_TWRM_MSK | MEC_I2C_ELEN_TRDM_MSK;
    } else {
        mmcr32_wr(0, rb + MEC_I2C_HC_OFS);
        msk = MEC_I2C_ELEN_HWRM_MSK | MEC_I2C_ELEN_HRDM_MSK;
    }

    mmcr32_clr_bits(rb + MEC_I2C_ELEN_OFS, msk);

    return MEC_RET_OK;
}

/* I2C-NL has paused clearing proceed bit. Write RUN=PROC=1 to
 * continue processing.
 */
int mec_hal_i2c_nl_cm_proceed(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase + MEC_I2C_HC_OFS;
    uint32_t msk = MEC_BIT(MEC_I2C_HC_RUN_POS) | MEC_BIT(MEC_I2C_HC_PROC_POS);

    mmcr32_set_bits(rb, msk);

    return MEC_RET_OK;
}

int mec_hal_i2c_nl_tm_proceed(struct mec_i2c_smb_ctx *ctx)
{
#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase + MEC_I2C_TC_OFS;
    uint32_t msk = MEC_BIT(MEC_I2C_TC_RUN_POS) | MEC_BIT(MEC_I2C_TC_PROC_POS);

    mmcr32_set_bits(rb, msk);

    return MEC_RET_OK;
}

int mec_hal_i2c_nl_state_get(uintptr_t regbase, struct mec_i2c_smb_nl_state *state, uint8_t is_tm)
{
    uint32_t elen = 0, r = 0;
    uint16_t wrcnt = 0, rdcnt = 0;

#ifdef MEC_I2C_BASE_CHECK
    if (regbase == 0) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    if (state == NULL) {
        return MEC_RET_ERR_INVAL;
    }

    elen = mmcr32_rd(regbase + MEC_I2C_ELEN_OFS);

    if (is_tm) {
        r = mmcr32_rd(regbase + MEC_I2C_TC_OFS);
        state->ctrl = (uint16_t)(r & 0xffu);

        wrcnt = MEC_I2C_ELEN_TWRM_GET(elen);
        wrcnt <<= 8;
        wrcnt |= MEC_I2C_TC_WCL_GET(r);

        rdcnt = MEC_I2C_ELEN_TRDM_GET(elen);
        rdcnt <<= 8;
        rdcnt |= MEC_I2C_TC_RCL_GET(r);
    } else {
        r = mmcr32_rd(regbase + MEC_I2C_HC_OFS);
        state->ctrl = (uint16_t)(r & 0xffffu);
        
        wrcnt = MEC_I2C_ELEN_HWRM_GET(elen);
        wrcnt <<= 8;
        wrcnt |= MEC_I2C_HC_WCL_GET(r);

        rdcnt = MEC_I2C_ELEN_HRDM_GET(elen);
        rdcnt <<= 8;
        rdcnt |= MEC_I2C_HC_RCL_GET(r);
    }

    state->wrcnt = wrcnt;
    state->rdcnt = rdcnt;

    return MEC_RET_OK;
}

uint32_t mec_hal_i2c_nl_cm_xfr_count_get(uintptr_t regbase, uint8_t is_read)
{
    uint32_t cm = 0, cnt = 0;

#ifdef MEC_I2C_BASE_CHECK
    if (regbase == 0) {
        return cnt;
    }
#endif

    cm = mmcr32_rd(regbase + MEC_I2C_HC_OFS);
    cnt = mmcr32_rd(regbase + MEC_I2C_ELEN_OFS);

    if (is_read != 0) {
        cnt = MEC_I2C_ELEN_HWRM_GET(cnt);
        cnt <<= 8;
        cnt |= MEC_I2C_HC_RCL_GET(cm);
    } else {
        cnt = MEC_I2C_ELEN_HRDM_GET(cnt);
        cnt <<= 8;
        cnt |= MEC_I2C_HC_WCL_GET(cm);
    }

    return cnt;
}

/* Modify CM write or read byte count.
 * LSB's of the 16-bit counts are located in the CM_CMD register.
 * Bit[1:0] of this register should not be written with 0.
 * We access count LSB's using byte access.
 * write count LSB is in bits[15:8], read count LSB is in bits[32:26].
 */
int mec_hal_i2c_nl_cm_xfr_count_set(uintptr_t regbase, uint8_t is_read, uint32_t cnt)
{
    uint32_t cnt_lsb = 0, cnt_msb = 0;

#ifdef MEC_I2C_BASE_CHECK
    if (regbase == 0) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    if (cnt > MEC_I2C_SMB_NL_MAX_XFR_COUNT) {
        return MEC_RET_ERR_INVAL;
    }

    cnt_lsb = cnt & 0xffu;
    cnt_msb = (cnt >> 8) & 0xffu;

    /* Set MSB first in case LSB is 0 */
    if (is_read == 0) {
        cnt_lsb = MEC_I2C_HC_WCL_SET(cnt_lsb);
        cnt_msb = MEC_I2C_ELEN_HWRM_SET(cnt_msb);
        mmcr32_update_field(regbase + MEC_I2C_ELEN_OFS, cnt_msb, MEC_I2C_ELEN_HWRM_MSK);
        mmcr32_update_field(regbase + MEC_I2C_HC_OFS, cnt_lsb, MEC_I2C_HC_WCL_MSK);
    } else {
        cnt_lsb = MEC_I2C_HC_RCL_SET(cnt_lsb);
        cnt_msb = MEC_I2C_ELEN_HRDM_SET(cnt_msb);
        mmcr32_update_field(regbase + MEC_I2C_ELEN_OFS, cnt_msb, MEC_I2C_ELEN_HRDM_MSK);
        mmcr32_update_field(regbase + MEC_I2C_HC_OFS, cnt_lsb, MEC_I2C_HC_RCL_MSK);
    }

    return MEC_RET_OK;
}

uint32_t mec_hal_i2c_nl_cmd_get(struct mec_i2c_smb_ctx *ctx, uint8_t is_tm)
{
    uint32_t ofs = MEC_I2C_HC_OFS;

#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    if (is_tm != 0) {
        ofs = MEC_I2C_TC_OFS;
    }

    return mmcr32_rd(ctx->regbase + ofs);
}


/* ---- Target Mode Network Layer ---- */

/* Configure I2C controller's target command register
 * b[23:16] = Read count. This reflects the number of bytes we are receiving from
 * the external I2C Controller. This value is decremented by 1 for each by the
 * Central DMA channel reads from the TM Receive Buffer register.
 * Read count represents the buffer size the driver implements. The buffer should
 * be large enought to receive the message data plus possible Rpt-START address byte.
 * Read Count MSB is located in b[31:24] of the EXTLEN register.
 *
 * b[15:8] = Write count. The number of bytes this target will send to the external
 * I2C Controller. If this field and TM_PEC are 0 when the External controller requests
 * data then the current contents of the TM Transmit Buffer are used and the SPROT
 * status bit is set in the Completion register.
 * Write count MSB is located in b[23:16] of the EXTLEN register.
 *
 * b[2] = TM_PEC. If this bit is 1 when Write count is decrement to 0 then HW FSM will
 * copy contents of the PEC register to the I2C.Data register. HW FSM then clears the PEC
 * register and this bit.
 *
 * b[1] = TM_PROCEED
 * b[0] = TM_RUN
 *
 */
/* Flags used by CM cfg_start
 * #define MEC_I2C_NL_FLAG_START       0x01
 * #define MEC_I2C_NL_FLAG_RPT_START   0x02
 * #define MEC_I2C_NL_FLAG_STOP        0x04
 * #define MEC_I2C_NL_FLAG_CM_DONE_IEN 0x100u
 *
 * New flags for TM config?
 * TM_CMD has no start, rpt-start, or stop flags.
 * TM_MODE has
 * Completion register status bits
 *   TM_DONE
 *   Rpt-START on matching target write address
 *   Rpt-START on matching target read address
 *   TM_PROTOCOL_WR_CNT_ERROR
 *   TM_TR(RO) = 0 TM finished rx phase, 1 = finished tx phase
 *   TM_NAKR TM sent a NAK while it was receiving data
 *
 * Configuration register
 *   TM_DONE interrupt enable
 *   AAT interrupt enable
 */
int mec_hal_i2c_nl_tm_config(struct mec_i2c_smb_ctx *ctx, uint16_t ntx, uint16_t nrx,
                             uint32_t flags)
{
    uint32_t tm_cmd = 0;
    uint32_t tm_ien = 0;
    uint32_t elen = 0;
    uint32_t tm_ien_msk = (MEC_BIT(MEC_I2C_CFG_TD_IEN_POS) | MEC_BIT(MEC_I2C_CFG_AAT_IEN_POS));

#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    uintptr_t rb = ctx->regbase;

    ctx->wrcnt = ntx;
    ctx->rdcnt = nrx;

    mmcr32_clr_bits(rb + MEC_I2C_CFG_OFS, tm_ien_msk);
    mmcr32_wr(MEC_I2C_COMP_RW1C_MSK, rb + MEC_I2C_COMP_OFS);

    if ((flags & MEC_I2C_NL_TM_FLAG_DONE_IEN) != 0) {
        tm_ien |= MEC_BIT(MEC_I2C_CFG_TD_IEN_POS);
    }

    if ((flags & MEC_I2C_NL_TM_FLAG_AAT_IEN) != 0) {
        tm_ien |= MEC_BIT(MEC_I2C_CFG_AAT_IEN_POS);
    }

#ifdef MEC5_I2C_SMB_HAS_STOP_DETECT_IRQ
    if ((flags & MEC_I2C_NL_TM_FLAG_STOP_IEN) != 0) {
        tm_ien |= MEC_BIT(MEC_I2C_CFG_STD_NL_IEN_POS);
    }
#endif

    /* build target command register value */
    tm_cmd = MEC_I2C_TC_WCL_SET(((uint32_t)ntx & 0xffu));
    tm_cmd |= MEC_I2C_TC_RCL_SET(((uint32_t)nrx & 0xffu));
    elen = MEC_I2C_ELEN_TWRM_SET(((uint32_t)ntx >> 8) & 0xffu);
    elen |= MEC_I2C_ELEN_TRDM_SET(((uint32_t)nrx >> 8) & 0xffu);
    
    if ((flags & MEC_I2C_NL_TM_FLAG_RUN) != 0) {
        tm_cmd |= (MEC_BIT(MEC_I2C_TC_RUN_POS) | MEC_BIT(MEC_I2C_TC_PROC_POS));
    }

    /* save target command b[7:0]  in context */
    ctx->cmdctrl = (uint16_t)(tm_cmd & 0xffu);

    mmcr32_update_field(rb + MEC_I2C_ELEN_OFS, elen,
                        (MEC_I2C_ELEN_TWRM_MSK | MEC_I2C_ELEN_TRDM_MSK));
    mmcr32_wr(tm_cmd, rb + MEC_I2C_TC_OFS);
    mmcr32_set_bits(rb + MEC_I2C_CFG_OFS, tm_ien);

    return MEC_RET_OK;
}

uint32_t mec_hal_i2c_nl_tm_xfr_count_get(struct mec_i2c_smb_ctx *ctx, uint8_t is_rx)
{
    uint32_t tm_cmd = 0, cnt = 0;

#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    uintptr_t rb = ctx->regbase;

    cnt = mmcr32_rd(rb + MEC_I2C_ELEN_OFS);
    tm_cmd = mmcr32_rd(rb + MEC_I2C_TC_OFS);

    if (is_rx != 0) { /* External Controller transmits clocks and data. We capture(receive) data */
        cnt = MEC_I2C_ELEN_TRDM_GET(cnt);
        cnt <<= 8;
        cnt |= MEC_I2C_TC_RCL_GET(tm_cmd);
    } else { /* External Controller generates clocks and we supply(transmit) data */
        cnt = MEC_I2C_ELEN_TWRM_GET(cnt);
        cnt <<= 8;
        cnt |= MEC_I2C_TC_WCL_GET(tm_cmd);        
    }

    return cnt;
}

int mec_hal_i2c_nl_tm_xfr_count_set(uintptr_t regbase, uint8_t is_read, uint32_t cnt)
{
    uint32_t v = 0;

#ifdef MEC_I2C_BASE_CHECK
    if (regbase == 0) {
        return MEC_RET_ERR_INVAL;
    }
#endif

    if (cnt > MEC_I2C_SMB_NL_MAX_XFR_COUNT) {
        return MEC_RET_ERR_INVAL;
    }

    /* Set MSB first in case LSB is 0 */
    if (is_read != 0) {
        v = MEC_I2C_ELEN_TRDM_SET(cnt >> 8); 
        mmcr32_update_field(regbase + MEC_I2C_ELEN_OFS, v, MEC_I2C_ELEN_TRDM_MSK);
        v = MEC_I2C_TC_RCL_SET(cnt);
        mmcr32_update_field(regbase + MEC_I2C_TC_OFS, v, MEC_I2C_TC_RCL_MSK);
    } else {
        v = MEC_I2C_ELEN_TWRM_SET(cnt >> 8); 
        mmcr32_update_field(regbase + MEC_I2C_ELEN_OFS, v, MEC_I2C_ELEN_TWRM_MSK);
        v = MEC_I2C_TC_WCL_SET(cnt);
        mmcr32_update_field(regbase + MEC_I2C_TC_OFS, v, MEC_I2C_TC_WCL_MSK);
    }

    return MEC_RET_OK;
}

uint32_t mec_hal_i2c_nl_tm_transfered(struct mec_i2c_smb_ctx *ctx, uint8_t is_rx)
{
    uint32_t nxfr = 0, hwcnt = 0;

#ifdef MEC_I2C_BASE_CHECK
    if ((ctx == NULL) || (ctx->regbase == 0)) {
        return 0;
    }
#endif

    hwcnt = mec_hal_i2c_nl_tm_xfr_count_get(ctx, is_rx);

    if (is_rx != 0) {
        if (ctx->rdcnt >= hwcnt) {
            nxfr = ctx->rdcnt - hwcnt;
        }
    } else {
        if (ctx->wrcnt >= hwcnt) {
            nxfr = ctx->wrcnt - hwcnt;
        }
    }

    return nxfr;
}


/* ---- Power Management ----
 * Each controller has a wake enable interrupt on detection of an
 * external I2C START. This is only required if the controller is
 * being used in target mode.
 */

static uint32_t i2c_pm_save_buf;

/* Save and disable the controller */
void mec_hal_i2c_pm_save_disable(void)
{
    for (uint32_t i = 0; i < MEC5_I2C_SMB_INSTANCES; i++) {
        uintptr_t rb = i2c_instances[i].base_addr;

        if (mmcr32_test_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_ENAB_POS) != 0) {
            mmcr32_clr_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_ENAB_POS);
            i2c_pm_save_buf |= MEC_BIT(i);
        } else {
            i2c_pm_save_buf &= (uint32_t)~MEC_BIT(i);
        }
    }
}

void mec_hal_i2c_pm_restore(void)
{
    for (uint32_t i = 0; i < MEC5_I2C_SMB_INSTANCES; i++) {
        uintptr_t rb  = i2c_instances[i].base_addr;

        if ((i2c_pm_save_buf & MEC_BIT(i)) != 0) {
            mmcr32_set_bit(rb + MEC_I2C_CFG_OFS, MEC_I2C_CFG_ENAB_POS);
        }
    }
}

/* end mec_i2c.c */
